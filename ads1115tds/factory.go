// factory.go
//
// ADS1115 TDS driver factory.
//
// Exposes a single AnalogInput pin that reads one ADS1115 single-ended channel (AIN0..AIN3)
// Converts ADC counts -> volts -> (optional) volts@RefTempC -> linear TDS output
// Supports Snapshot() for the Chemistry snapshot + calibration wizard UI
//
// Added in this version:
// - DoTempComp checkbox (default false)
// - RefTempC numeric parameter (default 25.0)
// - Temperature injection hook: SetTemperatureC(tempC float64) on the pin
//
package ads1115tds

import (
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"strconv"
	"strings"
	"sync"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	// Factory params
	paramDebug      = "Debug"
	paramAddress    = "Address"
	paramChannel    = "Channel"
	paramGain       = "Gain"
	paramTdsK       = "TdsK"
	paramTdsOff     = "TdsOffset"
	paramClampV     = "ClampV"      // 3.3 or 5.0
	paramAlphaPer   = "AlphaPerC"   // e.g. 0.02
	paramDoTempComp = "DoTempComp"  // checkbox
	paramRefTempC   = "RefTempC"    // reference temperature for compensation
)

// Default alpha (typical conductivity temp coefficient)
const defaultAlphaPerC = 0.02

type factory struct {
	meta       hal.Metadata
	parameters []hal.ConfigParameter
}

var f *factory
var once sync.Once

func Factory() hal.DriverFactory {
	once.Do(func() {
		f = &factory{
			meta: hal.Metadata{
				Name:         driverName,
				Description:  "ADS1115 analog TDS (AINx -> volts -> optional temp-normalized volts@RefTempC -> linear TDS). Snapshot-capable.",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			parameters: []hal.ConfigParameter{
				{Name: paramDebug, Type: hal.Boolean, Order: 0, Default: false},
				{Name: paramAddress, Type: hal.String, Order: 1, Default: "0x48"},
				{Name: paramChannel, Type: hal.Integer, Order: 2, Default: 0},
				{Name: paramGain, Type: hal.String, Order: 3, Default: "1"},
				{Name: paramTdsK, Type: hal.Decimal, Order: 4, Default: 1.0},
				{Name: paramTdsOff, Type: hal.Decimal, Order: 5, Default: 0.0},

				// ClampV lets you match your ADC supply/reference assumptions.
				{Name: paramClampV, Type: hal.Decimal, Order: 6, Default: 3.3},

				// Alpha coefficient (typical is ~0.02). Only used when DoTempComp is enabled.
				{Name: paramAlphaPer, Type: hal.Decimal, Order: 7, Default: defaultAlphaPerC},

				// Temperature compensation controls
				{Name: paramRefTempC, Type: hal.Decimal, Order: 8, Default: 25.0},
				{Name: paramDoTempComp, Type: hal.Boolean, Order: 9, Default: false},
			},
		}
	})
	return f
}

func (f *factory) Metadata() hal.Metadata               { return f.meta }
func (f *factory) GetParameters() []hal.ConfigParameter { return f.parameters }

// ValidateParameters checks parameter values and returns per-key errors for the UI.
func (f *factory) ValidateParameters(p map[string]interface{}) (bool, map[string][]string) {
	fail := map[string][]string{}

	if v, ok := getAny(p, paramAddress, "address"); ok {
		if _, err := parseI2CAddress(v); err != nil {
			fail[paramAddress] = append(fail[paramAddress], err.Error())
		}
	}

	if v, ok := getAny(p, paramChannel, "channel"); ok {
		i, ok2 := hal.ConvertToInt(v)
		if !ok2 || i < 0 || i > 3 {
			fail[paramChannel] = append(fail[paramChannel], "must be 0..3 (AIN0..AIN3)")
		}
	}

	if v, ok := getAny(p, paramGain, "gain"); ok {
		if _, err := parseGain(v); err != nil {
			fail[paramGain] = append(fail[paramGain], err.Error())
		}
	}

	if v, ok := getAny(p, paramTdsK, "tdsk", "TDSK", "Tds_K", "tds_k"); ok {
		if _, err := convertToFloat(v); err != nil {
			fail[paramTdsK] = append(fail[paramTdsK], "must be a decimal number")
		}
	}

	if v, ok := getAny(p, paramTdsOff, "tdsoffset", "TDSOFFSET", "Tds_Offset", "tds_offset"); ok {
		if _, err := convertToFloat(v); err != nil {
			fail[paramTdsOff] = append(fail[paramTdsOff], "must be a decimal number")
		}
	}

	if v, ok := getAny(p, paramClampV, "clampv", "clamp_v"); ok {
		fv, err := convertToFloat(v)
		if err != nil {
			fail[paramClampV] = append(fail[paramClampV], "must be a number (e.g. 3.3 or 5)")
		} else if fv <= 0 || fv > 6 {
			fail[paramClampV] = append(fail[paramClampV], "must be in (0..6] volts")
		}
	}

	if v, ok := getAny(p, paramAlphaPer, "alphaperc", "alpha_per_c", "alpha"); ok {
		fv, err := convertToFloat(v)
		if err != nil {
			fail[paramAlphaPer] = append(fail[paramAlphaPer], "must be a number (e.g. 0.02)")
		} else if fv < 0 || fv > 0.1 {
			fail[paramAlphaPer] = append(fail[paramAlphaPer], "must be 0..0.1 (typical is ~0.02)")
		}
	}

	if v, ok := getAny(p, paramRefTempC, "reftempc", "ref_temp_c"); ok {
		fv, err := convertToFloat(v)
		if err != nil {
			fail[paramRefTempC] = append(fail[paramRefTempC], "must be a number (e.g. 25.0)")
		} else if fv < 0 || fv > 60 {
			fail[paramRefTempC] = append(fail[paramRefTempC], "must be 0..60 °C")
		}
	}

	// DoTempComp is bool; tolerate typical values. No strict validation needed.

	return len(fail) == 0, fail
}

func (f *factory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	if ok, failures := f.ValidateParameters(parameters); !ok {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	// Determine debug early so we can gate noisy logs
	debug := getBoolAny(parameters, false, paramDebug, "debug")

	// Only dump raw parameters when debug is enabled (keeps journal clean)
	if debug {
		if b, err := json.MarshalIndent(parameters, "", "  "); err == nil {
			log.Printf("ads1115tds NewDriver raw parameters:\n%s", string(b))
		}
	}

	bus, ok := hardwareResources.(i2c.Bus)
	if !ok {
		return nil, fmt.Errorf("ads1115tds: expected i2c.Bus as hardware resource, got %T", hardwareResources)
	}

	// Address default (0x48) unless overridden
	addr := byte(0x48)
	if v, ok := getAny(parameters, paramAddress, "address"); ok {
		a, err := parseI2CAddress(v)
		if err != nil {
			return nil, err
		}
		addr = a
	}

	// Channel default 0 unless overridden
	ch := 0
	if v, ok := getAny(parameters, paramChannel, "channel"); ok {
		if i, ok2 := hal.ConvertToInt(v); ok2 {
			ch = i
		}
	}

	mux, okMux := muxForChannel(ch)
	if !okMux {
		return nil, fmt.Errorf("ads1115tds: invalid channel %d (must be 0..3)", ch)
	}

	// Gain default 1 unless overridden
	gain := configGainOne
	if v, ok := getAny(parameters, paramGain, "gain"); ok {
		g, err := parseGain(v)
		if err != nil {
			return nil, err
		}
		gain = g
	}

	// Linear conversion coefficients
	tdsK := getFloatAny(parameters, 1.0, paramTdsK, "tdsk", "TDSK", "Tds_K", "tds_k")
	tdsOff := getFloatAny(parameters, 0.0, paramTdsOff, "tdsoffset", "TDSOFFSET", "Tds_Offset", "tds_offset")

	// Clamp voltage (3.3V or 5V typically)
	clampV := getFloatAny(parameters, 3.3, paramClampV, "clampv", "clamp_v")

	// Temperature coefficient (used only when DoTempComp=true)
	alpha := getFloatAny(parameters, defaultAlphaPerC, paramAlphaPer, "alphaperc", "alpha_per_c", "alpha")

	// Temp compensation controls
	refTempC := getFloatAny(parameters, 25.0, paramRefTempC, "reftempc", "ref_temp_c")
	doTempComp := getBoolAny(parameters, false, paramDoTempComp, "dotempcomp", "do_tc", "dotc")

	if debug {
		fs, _ := fsVoltsForGain(gain)
		log.Printf("ads1115tds resolved config: addr=0x%02X ch=%d mux=0x%04X gain=0x%04X fs=%.6fV k=%.9f off=%.9f clampV=%.3f alpha=%.4f DoTC=%v RefTempC=%.2f debug=%v",
			addr, ch, mux, gain, fs, tdsK, tdsOff, clampV, alpha, doTempComp, refTempC, debug)
	}

	pin := newTdsChannel(
		bus, addr, ch, mux, gain,
		tdsK, tdsOff,
		clampV,
		alpha,
		doTempComp,
		refTempC,
		debug,
		f.meta,
	)

	// Keep a one-line init log (useful even when debug=false)
	log.Printf("ads1115tds init addr=0x%02X ch=%d gain=0x%04X k=%.6f off=%.6f clampV=%.3f alpha=%.4f DoTC=%v RefTempC=%.2f debug=%v",
		addr, ch, gain, tdsK, tdsOff, clampV, alpha, doTempComp, refTempC, debug)

	return &Driver{
		meta: f.meta,
		pin:  pin,
	}, nil
}

// ---------- parsing helpers ----------

// parseI2CAddress accepts "0x48" or int-like values and returns a 7-bit address.
func parseI2CAddress(v interface{}) (byte, error) {
	switch t := v.(type) {
	case string:
		s := strings.TrimSpace(t)
		base := 10
		if strings.HasPrefix(s, "0x") || strings.HasPrefix(s, "0X") {
			base = 0
		}
		n64, err := strconv.ParseInt(s, base, 32)
		if err != nil {
			return 0, fmt.Errorf("Address must be int or hex string like 0x48: %v", err)
		}
		if n64 < 0 || n64 > 127 {
			return 0, fmt.Errorf("Address out of range (0..127): %d", n64)
		}
		return byte(n64), nil
	default:
		if i, ok := hal.ConvertToInt(v); ok {
			if i < 0 || i > 127 {
				return 0, fmt.Errorf("Address out of range (0..127): %d", i)
			}
			return byte(i), nil
		}
		return 0, fmt.Errorf("Address must be int or hex string like 0x48")
	}
}

// parseGain accepts "2/3", "1", "2", "4", "8", "16" or int 0..5.
func parseGain(v interface{}) (uint16, error) {
	if s, ok := v.(string); ok {
		switch strings.TrimSpace(s) {
		case "2/3":
			return configGainTwoThirds, nil
		case "1":
			return configGainOne, nil
		case "2":
			return configGainTwo, nil
		case "4":
			return configGainFour, nil
		case "8":
			return configGainEight, nil
		case "16":
			return configGainSixteen, nil
		default:
			if n, err := strconv.Atoi(s); err == nil {
				return parseGain(n)
			}
			return 0, fmt.Errorf("Gain must be one of: 2/3,1,2,4,8,16")
		}
	}

	if n, ok := hal.ConvertToInt(v); ok {
		switch n {
		case 0:
			return configGainTwoThirds, nil
		case 1:
			return configGainOne, nil
		case 2:
			return configGainTwo, nil
		case 3:
			return configGainFour, nil
		case 4:
			return configGainEight, nil
		case 5:
			return configGainSixteen, nil
		default:
			return 0, fmt.Errorf("Gain int must be 0..5")
		}
	}

	return 0, fmt.Errorf("Gain must be string (2/3,1,2,4,8,16) or int (0..5)")
}

// --- alias/tolerant conversions ---

// getAny fetches parameter values with multiple key aliases (case-insensitive).
func getAny(m map[string]interface{}, keys ...string) (interface{}, bool) {
	for _, k := range keys {
		if v, ok := m[k]; ok {
			return unwrapValue(v), true
		}
		for kk, vv := range m {
			if strings.EqualFold(kk, k) {
				return unwrapValue(vv), true
			}
		}
	}
	return nil, false
}

// getFloatAny returns a float value if present, otherwise def.
func getFloatAny(m map[string]interface{}, def float64, keys ...string) float64 {
	v, ok := getAny(m, keys...)
	if !ok {
		return def
	}
	f, err := convertToFloat(v)
	if err != nil {
		return def
	}
	return f
}

// getBoolAny returns a bool if present, otherwise def.
func getBoolAny(m map[string]interface{}, def bool, keys ...string) bool {
	v, ok := getAny(m, keys...)
	if !ok {
		return def
	}
	switch t := v.(type) {
	case bool:
		return t
	case string:
		s := strings.ToLower(strings.TrimSpace(t))
		return (s == "1" || s == "true" || s == "yes" || s == "y" || s == "on")
	case int:
		return t != 0
	case int64:
		return t != 0
	case float64:
		return t != 0
	default:
		return def
	}
}

// unwrapValue allows older/odd parameter payload shapes like {value: X}.
func unwrapValue(v interface{}) interface{} {
	if m, ok := v.(map[string]interface{}); ok {
		for _, k := range []string{"value", "Value", "current", "Current", "val", "Val"} {
			if vv, ok := m[k]; ok {
				return vv
			}
		}
	}
	return v
}

// convertToFloat converts common JSON-ish numeric types into float64.
func convertToFloat(v interface{}) (float64, error) {
	switch t := v.(type) {
	case float64:
		return t, nil
	case float32:
		return float64(t), nil
	case int:
		return float64(t), nil
	case int64:
		return float64(t), nil
	case json.Number:
		return t.Float64()
	case string:
		return strconv.ParseFloat(strings.TrimSpace(t), 64)
	default:
		if i, ok := hal.ConvertToInt(v); ok {
			return float64(i), nil
		}
		return 0, fmt.Errorf("not a number")
	}
}
