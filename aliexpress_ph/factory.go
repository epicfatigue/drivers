package aliexpress_ph

import (
	"encoding/json"
	"errors"
	"log"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

type factory struct {
	meta       hal.Metadata
	parameters []hal.ConfigParameter
}

// Parameter names (UI + config)
const (
	addressParam       = "Address"      // 0x14,0x15,0x17,0x24(default),0x26,0x27
	vrefParam          = "Vref"         // 2.5 typical
	ph7mVParam         = "PH7_mV"
	ph4mVParam         = "PH4_mV"
	ph10mVParam        = "PH10_mV"
	slopeOverrideParam = "Slope_mV_pH"  // optional
	refTempCParam      = "RefTempC"     // reference for temp comp (25)
	doTempCompParam    = "DoTempComp"   // disabled by default
	debugParam         = "Debug"
)

var f *factory
var once sync.Once

func Factory() hal.DriverFactory {
	once.Do(func() {
		f = &factory{
			meta: hal.Metadata{
				Name:         driverName,
				Description:  "AliExpress I2C ADC module: reads raw electrode mV, converts to pH via calibration anchors.",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			parameters: []hal.ConfigParameter{
				// Address: accept integer 0..127. Default is 0x24 = 36.
				{Name: addressParam, Type: hal.Integer, Order: 0, Default: 36},

				{Name: vrefParam, Type: hal.Decimal, Order: 1, Default: 2.5},

				// Anchors in mV (entered from calibration wizard / snapshot observed_mv)
				{Name: ph7mVParam, Type: hal.Decimal, Order: 2, Default: 0.0},
				{Name: ph4mVParam, Type: hal.Decimal, Order: 3, Default: 0.0},
				{Name: ph10mVParam, Type: hal.Decimal, Order: 4, Default: 0.0},

				{Name: slopeOverrideParam, Type: hal.Decimal, Order: 5, Default: 0.0},

				{Name: refTempCParam, Type: hal.Decimal, Order: 6, Default: 25.0},
				{Name: doTempCompParam, Type: hal.Boolean, Order: 7, Default: false},

				{Name: debugParam, Type: hal.Boolean, Order: 8, Default: false},
			},
		}
	})
	return f
}

func (f *factory) Metadata() hal.Metadata { return f.meta }
func (f *factory) GetParameters() []hal.ConfigParameter { return f.parameters }

func (f *factory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
	failures := make(map[string][]string)

	addrRaw, ok := getAny(parameters, addressParam, "address")
	if !ok {
		failures[addressParam] = append(failures[addressParam], "Address parameter is required")
	} else {
		addr, ok := toInt(addrRaw)
		if !ok {
			failures[addressParam] = append(failures[addressParam], "Address must be an integer (0..127)")
		} else if addr < 0 || addr > 127 {
			failures[addressParam] = append(failures[addressParam], "Address must be 0..127 (7-bit)")
		}
	}

	vref := getFloatAny(parameters, 2.5, vrefParam, "vref")
	if vref <= 0 || vref > 5.5 {
		failures[vrefParam] = append(failures[vrefParam], "Vref must be >0 and reasonable (e.g. 2.5)")
	}

	// We don't hard-require PH7 here because some users truly see ~0mV in pH7,
	// but having PH7 anchor configured is strongly recommended.
	_ = getFloatAny(parameters, 0, ph7mVParam, "ph7_mv")

	return len(failures) == 0, failures
}

func (f *factory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	if valid, failures := f.ValidateParameters(parameters); !valid {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	debug := getBoolAny(parameters, false, debugParam, "debug")

	// Debug: show EXACT keys being passed
	if debug {
		if b, err := json.MarshalIndent(parameters, "", "  "); err == nil {
			log.Printf("aliexpress_ph NewDriver raw parameters:\n%s", string(b))
		}
	}

	addrInt := getIntAny(parameters, 36, addressParam, "address")
	vref := getFloatAny(parameters, 2.5, vrefParam, "vref")

	ph7 := getFloatAny(parameters, 0.0, ph7mVParam, "ph7_mv")
	ph4 := getFloatAny(parameters, 0.0, ph4mVParam, "ph4_mv")
	ph10 := getFloatAny(parameters, 0.0, ph10mVParam, "ph10_mv")

	slopeOverride := getFloatAny(parameters, 0.0, slopeOverrideParam, "slope")
	refTempC := getFloatAny(parameters, 25.0, refTempCParam, "reftempc")
	doTempComp := getBoolAny(parameters, false, doTempCompParam, "dotempcomp", "dotc")

	d := &AliExpressPH{
		addr:          byte(addrInt),
		bus:           hardwareResources.(i2c.Bus),
		vrefV:         vref,
		ph7mV:         ph7,
		ph4mV:         ph4,
		ph10mV:        ph10,
		slopeOverride: slopeOverride,
		refTempC:      refTempC,
		doTempComp:    doTempComp,
		tempC:         refTempC, // initialize temp to ref until injected
		debug:         debug,
		meta: hal.Metadata{
			Name:         driverName,
			Description:  "AliExpress I2C ADC module: electrode mV → pH via anchors",
			Capabilities: []hal.Capability{hal.AnalogInput},
		},
	}

	d.pins = []*phPin{{parent: d, ch: 0}}

	if debug {
		log.Printf("aliexpress_ph init addr=%d (0x%02X) vref=%.3f PH7=%.2f PH4=%.2f PH10=%.2f slope_override=%.4f DoTC=%v RefTempC=%.2f tempC(init)=%.2f",
			addrInt, addrInt, vref, ph7, ph4, ph10, slopeOverride, doTempComp, refTempC, d.tempC)
	}

	// Small delay is not required for this module (pure read), but keep time import used in this file.
	_ = time.Millisecond

	return d, nil
}

// ----------------- helpers (same style as your robotank factory) -----------------

func getAny(m map[string]interface{}, keys ...string) (interface{}, bool) {
	for _, k := range keys {
		if v, ok := m[k]; ok {
			return unwrapValue(v), true
		}
	}
	return nil, false
}

func getFloatAny(m map[string]interface{}, def float64, keys ...string) float64 {
	v, ok := getAny(m, keys...)
	if !ok {
		return def
	}
	if f, ok := toFloat(v); ok {
		return f
	}
	return def
}

func getIntAny(m map[string]interface{}, def int, keys ...string) int {
	v, ok := getAny(m, keys...)
	if !ok {
		return def
	}
	if i, ok := toInt(v); ok {
		return i
	}
	if f, ok := toFloat(v); ok {
		return int(f)
	}
	return def
}

func getBoolAny(m map[string]interface{}, def bool, keys ...string) bool {
	v, ok := getAny(m, keys...)
	if !ok {
		return def
	}
	if b, ok := toBool(v); ok {
		return b
	}
	return def
}

func toInt(v interface{}) (int, bool) {
	v = unwrapValue(v)
	switch t := v.(type) {
	case int:
		return t, true
	case int64:
		return int(t), true
	case float64:
		return int(t), true
	case json.Number:
		i, err := t.Int64()
		if err != nil {
			return 0, false
		}
		return int(i), true
	case string:
		s := strings.TrimSpace(t)
		// allow "0x24"
		if strings.HasPrefix(strings.ToLower(s), "0x") {
			u, err := strconv.ParseUint(s[2:], 16, 8)
			if err == nil {
				return int(u), true
			}
		}
		if i, err := strconv.Atoi(s); err == nil {
			return i, true
		}
		if f, err := strconv.ParseFloat(s, 64); err == nil {
			return int(f), true
		}
		return 0, false
	default:
		return 0, false
	}
}

func toFloat(v interface{}) (float64, bool) {
	v = unwrapValue(v)
	switch t := v.(type) {
	case float64:
		return t, true
	case float32:
		return float64(t), true
	case int:
		return float64(t), true
	case int64:
		return float64(t), true
	case json.Number:
		f, err := t.Float64()
		if err != nil {
			return 0, false
		}
		return f, true
	case string:
		s := strings.TrimSpace(t)
		// tolerate commas
		s = strings.ReplaceAll(s, ",", ".")
		f, err := strconv.ParseFloat(s, 64)
		if err != nil {
			return 0, false
		}
		return f, true
	default:
		return 0, false
	}
}

func toBool(v interface{}) (bool, bool) {
	v = unwrapValue(v)
	switch t := v.(type) {
	case bool:
		return t, true
	case int:
		return t != 0, true
	case int64:
		return t != 0, true
	case float64:
		return t != 0, true
	case string:
		s := strings.ToLower(strings.TrimSpace(t))
		switch s {
		case "1", "true", "yes", "y", "on":
			return true, true
		case "0", "false", "no", "n", "off":
			return false, true
		default:
			return false, false
		}
	default:
		return false, false
	}
}

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
