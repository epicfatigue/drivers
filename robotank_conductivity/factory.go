package robotank_conductivity

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

// Canonical parameter names (what YOUR code uses)
// Note: we keep AbsD_53000 as the UI name for backwards compatibility,
// but the driver treats it as "AbsD_Std" (standard point) and RefUS is FIXED at 53000.
const (
	addressParam    = "Address"
	absDFreshParam  = "AbsD_Fresh"
	absDStdParamUI  = "AbsD_53000" // kept for compatibility with existing configs/UI
	refTempCParam   = "RefTempC"
	doTempCompParam = "DoTempComp"
	debugParam      = "Debug"
	delayMsParam    = "DelayMs" // optional if your fork sends it
)

var f *factory
var once sync.Once

func Factory() hal.DriverFactory {
	once.Do(func() {
		f = &factory{
			meta: hal.Metadata{
				Name:         driverName,
				Description:  "Robo-Tank conductivity circuit (µS/cm + ppt)",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			// This parameter list is what reef-pi UI renders for NEW configs.
			// We intentionally DO NOT expose RefUS / AlphaPerC / TempC (fixed / injected).
			parameters: []hal.ConfigParameter{
				{Name: addressParam, Type: hal.Integer, Order: 0, Default: 106},

				{Name: absDFreshParam, Type: hal.Decimal, Order: 1, Default: 983},
				{Name: absDStdParamUI, Type: hal.Decimal, Order: 2, Default: 21},

				{Name: refTempCParam, Type: hal.Decimal, Order: 3, Default: 25.0},
				{Name: doTempCompParam, Type: hal.Boolean, Order: 4, Default: false},

				{Name: delayMsParam, Type: hal.Integer, Order: 5, Default: 200},
				{Name: debugParam, Type: hal.Boolean, Order: 6, Default: false},
			},
		}
	})
	return f
}

func (f *factory) Metadata() hal.Metadata { return f.meta }
func (f *factory) GetParameters() []hal.ConfigParameter {
	return f.parameters
}

func (f *factory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
	failures := make(map[string][]string)

	// Address required (support aliases too)
	address, ok := getAny(parameters, addressParam, "address")
	if !ok {
		failures[addressParam] = append(failures[addressParam], "Address parameter is required")
		return false, failures
	}

	val, ok := toInt(address)
	if !ok {
		failures[addressParam] = append(failures[addressParam], "Address must be an integer")
	} else if val < 0 || val > 127 {
		failures[addressParam] = append(failures[addressParam], "Address must be 0..127 (7-bit)")
	}

	return len(failures) == 0, failures
}

func (f *factory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	if valid, failures := f.ValidateParameters(parameters); !valid {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	// Debug: shows the EXACT keys your fork is passing.
	if b, err := json.MarshalIndent(parameters, "", "  "); err == nil {
		log.Printf("robotank_cond NewDriver raw parameters:\n%s", string(b))
	}

	addrRaw, _ := getAny(parameters, addressParam, "address")
	addrInt, _ := toInt(addrRaw)

	absFresh := getFloatAny(parameters, 0,
		absDFreshParam, // "AbsD_Fresh"
		"Absd_fresh",   // your fork
		"Absd_Fresh",
		"absd_fresh",
	)

	// UI name is AbsD_53000, but we treat it as AbsD_Std (standard point @ fixedRefUS)
	absStd := getFloatAny(parameters, 0,
		absDStdParamUI, // "AbsD_53000"
		"Absd_53000",   // your fork
		"absd_53000",
		"AbsD_Std",
		"Absd_std",
		"absd_std",
	)

	refTempC := getFloatAny(parameters, 25.0,
		refTempCParam, // "RefTempC"
		"Reftempc",    // your fork
		"reftempc",
	)

	doTempComp := getBoolAny(parameters, false,
		doTempCompParam, // "DoTempComp"
		"Dotempcomp",
		"DoTC",
		"Dotc",
	)

	debug := getBoolAny(parameters, false,
		debugParam,
		"debug",
	)

	delayMs := getIntAny(parameters, 200,
		delayMsParam, // "DelayMs"
		"Delayms",    // your fork
		"delayms",
	)

	// Fixed constants (NOT configurable)
	refUS := fixedRefUS
	alphaPerC := fixedAlphaPerC

	// Initialize tempC to refTempC until reef-pi injects a real value
	tempC := refTempC

	d := &RoboTankConductivity{
		addr:       byte(addrInt),
		bus:        hardwareResources.(i2c.Bus),
		delay:      time.Duration(delayMs) * time.Millisecond,
		absDFresh:  absFresh,
		absDStd:    absStd,
		refUS:      refUS,
		refTempC:   refTempC,
		alphaPerC:  alphaPerC,
		tempC:      tempC,
		doTempComp: doTempComp,
		debug:      debug,
		meta: hal.Metadata{
			Name:         driverName,
			Description:  "Robo-Tank conductivity circuit (µS/cm + ppt)",
			Capabilities: []hal.Capability{hal.AnalogInput},
		},
	}

	d.pins = []*rtPin{
		{parent: d, ch: 0}, // uS
		{parent: d, ch: 1}, // ppt
	}

	log.Printf("robotank_cond init addr=%d AbsD_Fresh=%.3f AbsD_Std=%.3f RefUS=%.1f(fixed) DoTC=%v TempC=%.2f(init) RefTempC=%.2f Alpha=%.4f(fixed) Delay=%v Debug=%v",
		d.addr, d.absDFresh, d.absDStd, d.refUS, d.doTempComp, d.tempC, d.refTempC, d.alphaPerC, d.delay, d.debug)

	return d, nil
}

// ----------------- helpers -----------------

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
	// tolerate floats-as-ints
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
		s = strings.ReplaceAll(s, ",", ".")
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
