package aliexpress_orp

import (
	"encoding/json"
	"errors"
	"log"
	"strconv"
	"strings"
	"sync"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

type factory struct {
	meta       hal.Metadata
	parameters []hal.ConfigParameter
}

const (
	addressParam = "Address" // integer 0..127; default 0x24 = 36
	vrefParam    = "Vref"
	offsetParam  = "Offset"
	debugParam   = "Debug"
)

var f *factory
var once sync.Once

func Factory() hal.DriverFactory {
	once.Do(func() {
		f = &factory{
			meta: hal.Metadata{
				Name:         driverName,
				Description:  "AliExpress I2C ADC module: electrode mV → ORP mV via software offset.",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			parameters: []hal.ConfigParameter{
				{Name: addressParam, Type: hal.Integer, Order: 0, Default: 36},
				{Name: vrefParam, Type: hal.Decimal, Order: 1, Default: 2.5},
				{Name: offsetParam, Type: hal.Decimal, Order: 2, Default: 0.0},
				{Name: debugParam, Type: hal.Boolean, Order: 3, Default: false},
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

	return len(failures) == 0, failures
}

func (f *factory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	if valid, failures := f.ValidateParameters(parameters); !valid {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	debug := getBoolAny(parameters, false, debugParam, "debug")

	if debug {
		if b, err := json.MarshalIndent(parameters, "", "  "); err == nil {
			log.Printf("aliexpress_orp NewDriver raw parameters:\n%s", string(b))
		}
	}

	addrInt := getIntAny(parameters, 36, addressParam, "address")
	vref := getFloatAny(parameters, 2.5, vrefParam, "vref")
	offset := getFloatAny(parameters, 0.0, offsetParam, "offset")

	d := &AliExpressORP{
		addr:   byte(addrInt),
		bus:    hardwareResources.(i2c.Bus),
		vrefV:  vref,
		offset: offset,
		debug:  debug,
		meta: hal.Metadata{
			Name:         driverName,
			Description:  "AliExpress I2C ADC module: electrode mV → ORP mV via offset",
			Capabilities: []hal.Capability{hal.AnalogInput},
		},
	}
	d.pins = []*orpPin{{parent: d, ch: 0}}

	if debug {
		log.Printf("aliexpress_orp init addr=%d (0x%02X) vref=%.3f offset=%.2f", addrInt, addrInt, vref, offset)
	}

	return d, nil
}

// ---------------- helpers (same style as your robotank factory) ----------------

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
