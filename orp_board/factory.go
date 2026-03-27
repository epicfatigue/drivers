package orp_board

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
	addressParam     = "Address"
	calibrationParam = "Calibration_mV"
	debugParam       = "Debug"
)

var f *factory
var once sync.Once

func Factory() hal.DriverFactory {
	once.Do(func() {
		f = &factory{
			meta: hal.Metadata{
				Name:        driverName,
				Description: "I2C ORP module: reads electrode millivolts over I2C ADC commands.",
				Capabilities: []hal.Capability{
					hal.AnalogInput,
				},
			},
			parameters: []hal.ConfigParameter{
				{
					Name:        addressParam,
					Type:        hal.Integer,
					Order:       0,
					Default:     0x45,
					Description: "I²C 7-bit address of the I2C ORP board.",
				},
				{
					Name:        calibrationParam,
					Type:        hal.Decimal,
					Order:       1,
					Default:     0.0,
					Description: "Observed ORP mV when probe is placed in a 256 mV calibration solution. Enter the measured value. Leave 0 to disable correction.",
				},
				{
					Name:        debugParam,
					Type:        hal.Boolean,
					Order:       2,
					Default:     false,
					Description: "Enable verbose debug logging for raw ADC and ORP millivolt values.",
				},
			},
		}
	})
	return f
}

func (f *factory) Metadata() hal.Metadata               { return f.meta }
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

	return len(failures) == 0, failures
}

func (f *factory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	if valid, failures := f.ValidateParameters(parameters); !valid {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	debug := getBoolAny(parameters, false, debugParam, "debug")

	if debug {
		if b, err := json.MarshalIndent(parameters, "", "  "); err == nil {
			log.Printf("orp_board_driver NewDriver raw parameters:\n%s", string(b))
		}
	}

	addrInt := getIntAny(parameters, 0x45, addressParam, "address")
	calibrationMV := getFloatAny(parameters, 0.0, calibrationParam, "calibration_mv", "orp_calibration_mv", "reference_mv")

	d := &orpDriver{
		addr:          byte(addrInt),
		bus:           hardwareResources.(i2c.Bus),
		vrefV:         2.048, // ADS1119 internal reference
		calibrationMV: calibrationMV,
		debug:         debug,
		meta: hal.Metadata{
			Name:         driverName,
			Description:  "I2C ORP module: electrode mV",
			Capabilities: []hal.Capability{hal.AnalogInput},
		},
	}

	d.pins = []*orpPin{{parent: d, ch: 0}}

	if debug {
		log.Printf("orp_board_driver init addr=%d (0x%02X) vref=%.3f calibrationMV=%.2f",
			addrInt, addrInt, d.vrefV, d.calibrationMV)
	}

	if err := d.initADC(); err != nil {
		return nil, err
	}

	return d, nil
}

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