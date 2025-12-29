package orb

import (
	"encoding/json"
	"errors"
	"fmt"
	"strconv"
	"sync"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	addressParam       = "Address"
	adcMaxParam        = "ADCMax"
	vrefParam          = "Vref"
	orpRefVoltageParam = "ORPRefVoltage"
	orpRefmVParam      = "ORPRefmV"
)

type orbFactory struct {
	meta       hal.Metadata
	parameters []hal.ConfigParameter
}

var factory *orbFactory
var once sync.Once

func Factory() hal.DriverFactory {
	once.Do(func() {
		factory = &orbFactory{
			meta: hal.Metadata{
				Name:         "orb",
				Description:  "ORB ORP (mV) driver using onboard ADC over I2C",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			parameters: []hal.ConfigParameter{
				// ints
				{Name: addressParam, Type: hal.Integer, Order: 0, Default: 0x24},
				{Name: adcMaxParam, Type: hal.Integer, Order: 1, Default: 16383},

				// floats (hal has no Float type in your version, so use String)
				{Name: vrefParam, Type: hal.String, Order: 2, Default: "3.3"},
				{Name: orpRefVoltageParam, Type: hal.String, Order: 3, Default: "1.8491"},
				{Name: orpRefmVParam, Type: hal.String, Order: 4, Default: "256"},
			},
		}
	})
	return factory
}

func (f *orbFactory) Metadata() hal.Metadata {
	return f.meta
}

func (f *orbFactory) GetParameters() []hal.ConfigParameter {
	return f.parameters
}

func convertToFloat64(v interface{}) (float64, bool) {
	switch t := v.(type) {
	case float64:
		return t, true
	case float32:
		return float64(t), true
	case int:
		return float64(t), true
	case int8:
		return float64(t), true
	case int16:
		return float64(t), true
	case int32:
		return float64(t), true
	case int64:
		return float64(t), true
	case uint:
		return float64(t), true
	case uint8:
		return float64(t), true
	case uint16:
		return float64(t), true
	case uint32:
		return float64(t), true
	case uint64:
		return float64(t), true
	case string:
		fv, err := strconv.ParseFloat(t, 64)
		return fv, err == nil
	case json.Number:
		fv, err := t.Float64()
		return fv, err == nil
	default:
		return 0, false
	}
}

func (f *orbFactory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
	failures := make(map[string][]string)

	// Address (required, 1-255)
	if v, ok := parameters[addressParam]; ok {
		val, ok := hal.ConvertToInt(v)
		if !ok {
			failures[addressParam] = append(failures[addressParam],
				fmt.Sprintf("%s is not a number. %v was received.", addressParam, v))
		} else if val <= 0 || val >= 256 {
			failures[addressParam] = append(failures[addressParam],
				fmt.Sprintf("%s is out of range (1 - 255). %v was received.", addressParam, v))
		}
	} else {
		failures[addressParam] = append(failures[addressParam],
			fmt.Sprintf("%s is required but was not received.", addressParam))
	}

	// ADCMax (required, sane range)
	if v, ok := parameters[adcMaxParam]; ok {
		val, ok := hal.ConvertToInt(v)
		if !ok {
			failures[adcMaxParam] = append(failures[adcMaxParam],
				fmt.Sprintf("%s is not a number. %v was received.", adcMaxParam, v))
		} else if val <= 0 {
			failures[adcMaxParam] = append(failures[adcMaxParam],
				fmt.Sprintf("%s must be > 0. %v was received.", adcMaxParam, v))
		}
	} else {
		failures[adcMaxParam] = append(failures[adcMaxParam],
			fmt.Sprintf("%s is required but was not received.", adcMaxParam))
	}

	// Vref (required float > 0)
	if v, ok := parameters[vrefParam]; ok {
		val, ok := convertToFloat64(v)
		if !ok {
			failures[vrefParam] = append(failures[vrefParam],
				fmt.Sprintf("%s is not a float. %v was received.", vrefParam, v))
		} else if val <= 0 {
			failures[vrefParam] = append(failures[vrefParam],
				fmt.Sprintf("%s must be > 0. %v was received.", vrefParam, v))
		}
	} else {
		failures[vrefParam] = append(failures[vrefParam],
			fmt.Sprintf("%s is required but was not received.", vrefParam))
	}

	// ORPRefVoltage (required float)
	if v, ok := parameters[orpRefVoltageParam]; ok {
		if _, ok := convertToFloat64(v); !ok {
			failures[orpRefVoltageParam] = append(failures[orpRefVoltageParam],
				fmt.Sprintf("%s is not a float. %v was received.", orpRefVoltageParam, v))
		}
	} else {
		failures[orpRefVoltageParam] = append(failures[orpRefVoltageParam],
			fmt.Sprintf("%s is required but was not received.", orpRefVoltageParam))
	}

	// ORPRefmV (required float)
	if v, ok := parameters[orpRefmVParam]; ok {
		if _, ok := convertToFloat64(v); !ok {
			failures[orpRefmVParam] = append(failures[orpRefmVParam],
				fmt.Sprintf("%s is not a float. %v was received.", orpRefmVParam, v))
		}
	} else {
		failures[orpRefmVParam] = append(failures[orpRefmVParam],
			fmt.Sprintf("%s is required but was not received.", orpRefmVParam))
	}

	return len(failures) == 0, failures
}

func (f *orbFactory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	if valid, failures := f.ValidateParameters(parameters); !valid {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	intAddress, _ := hal.ConvertToInt(parameters[addressParam])
	address := byte(intAddress)

	adcMax, _ := hal.ConvertToInt(parameters[adcMaxParam])

	vref, _ := convertToFloat64(parameters[vrefParam])
	orpRefV, _ := convertToFloat64(parameters[orpRefVoltageParam])
	orpRefmV, _ := convertToFloat64(parameters[orpRefmVParam])

	bus := hardwareResources.(i2c.Bus)

	// your orb driver should already exist (orb.go) and store these fields
	return &ORB{
		addr:          address,
		bus:           bus,
		adcMax:        adcMax,
		vref:          vref,
		orpRefVoltage: orpRefV,
		orpRefmV:      orpRefmV,
		meta:          f.meta,
	}, nil
}
