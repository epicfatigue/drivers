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
				Name:         driverName,
				Description:  "ORB I2C 14-bit ADC ORP driver (voltage -> ORP mV via offset calibration)",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			// NOTE: reef-pi hal doesn't have hal.Float in your versions,
			// so we declare float-like params as hal.String to stay compatible.
			parameters: []hal.ConfigParameter{
				{Name: addressParam, Type: hal.Integer, Order: 0, Default: 0x24},
				{Name: adcMaxParam, Type: hal.Integer, Order: 1, Default: 16383},
				{Name: vrefParam, Type: hal.String, Order: 2, Default: "3.3"},
				{Name: orpRefVoltageParam, Type: hal.String, Order: 3, Default: "1.8491"},
				{Name: orpRefmVParam, Type: hal.String, Order: 4, Default: "256"},
			},
		}
	})
	return factory
}

func (f *orbFactory) Metadata() hal.Metadata { return f.meta }

func (f *orbFactory) GetParameters() []hal.ConfigParameter { return f.parameters }

func (f *orbFactory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
	failures := make(map[string][]string)

	// helper
	req := func(name string) (interface{}, bool) {
		v, ok := parameters[name]
		if !ok {
			failures[name] = append(failures[name], fmt.Sprintf("%s is required but was not received.", name))
			return nil, false
		}
		return v, true
	}

	// Address
	if v, ok := req(addressParam); ok {
		val, ok := hal.ConvertToInt(v)
		if !ok {
			failures[addressParam] = append(failures[addressParam],
				fmt.Sprintf("%s is not a number. %v was received.", addressParam, v))
		} else if val <= 0 || val >= 256 {
			failures[addressParam] = append(failures[addressParam],
				fmt.Sprintf("%s is out of range (1-255). %v was received.", addressParam, v))
		}
	}

	// ADCMax
	if v, ok := req(adcMaxParam); ok {
		val, ok := hal.ConvertToInt(v)
		if !ok {
			failures[adcMaxParam] = append(failures[adcMaxParam],
				fmt.Sprintf("%s is not a number. %v was received.", adcMaxParam, v))
		} else if val <= 0 {
			failures[adcMaxParam] = append(failures[adcMaxParam],
				fmt.Sprintf("%s must be > 0. %v was received.", adcMaxParam, v))
		}
	}

	// Floats
	if v, ok := req(vrefParam); ok {
		if _, ok := convertToFloat(v); !ok {
			failures[vrefParam] = append(failures[vrefParam],
				fmt.Sprintf("%s is not a valid float. %v was received.", vrefParam, v))
		}
	}
	if v, ok := req(orpRefVoltageParam); ok {
		if _, ok := convertToFloat(v); !ok {
			failures[orpRefVoltageParam] = append(failures[orpRefVoltageParam],
				fmt.Sprintf("%s is not a valid float. %v was received.", orpRefVoltageParam, v))
		}
	}
	if v, ok := req(orpRefmVParam); ok {
		if _, ok := convertToFloat(v); !ok {
			failures[orpRefmVParam] = append(failures[orpRefmVParam],
				fmt.Sprintf("%s is not a valid float. %v was received.", orpRefmVParam, v))
		}
	}

	return len(failures) == 0, failures
}

func (f *orbFactory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	if valid, failures := f.ValidateParameters(parameters); !valid {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	intAddress, _ := hal.ConvertToInt(parameters[addressParam])
	address := byte(intAddress)

	intADCMax, _ := hal.ConvertToInt(parameters[adcMaxParam])

	vref, _ := convertToFloat(parameters[vrefParam])
	refV, _ := convertToFloat(parameters[orpRefVoltageParam])
	refmV, _ := convertToFloat(parameters[orpRefmVParam])

	bus := hardwareResources.(i2c.Bus)

	return &Orb{
		addr:       address,
		bus:        bus,
		adcMax:     intADCMax,
		vref:       vref,
		refVoltage: refV,
		refmV:      refmV,
		meta:       f.meta,
	}, nil
}

// convertToFloat handles what reef-pi passes in from JSON config/UI:
// float64, int, json.Number, string, etc.
func convertToFloat(v interface{}) (float64, bool) {
	switch t := v.(type) {
	case float64:
		return t, true
	case float32:
		return float64(t), true
	case int:
		return float64(t), true
	case int64:
		return float64(t), true
	case uint:
		return float64(t), true
	case uint64:
		return float64(t), true
	case json.Number:
		f, err := t.Float64()
		return f, err == nil
	case string:
		f, err := strconv.ParseFloat(t, 64)
		return f, err == nil
	default:
		return 0, false
	}
}
