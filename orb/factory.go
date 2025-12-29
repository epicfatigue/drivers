package orb

import (
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
				Description:  "ORB I2C ADC -> ORP(mV) driver",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			parameters: []hal.ConfigParameter{
				{Name: addressParam, Type: hal.Integer, Order: 0, Default: 0x24},
				{Name: adcMaxParam, Type: hal.Integer, Order: 1, Default: 16383},

				// hal.Float doesn't exist in your hal version, so store these as strings.
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
	case string:
		x, err := strconv.ParseFloat(t, 64)
		return x, err == nil
	default:
		// try fmt.Sprint for json.Number-ish or other representations
		s := fmt.Sprint(v)
		x, err := strconv.ParseFloat(s, 64)
		return x, err == nil
	}
}

func (f *orbFactory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
	failures := make(map[string][]string)

	// Address
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
			fmt.Sprintf("%s is a required parameter, but was not received.", addressParam))
	}

	// ADCMax
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
			fmt.Sprintf("%s is a required parameter, but was not received.", adcMaxParam))
	}

	// Vref
	if v, ok := parameters[vrefParam]; ok {
		if fv, ok := convertToFloat(v); !ok || fv <= 0 {
			failures[vrefParam] = append(failures[vrefParam],
				fmt.Sprintf("%s must be a float > 0. %v was received.", vrefParam, v))
		}
	} else {
		failures[vrefParam] = append(failures[vrefParam],
			fmt.Sprintf("%s is a required parameter, but was not received.", vrefParam))
	}

	// ORPRefVoltage
	if v, ok := parameters[orpRefVoltageParam]; ok {
		if fv, ok := convertToFloat(v); !ok {
			failures[orpRefVoltageParam] = append(failures[orpRefVoltageParam],
				fmt.Sprintf("%s must be a float. %v was received.", orpRefVoltageParam, v))
		} else if fv < 0 {
			failures[orpRefVoltageParam] = append(failures[orpRefVoltageParam],
				fmt.Sprintf("%s must be >= 0. %v was received.", orpRefVoltageParam, v))
		}
	} else {
		failures[orpRefVoltageParam] = append(failures[orpRefVoltageParam],
			fmt.Sprintf("%s is a required parameter, but was not received.", orpRefVoltageParam))
	}

	// ORPRefmV
	if v, ok := parameters[orpRefmVParam]; ok {
		if _, ok := convertToFloat(v); !ok {
			failures[orpRefmVParam] = append(failures[orpRefmVParam],
				fmt.Sprintf("%s must be a float. %v was received.", orpRefmVParam, v))
		}
	} else {
		failures[orpRefmVParam] = append(failures[orpRefmVParam],
			fmt.Sprintf("%s is a required parameter, but was not received.", orpRefmVParam))
	}

	return len(failures) == 0, failures
}

func (f *orbFactory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	if valid, failures := f.ValidateParameters(parameters); !valid {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	intAddr, _ := hal.ConvertToInt(parameters[addressParam])
	addr := byte(intAddr)

	adcMax, _ := hal.ConvertToInt(parameters[adcMaxParam])

	vref, _ := convertToFloat(parameters[vrefParam])
	orpRefVoltage, _ := convertToFloat(parameters[orpRefVoltageParam])
	orpRefmV, _ := convertToFloat(parameters[orpRefmVParam])

	bus := hardwareResources.(i2c.Bus)

	return &Driver{
		addr:          addr,
		bus:           bus,
		adcMax:        adcMax,
		vref:          vref,
		orpRefVoltage: orpRefVoltage,
		orpRefmV:      orpRefmV,
		meta:          f.meta,
	}, nil
}
