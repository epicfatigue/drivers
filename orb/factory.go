package orb

import (
	"errors"
	"fmt"
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
				Description:  "ORB I2C ADC module (14-bit) converted to ORP (mV) using offset calibration",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			parameters: []hal.ConfigParameter{
				{Name: addressParam, Type: hal.Integer, Order: 0, Default: 0x24},
				{Name: adcMaxParam, Type: hal.Integer, Order: 1, Default: 16383},
				{Name: vrefParam, Type: hal.Float, Order: 2, Default: 3.3},
				{Name: orpRefVoltageParam, Type: hal.Float, Order: 3, Default: 1.8491},
				{Name: orpRefmVParam, Type: hal.Float, Order: 4, Default: 256.0},
			},
		}
	})
	return factory
}

func (f *orbFactory) Metadata() hal.Metadata { return f.meta }

func (f *orbFactory) GetParameters() []hal.ConfigParameter { return f.parameters }

func (f *orbFactory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
	failures := make(map[string][]string)

	// Address (required)
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

	// ADCMax (required)
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

	// Vref (required)
	if v, ok := parameters[vrefParam]; ok {
		val, ok := hal.ConvertToFloat(v)
		if !ok {
			failures[vrefParam] = append(failures[vrefParam],
				fmt.Sprintf("%s is not a number. %v was received.", vrefParam, v))
		} else if val <= 0 {
			failures[vrefParam] = append(failures[vrefParam],
				fmt.Sprintf("%s must be > 0. %v was received.", vrefParam, v))
		}
	} else {
		failures[vrefParam] = append(failures[vrefParam],
			fmt.Sprintf("%s is a required parameter, but was not received.", vrefParam))
	}

	// ORPRefVoltage (required)
	if v, ok := parameters[orpRefVoltageParam]; ok {
		val, ok := hal.ConvertToFloat(v)
		if !ok {
			failures[orpRefVoltageParam] = append(failures[orpRefVoltageParam],
				fmt.Sprintf("%s is not a number. %v was received.", orpRefVoltageParam, v))
		} else if val <= 0 {
			failures[orpRefVoltageParam] = append(failures[orpRefVoltageParam],
				fmt.Sprintf("%s must be > 0. %v was received.", orpRefVoltageParam, v))
		}
	} else {
		failures[orpRefVoltageParam] = append(failures[orpRefVoltageParam],
			fmt.Sprintf("%s is a required parameter, but was not received.", orpRefVoltageParam))
	}

	// ORPRefmV (required)
	if v, ok := parameters[orpRefmVParam]; ok {
		_, ok := hal.ConvertToFloat(v)
		if !ok {
			failures[orpRefmVParam] = append(failures[orpRefmVParam],
				fmt.Sprintf("%s is not a number. %v was received.", orpRefmVParam, v))
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

	intAddress, _ := hal.ConvertToInt(parameters[addressParam])
	adcMax, _ := hal.ConvertToInt(parameters[adcMaxParam])
	vref, _ := hal.ConvertToFloat(parameters[vrefParam])
	orpRefVoltage, _ := hal.ConvertToFloat(parameters[orpRefVoltageParam])
	orpRefmV, _ := hal.ConvertToFloat(parameters[orpRefmVParam])

	bus := hardwareResources.(i2c.Bus)

	return &ORB{
		addr:          byte(intAddress),
		bus:           bus,
		meta:          f.meta,
		adcMax:        adcMax,
		vref:          vref,
		orpRefVoltage: orpRefVoltage,
		orpRefmV:      orpRefmV,
	}, nil
}
