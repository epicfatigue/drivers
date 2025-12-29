package orb

import (
	"errors"
	"fmt"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	addressParam       = "Address"
	adcMaxParam        = "ADCMax"
	vrefParam          = "Vref"
	orpRefVoltageParam = "ORPRefVoltage"
	orpRefmVParam      = "ORPRefmV"
	delayParamMs       = "DelayMs"
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
				Description:  "ORB I2C ADC -> ORP (mV) driver (14-bit ADC, offset-calibrated)",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			parameters: []hal.ConfigParameter{
				{Name: addressParam, Type: hal.Integer, Order: 0, Default: 0x24},
				{Name: adcMaxParam, Type: hal.Integer, Order: 1, Default: 16383},
				{Name: vrefParam, Type: hal.Decimal, Order: 2, Default: 3.3},
				{Name: orpRefVoltageParam, Type: hal.Decimal, Order: 3, Default: 1.8491},
				{Name: orpRefmVParam, Type: hal.Decimal, Order: 4, Default: 256.0},
				{Name: delayParamMs, Type: hal.Integer, Order: 5, Default: 0},
			},
		}
	})
	return factory
}

func (f *orbFactory) Metadata() hal.Metadata { return f.meta }
func (f *orbFactory) GetParameters() []hal.ConfigParameter {
	return f.parameters
}

func (f *orbFactory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
	failures := make(map[string][]string)

	req := func(name string) {
		if _, ok := parameters[name]; !ok {
			failures[name] = append(failures[name], fmt.Sprintf("%s is required but was not received", name))
		}
	}

	req(addressParam)
	req(adcMaxParam)
	req(vrefParam)
	req(orpRefVoltageParam)
	req(orpRefmVParam)

	// Address
	if v, ok := parameters[addressParam]; ok {
		val, ok := hal.ConvertToInt(v)
		if !ok {
			failures[addressParam] = append(failures[addressParam], fmt.Sprintf("%s is not a number. %v was received", addressParam, v))
		} else if val <= 0 || val >= 256 {
			failures[addressParam] = append(failures[addressParam], fmt.Sprintf("%s out of range (1-255). %v was received", addressParam, v))
		}
	}

	// ADCMax
	if v, ok := parameters[adcMaxParam]; ok {
		val, ok := hal.ConvertToInt(v)
		if !ok || val <= 0 {
			failures[adcMaxParam] = append(failures[adcMaxParam], fmt.Sprintf("%s must be > 0. %v was received", adcMaxParam, v))
		}
	}

	// Vref
	if v, ok := parameters[vrefParam]; ok {
		val, ok := hal.ConvertToFloat(v)
		if !ok || val <= 0 {
			failures[vrefParam] = append(failures[vrefParam], fmt.Sprintf("%s must be > 0. %v was received", vrefParam, v))
		}
	}

	// ORPRefVoltage
	if v, ok := parameters[orpRefVoltageParam]; ok {
		val, ok := hal.ConvertToFloat(v)
		if !ok || val < 0 {
			failures[orpRefVoltageParam] = append(failures[orpRefVoltageParam], fmt.Sprintf("%s must be >= 0. %v was received", orpRefVoltageParam, v))
		}
	}

	// ORPRefmV
	if v, ok := parameters[orpRefmVParam]; ok {
		_, ok := hal.ConvertToFloat(v)
		if !ok {
			failures[orpRefmVParam] = append(failures[orpRefmVParam], fmt.Sprintf("%s is not a number. %v was received", orpRefmVParam, v))
		}
	}

	// DelayMs optional
	if v, ok := parameters[delayParamMs]; ok {
		val, ok := hal.ConvertToInt(v)
		if !ok || val < 0 {
			failures[delayParamMs] = append(failures[delayParamMs], fmt.Sprintf("%s must be >= 0. %v was received", delayParamMs, v))
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

	adcMax, _ := hal.ConvertToInt(parameters[adcMaxParam])
	vref, _ := hal.ConvertToFloat(parameters[vrefParam])
	orpRefVoltage, _ := hal.ConvertToFloat(parameters[orpRefVoltageParam])
	orpRefmV, _ := hal.ConvertToFloat(parameters[orpRefmVParam])

	delayMs := 0
	if v, ok := parameters[delayParamMs]; ok {
		delayMs, _ = hal.ConvertToInt(v)
	}

	bus := hardwareResources.(i2c.Bus)

	return &Driver{
		addr:          address,
		bus:           bus,
		delay:         time.Duration(delayMs) * time.Millisecond,
		adcMax:        adcMax,
		vref:          vref,
		orpRefVoltage: orpRefVoltage,
		orpRefmV:      orpRefmV,
		meta:          f.meta,
	}, nil
}
