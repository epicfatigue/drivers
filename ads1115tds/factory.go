package ads1115tds

import (
	"errors"
	"fmt"
	"strconv"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	addressParam   = "Address"
	delayMsParam   = "DelayMs"
	vrefParam      = "Vref"
	tdsKParam      = "TdsK"
	tdsOffsetParam = "TdsOffset"
)

var channelMux = [4]uint16{configMuxSingle0, configMuxSingle1, configMuxSingle2, configMuxSingle3}
var channelGains = [4]string{"Gain 1", "Gain 2", "Gain 3", "Gain 4"}

var gainOptions = map[string]uint16{
	"2/3": configGainTwoThirds,
	"1":   configGainOne,
	"2":   configGainTwo,
	"4":   configGainFour,
	"8":   configGainEight,
	"16":  configGainSixteen,
}

type ads1115TdsFactory struct {
	meta       hal.Metadata
	parameters []hal.ConfigParameter
}

func Factory() hal.DriverFactory {
	f := &ads1115TdsFactory{
		meta: hal.Metadata{
			Name:        "ads1115-tds",
			Description: "ADS1115 analog input driver for TDS/Salinity boards (linear K + offset conversion)",
			Capabilities: []hal.Capability{
				hal.AnalogInput,
			},
		},
	}

	f.parameters = []hal.ConfigParameter{
		{Name: addressParam, Type: hal.Number, Order: 0, Default: 0x48},
		{Name: delayMsParam, Type: hal.Number, Order: 1, Default: 2}, // safe at 860 SPS
		{Name: vrefParam, Type: hal.Float, Order: 2, Default: 3.3},
		{Name: tdsKParam, Type: hal.Float, Order: 3, Default: 1.0},
		{Name: tdsOffsetParam, Type: hal.Float, Order: 4, Default: 0.0},
	}

	// Per-channel gain (so you can use any AINx)
	for i, name := range channelGains {
		f.parameters = append(f.parameters, hal.ConfigParameter{
			Name:    name,
			Type:    hal.String,
			Order:   5 + i,
			Default: "2/3",
		})
	}

	return f
}

func (f *ads1115TdsFactory) Metadata() hal.Metadata {
	return f.meta
}

func (f *ads1115TdsFactory) GetParameters() []hal.ConfigParameter {
	return f.parameters
}

func (f *ads1115TdsFactory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
	failures := make(map[string][]string)

	// Address
	v, ok := parameters[addressParam]
	if !ok {
		failures[addressParam] = append(failures[addressParam], addressParam+" is required.")
	} else {
		val, ok := hal.ConvertToInt(v)
		if !ok {
			failures[addressParam] = append(failures[addressParam], fmt.Sprintf("%s must be a number, got: %v", addressParam, v))
		} else if val.(int) <= 0 || val.(int) >= 128 {
			failures[addressParam] = append(failures[addressParam], fmt.Sprintf("%s out of range (1-127), got: %v", addressParam, v))
		}
	}

	// DelayMs
	if v, ok := parameters[delayMsParam]; ok {
		val, ok := hal.ConvertToInt(v)
		if !ok || val.(int) < 0 || val.(int) > 500 {
			failures[delayMsParam] = append(failures[delayMsParam], fmt.Sprintf("%s must be 0..500, got: %v", delayMsParam, v))
		}
	}

	// Vref, TdsK, TdsOffset (just validate they parse as float)
	for _, p := range []string{vrefParam, tdsKParam, tdsOffsetParam} {
		if v, ok := parameters[p]; ok {
			if _, ok := hal.ConvertToFloat(v); !ok {
				failures[p] = append(failures[p], fmt.Sprintf("%s must be a float, got: %v", p, v))
			}
		}
	}

	// Gains
	for _, gname := range channelGains {
		v, ok := parameters[gname]
		if !ok {
			failures[gname] = append(failures[gname], gname+" is required.")
			continue
		}
		gs, err := parseGain(v)
		if err != nil {
			failures[gname] = append(failures[gname], gname+err.Error())
			continue
		}
		if _, ok := gainOptions[gs]; !ok {
			failures[gname] = append(failures[gname], fmt.Sprintf("%s invalid (use 2/3,1,2,4,8,16), got: %v", gname, v))
		}
	}

	return len(failures) == 0, failures
}

func parseGain(v interface{}) (string, error) {
	// UI often passes strings, but sometimes numbers
	if s, ok := v.(string); ok {
		return s, nil
	}
	if iv, ok := hal.ConvertToInt(v); ok {
		return strconv.Itoa(iv.(int)), nil
	}
	return "", fmt.Errorf(" must be a string or number, got: %T", v)
}

func (f *ads1115TdsFactory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	if valid, failures := f.ValidateParameters(parameters); !valid {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	bus := hardwareResources.(i2c.Bus)

	intAddress, _ := hal.ConvertToInt(parameters[addressParam])
	address := byte(intAddress.(int))

	// Confirm device responds
	var tmp [2]byte
	if err := bus.ReadFromReg(address, regConfig, tmp[:]); err != nil {
		return nil, err
	}

	// Delay
	delayMs := 2
	if v, ok := parameters[delayMsParam]; ok {
		if iv, ok := hal.ConvertToInt(v); ok {
			delayMs = iv.(int)
		}
	}
	delay := time.Duration(delayMs) * time.Millisecond

	// Knobs
	vf, _ := hal.ConvertToFloat(parameters[vrefParam])
	kf, _ := hal.ConvertToFloat(parameters[tdsKParam])
	of, _ := hal.ConvertToFloat(parameters[tdsOffsetParam])

	vref := vf.(float64)
	tdsK := kf.(float64)
	tdsOffset := of.(float64)

	d := driver{
		meta:     f.meta,
		channels: []hal.AnalogInputPin{},
	}

	for i, mux := range channelMux {
		gainStr, _ := parseGain(parameters[channelGains[i]])
		gainCfg := gainOptions[gainStr]

		ch, err := newTdsChannel(
			bus,
			address,
			i,
			mux,
			gainCfg,
			delay,
			vref,
			tdsK,
			tdsOffset,
		)
		if err != nil {
			return nil, err
		}
		d.channels = append(d.channels, ch)
	}

	return &d, nil
}
