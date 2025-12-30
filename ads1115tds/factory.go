package ads1115tds

import (
	"errors"
	"fmt"
	"strconv"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	addressParam   = "Address"
	vrefParam      = "Vref"
	tdsKParam      = "TdsK"
	tdsOffsetParam = "TdsOffset"

	// Keep these simple; reef-pi stores config as strings nicely
	gain0Param = "Gain A0"
	gain1Param = "Gain A1"
	gain2Param = "Gain A2"
	gain3Param = "Gain A3"
)

var gainParams = []string{gain0Param, gain1Param, gain2Param, gain3Param}

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

var (
	factoryInstance *ads1115TdsFactory
	factoryOnce     sync.Once
)

// Ads1115TdsFactory returns the driver factory (reef-pi expects factories for UI + driver creation).
func Ads1115TdsFactory() hal.DriverFactory {
	factoryOnce.Do(func() {
		factoryInstance = &ads1115TdsFactory{
			meta: hal.Metadata{
				Name:         "ADS1115-TDS",
				Description:  "ADS1115 analog input (A0-A3) with basic TDS conversion on Measure()",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
		}
		factoryInstance.appendParameters()
	})
	return factoryInstance
}

func (f *ads1115TdsFactory) Metadata() hal.Metadata { return f.meta }
func (f *ads1115TdsFactory) GetParameters() []hal.ConfigParameter {
	return f.parameters
}

func (f *ads1115TdsFactory) appendParameters() {
	f.parameters = []hal.ConfigParameter{
		{Name: addressParam, Type: hal.Integer, Order: 0, Default: 0x48},
		{Name: vrefParam, Type: hal.Float, Order: 1, Default: 3.3},
		{Name: tdsKParam, Type: hal.Float, Order: 2, Default: 1.0},
		{Name: tdsOffsetParam, Type: hal.Float, Order: 3, Default: 0.0},
		{Name: gain0Param, Type: hal.String, Order: 4, Default: "1"},
		{Name: gain1Param, Type: hal.String, Order: 5, Default: "1"},
		{Name: gain2Param, Type: hal.String, Order: 6, Default: "1"},
		{Name: gain3Param, Type: hal.String, Order: 7, Default: "1"},
	}
}

func (f *ads1115TdsFactory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
	failures := make(map[string][]string)

	// Address
	if v, ok := parameters[addressParam]; !ok {
		failures[addressParam] = append(failures[addressParam], "Address is required.")
	} else if val, ok := hal.ConvertToInt(v); !ok {
		failures[addressParam] = append(failures[addressParam], fmt.Sprintf("Address must be an integer, got %v.", v))
	} else {
		addr := val.(int)
		if addr < 1 || addr > 0x7F { // i2c 7-bit typical range; reef-pi used 1..255 earlier, but 7-bit is safer
			failures[addressParam] = append(failures[addressParam], fmt.Sprintf("Address out of range (1-127), got %d.", addr))
		}
	}

	// Vref
	if v, ok := parameters[vrefParam]; !ok {
		failures[vrefParam] = append(failures[vrefParam], "Vref is required.")
	} else if val, ok := hal.ConvertToFloat(v); !ok {
		failures[vrefParam] = append(failures[vrefParam], fmt.Sprintf("Vref must be a number, got %v.", v))
	} else if vf := val.(float64); vf <= 0 || vf > 6.144 {
		failures[vrefParam] = append(failures[vrefParam], fmt.Sprintf("Vref out of range (0-6.144), got %v.", vf))
	}

	// TdsK + Offset
	if v, ok := parameters[tdsKParam]; !ok {
		failures[tdsKParam] = append(failures[tdsKParam], "TdsK is required.")
	} else if _, ok := hal.ConvertToFloat(v); !ok {
		failures[tdsKParam] = append(failures[tdsKParam], fmt.Sprintf("TdsK must be a number, got %v.", v))
	}
	if v, ok := parameters[tdsOffsetParam]; !ok {
		failures[tdsOffsetParam] = append(failures[tdsOffsetParam], "TdsOffset is required.")
	} else if _, ok := hal.ConvertToFloat(v); !ok {
		failures[tdsOffsetParam] = append(failures[tdsOffsetParam], fmt.Sprintf("TdsOffset must be a number, got %v.", v))
	}

	// Gains
	for _, p := range gainParams {
		v, ok := parameters[p]
		if !ok {
			failures[p] = append(failures[p], "Gain is required.")
			continue
		}
		if _, err := parseGain(v); err != nil {
			failures[p] = append(failures[p], err.Error())
		}
	}

	return len(failures) == 0, failures
}

func parseGain(v interface{}) (string, error) {
	// reef-pi sometimes passes numbers; accept that too
	if s, ok := v.(string); ok {
		if _, ok := gainOptions[s]; ok {
			return s, nil
		}
		return "", fmt.Errorf("gain must be one of 2/3, 1, 2, 4, 8, 16 (got %v)", v)
	}

	if vi, ok := hal.ConvertToInt(v); ok {
		s := strconv.Itoa(vi.(int))
		if _, ok := gainOptions[s]; ok {
			return s, nil
		}
		return "", fmt.Errorf("gain must be one of 2/3, 1, 2, 4, 8, 16 (got %v)", v)
	}

	return "", fmt.Errorf("gain must be a string/number (got %T)", v)
}

func (f *ads1115TdsFactory) New(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	if valid, failures := f.ValidateParameters(parameters); !valid {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	bus := hardwareResources.(i2c.Bus)

	ai, _ := hal.ConvertToInt(parameters[addressParam])
	address := byte(ai.(int))

	vf, _ := hal.ConvertToFloat(parameters[vrefParam])
	kf, _ := hal.ConvertToFloat(parameters[tdsKParam])
	of, _ := hal.ConvertToFloat(parameters[tdsOffsetParam])

	vref := vf.(float64)
	tdsK := kf.(float64)
	tdsOffset := of.(float64)

	// simple + reliable defaults:
	// 860 SPS => ~1.16ms conversion time; 5-10ms delay is plenty even with bus jitter
	dataRate := configDataRate860
	delay := 8 * time.Millisecond

	// shared lock so multiple channels don’t fight on the same I2C bus concurrently
	var busMu sync.Mutex

	d := &driver{
		meta:     f.meta,
		channels: make([]hal.AnalogInputPin, 0, 4),
	}

	for ch := 0; ch < 4; ch++ {
		gainStr, _ := parseGain(parameters[gainParams[ch]])
		gain := gainOptions[gainStr]

		// Try a lightweight read of config register once (per driver creation)
		// so a bad address fails fast. Do it once, before making channels.
		if ch == 0 {
			var tmp [2]byte
			if err := bus.ReadFromReg(address, regConfig, tmp[:]); err != nil {
				return nil, err
			}
		}

		pin, err := newTdsChannel(
			bus,
			&busMu,
			address,
			ch,
			channelMux[ch],
			gain,
			dataRate,
			delay,
			vref,
			tdsK,
			tdsOffset,
		)
		if err != nil {
			return nil, err
		}
		d.channels = append(d.channels, pin)
	}

	return d, nil
}
