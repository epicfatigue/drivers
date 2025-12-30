package ads1115tds

import (
	"fmt"
	"strconv"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	paramBus      = "Bus"      // i2c bus number (usually 1)
	paramAddress  = "Address"  // i2c address (decimal, e.g. 72) or hex string (e.g. "0x48")
	paramChannel  = "Channel"  // 0..3 for A0..A3
	paramGain     = "Gain"     // "2/3","1","2","4","8","16" OR numeric 0..5 (advanced)
	paramDelayMs  = "DelayMs"  // conversion wait in ms
	paramVref     = "Vref"     // optional clamp
	paramTdsK     = "TdsK"     // linear slope
	paramTdsOff   = "TdsOffset"
)

type factory struct{}

func Factory() hal.DriverFactory { return &factory{} }

func (f *factory) Metadata() hal.Metadata {
	return hal.Metadata{
		Name:        "ads1115-tds",
		Description: "ADS1115 analog TDS driver (single channel, linear volts->tds)",
		Capabilities: []hal.Capability{
			hal.AnalogInput,
		},
	}
}

func (f *factory) GetParameters() []hal.ConfigParameter {
	return []hal.ConfigParameter{
		{Name: paramBus, Type: hal.Integer, Order: 0, Default: 1},
		{Name: paramAddress, Type: hal.String, Order: 1, Default: "0x48"},
		{Name: paramChannel, Type: hal.Integer, Order: 2, Default: 0},
		{Name: paramGain, Type: hal.String, Order: 3, Default: "1"}, // +/-4.096
		{Name: paramDelayMs, Type: hal.Integer, Order: 4, Default: 10},
		{Name: paramVref, Type: hal.Decimal, Order: 5, Default: 3.3},
		{Name: paramTdsK, Type: hal.Decimal, Order: 6, Default: 1.0},
		{Name: paramTdsOff, Type: hal.Decimal, Order: 7, Default: 0.0},
	}
}

func (f *factory) ValidateParameters(p map[string]interface{}) (bool, map[string][]string) {
	fail := map[string][]string{}

	// bus
	if v, ok := p[paramBus]; ok {
		if i, ok2 := hal.ConvertToInt(v); !ok2 || i <= 0 {
			fail[paramBus] = append(fail[paramBus], "must be a positive integer")
		}
	}

	// address
	if v, ok := p[paramAddress]; ok {
		if _, err := parseI2CAddress(v); err != nil {
			fail[paramAddress] = append(fail[paramAddress], err.Error())
		}
	}

	// channel
	if v, ok := p[paramChannel]; ok {
		i, ok2 := hal.ConvertToInt(v)
		if !ok2 || i < 0 || i > 3 {
			fail[paramChannel] = append(fail[paramChannel], "must be 0..3 (A0..A3)")
		}
	}

	// gain
	if v, ok := p[paramGain]; ok {
		if _, err := parseGain(v); err != nil {
			fail[paramGain] = append(fail[paramGain], err.Error())
		}
	}

	// delay
	if v, ok := p[paramDelayMs]; ok {
		i, ok2 := hal.ConvertToInt(v)
		if !ok2 || i < 0 {
			fail[paramDelayMs] = append(fail[paramDelayMs], "must be >= 0")
		}
	}

	// decimals
	for _, k := range []string{paramVref, paramTdsK, paramTdsOff} {
		if v, ok := p[k]; ok {
			if _, err := convertToFloat(v); err != nil {
				fail[k] = append(fail[k], "must be a decimal number")
			}
		}
	}

	return len(fail) == 0, fail
}

func (f *factory) NewDriver(parameters map[string]interface{}, _ interface{}) (hal.Driver, error) {
	ok, failures := f.ValidateParameters(parameters)
	if !ok {
		return nil, fmt.Errorf("invalid parameters: %s", hal.ToErrorString(failures))
	}

	// bus
	busNum := 1
	if v, ok := parameters[paramBus]; ok {
		if i, ok2 := hal.ConvertToInt(v); ok2 {
			busNum = i
		}
	}

	// address
	addr, err := parseI2CAddress(parameters[paramAddress])
	if err != nil {
		return nil, err
	}

	// channel -> mux
	ch := 0
	if v, ok := parameters[paramChannel]; ok {
		if i, ok2 := hal.ConvertToInt(v); ok2 {
			ch = i
		}
	}
	mux, _ := muxForChannel(ch)

	// gain
	gain, err := parseGain(parameters[paramGain])
	if err != nil {
		return nil, err
	}

	// delay
	delayMs := 10
	if v, ok := parameters[paramDelayMs]; ok {
		if i, ok2 := hal.ConvertToInt(v); ok2 {
			delayMs = i
		}
	}
	delay := time.Duration(delayMs) * time.Millisecond

	// floats
	vref, _ := convertToFloat(parameters[paramVref])
	tdsK, _ := convertToFloat(parameters[paramTdsK])
	tdsOff, _ := convertToFloat(parameters[paramTdsOff])

	// Open i2c bus
	bus, err := i2c.New(busNum)
	if err != nil {
		return nil, err
	}

	d := &Driver{
		bus:     bus,
		address: addr,
		channels: []*tdsChannel{
			newTdsChannel(bus, addr, ch, mux, gain, delay, vref, tdsK, tdsOff),
		},
	}
	return d, nil
}

func parseI2CAddress(v interface{}) (byte, error) {
	switch t := v.(type) {
	case string:
		s := t
		base := 10
		if len(s) > 2 && (s[0:2] == "0x" || s[0:2] == "0X") {
			base = 0
		}
		n64, err := strconv.ParseInt(s, base, 32)
		if err != nil {
			return 0, fmt.Errorf("Address must be int or hex string like 0x48: %v", err)
		}
		if n64 < 0 || n64 > 127 {
			return 0, fmt.Errorf("Address out of range (0..127): %d", n64)
		}
		return byte(n64), nil
	default:
		if i, ok := hal.ConvertToInt(v); ok {
			if i < 0 || i > 127 {
				return 0, fmt.Errorf("Address out of range (0..127): %d", i)
			}
			return byte(i), nil
		}
		return 0, fmt.Errorf("Address must be int or hex string like 0x48")
	}
}

func parseGain(v interface{}) (uint16, error) {
	// Accept a friendly string: "2/3","1","2","4","8","16"
	if s, ok := v.(string); ok {
		switch s {
		case "2/3", "0", "0.666", "0.67":
			return configGainTwoThirds, nil
		case "1":
			return configGainOne, nil
		case "2":
			return configGainTwo, nil
		case "4":
			return configGainFour, nil
		case "8":
			return configGainEight, nil
		case "16":
			return configGainSixteen, nil
		default:
			// allow "0..5" as string
			if n, err := strconv.Atoi(s); err == nil {
				return parseGain(n)
			}
			return 0, fmt.Errorf("Gain must be one of: 2/3,1,2,4,8,16")
		}
	}

	// Or accept an int (advanced): 0..5 mapping
	if n, ok := hal.ConvertToInt(v); ok {
		switch n {
		case 0:
			return configGainTwoThirds, nil
		case 1:
			return configGainOne, nil
		case 2:
			return configGainTwo, nil
		case 3:
			return configGainFour, nil
		case 4:
			return configGainEight, nil
		case 5:
			return configGainSixteen, nil
		default:
			return 0, fmt.Errorf("Gain int must be 0..5")
		}
	}

	return 0, fmt.Errorf("Gain must be string (2/3,1,2,4,8,16) or int (0..5)")
}

func convertToFloat(v interface{}) (float64, error) {
	switch t := v.(type) {
	case float64:
		return t, nil
	case float32:
		return float64(t), nil
	case int:
		return float64(t), nil
	case int64:
		return float64(t), nil
	case jsonNumber:
		return t.Float64()
	case string:
		f, err := strconv.ParseFloat(t, 64)
		if err != nil {
			return 0, err
		}
		return f, nil
	default:
		// some JSON libs may use int via interface{}, hal doesn't provide ConvertToFloat
		if i, ok := hal.ConvertToInt(v); ok {
			return float64(i), nil
		}
		return 0, fmt.Errorf("not a number")
	}
}

// tiny shim so we don't hard-depend on encoding/json here;
// if the value happens to be json.Number, it will satisfy this.
type jsonNumber interface {
	Float64() (float64, error)
}
