package ads1115tds

import (
	"fmt"
	"strconv"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	paramAddress = "Address"  // i2c address (decimal int or hex string like "0x48")
	paramChannel = "Channel"  // 0..3 for A0..A3
	paramGain    = "Gain"     // "2/3","1","2","4","8","16"
	paramDelayMs = "DelayMs"  // conversion wait ms
	paramVref    = "Vref"     // optional clamp
	paramTdsK    = "TdsK"     // slope
	paramTdsOff  = "TdsOffset"
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
		{Name: paramAddress, Type: hal.String, Order: 0, Default: "0x48"},
		{Name: paramChannel, Type: hal.Integer, Order: 1, Default: 0},
		{Name: paramGain, Type: hal.String, Order: 2, Default: "1"}, // +/-4.096
		{Name: paramDelayMs, Type: hal.Integer, Order: 3, Default: 10},
		{Name: paramVref, Type: hal.Decimal, Order: 4, Default: 3.3},
		{Name: paramTdsK, Type: hal.Decimal, Order: 5, Default: 1.0},
		{Name: paramTdsOff, Type: hal.Decimal, Order: 6, Default: 0.0},
	}
}

func (f *factory) ValidateParameters(p map[string]interface{}) (bool, map[string][]string) {
	fail := map[string][]string{}

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

// hardwareResources is expected to be an i2c.Bus injected by reef-pi.
func (f *factory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	ok, failures := f.ValidateParameters(parameters)
	if !ok {
		return nil, fmt.Errorf("invalid parameters: %s", hal.ToErrorString(failures))
	}

	bus, ok := hardwareResources.(i2c.Bus)
	if !ok {
		return nil, fmt.Errorf("ads1115-tds: expected i2c.Bus as hardware resource, got %T", hardwareResources)
	}

	addr, err := parseI2CAddress(parameters[paramAddress])
	if err != nil {
		return nil, err
	}

	ch := 0
	if v, ok := parameters[paramChannel]; ok {
		if i, ok2 := hal.ConvertToInt(v); ok2 {
			ch = i
		}
	}
	mux, okMux := muxForChannel(ch)
	if !okMux {
		return nil, fmt.Errorf("ads1115-tds: invalid channel %d (must be 0..3)", ch)
	}

	gain, err := parseGain(parameters[paramGain])
	if err != nil {
		return nil, err
	}

	delayMs := 10
	if v, ok := parameters[paramDelayMs]; ok {
		if i, ok2 := hal.ConvertToInt(v); ok2 {
			delayMs = i
		}
	}
	delay := time.Duration(delayMs) * time.Millisecond

	vref, _ := convertToFloat(parameters[paramVref])
	tdsK, _ := convertToFloat(parameters[paramTdsK])
	tdsOff, _ := convertToFloat(parameters[paramTdsOff])

	pin := newTdsChannel(bus, addr, ch, mux, gain, delay, vref, tdsK, tdsOff)

	return &Driver{
		meta:     f.Metadata(),
		channels: []hal.AnalogInputPin{pin},
	}, nil
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
	// Friendly strings
	if s, ok := v.(string); ok {
		switch s {
		case "2/3":
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
			// allow "0..5" as string too
			if n, err := strconv.Atoi(s); err == nil {
				return parseGain(n)
			}
			return 0, fmt.Errorf("Gain must be one of: 2/3,1,2,4,8,16")
		}
	}

	// Advanced: int mapping 0..5
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
	case string:
		return strconv.ParseFloat(t, 64)
	default:
		if i, ok := hal.ConvertToInt(v); ok {
			return float64(i), nil
		}
		return 0, fmt.Errorf("not a number")
	}
}
