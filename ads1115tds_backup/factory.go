package ads1115tds

import (
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	paramDebug   = "Debug"
	paramAddress = "Address" // i2c address (decimal int or hex string like "0x48")
	paramChannel = "Channel" // 0..3 for A0..A3
	paramGain    = "Gain"    // "2/3","1","2","4","8","16"
	paramTdsK    = "TdsK"    // slope
	paramTdsOff  = "TdsOffset"
	paramDelayMs = "DelayMs" // optional, default 0 (we can keep conversion polling only)
)

type factory struct {
	meta       hal.Metadata
	parameters []hal.ConfigParameter
}

var f *factory
var once sync.Once

func Factory() hal.DriverFactory {
	once.Do(func() {
		f = &factory{
			meta: hal.Metadata{
				Name:         driverName,
				Description:  "ADS1115 analog TDS driver (single-ended AINx->volts->linear TDS)",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			parameters: []hal.ConfigParameter{
				{Name: paramDebug, Type: hal.Boolean, Order: 0, Default: false},
				{Name: paramAddress, Type: hal.String, Order: 1, Default: "0x48"},
				{Name: paramChannel, Type: hal.Integer, Order: 2, Default: 0},
				{Name: paramGain, Type: hal.String, Order: 3, Default: "1"}, // +/-4.096V
				{Name: paramTdsK, Type: hal.Decimal, Order: 4, Default: 1.0},
				{Name: paramTdsOff, Type: hal.Decimal, Order: 5, Default: 0.0},
				{Name: paramDelayMs, Type: hal.Integer, Order: 6, Default: 0},
			},
		}
	})
	return f
}

func (f *factory) Metadata() hal.Metadata               { return f.meta }
func (f *factory) GetParameters() []hal.ConfigParameter { return f.parameters }

func (f *factory) ValidateParameters(p map[string]interface{}) (bool, map[string][]string) {
	fail := map[string][]string{}

	// address
	if v, ok := p[paramAddress]; ok {
		if _, err := parseI2CAddress(v); err != nil {
			fail[paramAddress] = append(fail[paramAddress], err.Error())
		}
	} else {
		fail[paramAddress] = append(fail[paramAddress], "Address is required")
	}

	// channel
	if v, ok := p[paramChannel]; ok {
		i, ok2 := hal.ConvertToInt(v)
		if !ok2 || i < 0 || i > 3 {
			fail[paramChannel] = append(fail[paramChannel], "must be 0..3 (AIN0..AIN3)")
		}
	} else {
		fail[paramChannel] = append(fail[paramChannel], "Channel is required")
	}

	// gain
	if v, ok := p[paramGain]; ok {
		if _, err := parseGain(v); err != nil {
			fail[paramGain] = append(fail[paramGain], err.Error())
		}
	} else {
		fail[paramGain] = append(fail[paramGain], "Gain is required")
	}

	// decimals
	for _, k := range []string{paramTdsK, paramTdsOff} {
		if v, ok := p[k]; ok {
			if _, err := convertToFloat(v); err != nil {
				fail[k] = append(fail[k], "must be a decimal number")
			}
		} else {
			fail[k] = append(fail[k], "is required")
		}
	}

	// delay
	if v, ok := p[paramDelayMs]; ok {
		i, ok2 := hal.ConvertToInt(v)
		if !ok2 || i < 0 || i > 5000 {
			fail[paramDelayMs] = append(fail[paramDelayMs], "must be 0..5000 ms")
		}
	}

	return len(fail) == 0, fail
}

// hardwareResources is expected to be an i2c.Bus injected by reef-pi.
func (f *factory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	ok, failures := f.ValidateParameters(parameters)
	if !ok {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	// helpful when your fork passes weird key casing
	if b, err := json.MarshalIndent(parameters, "", "  "); err == nil {
		log.Printf("ads1115tds NewDriver raw parameters:\n%s", string(b))
	}

	bus, ok := hardwareResources.(i2c.Bus)
	if !ok {
		return nil, fmt.Errorf("ads1115tds: expected i2c.Bus as hardware resource, got %T", hardwareResources)
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
		return nil, fmt.Errorf("ads1115tds: invalid channel %d (must be 0..3)", ch)
	}

	gain, err := parseGain(parameters[paramGain])
	if err != nil {
		return nil, err
	}

	tdsK, err := convertToFloat(parameters[paramTdsK])
	if err != nil {
		return nil, fmt.Errorf("%s must be a number", paramTdsK)
	}
	tdsOff, err := convertToFloat(parameters[paramTdsOff])
	if err != nil {
		return nil, fmt.Errorf("%s must be a number", paramTdsOff)
	}

	debug := getBool(parameters, paramDebug, false)
	delayMs := getInt(parameters, paramDelayMs, 0)

	pin := newTdsChannel(bus, addr, ch, mux, gain, tdsK, tdsOff, time.Duration(delayMs)*time.Millisecond, debug, f.meta)

	log.Printf("ads1115tds init addr=0x%02X ch=%d gain=0x%04X k=%.6f off=%.6f delay=%v debug=%v",
		addr, ch, gain, tdsK, tdsOff, time.Duration(delayMs)*time.Millisecond, debug)

	return &Driver{
		meta:     f.meta,
		channels: []hal.AnalogInputPin{pin},
	}, nil
}

func parseI2CAddress(v interface{}) (byte, error) {
	switch t := v.(type) {
	case string:
		s := strings.TrimSpace(t)
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
		switch strings.TrimSpace(s) {
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
	case json.Number:
		return t.Float64()
	case string:
		return strconv.ParseFloat(strings.TrimSpace(t), 64)
	default:
		if i, ok := hal.ConvertToInt(v); ok {
			return float64(i), nil
		}
		return 0, fmt.Errorf("not a number")
	}
}

func getInt(m map[string]interface{}, key string, def int) int {
	v, ok := m[key]
	if !ok {
		return def
	}
	if i, ok := hal.ConvertToInt(v); ok {
		return i
	}
	return def
}

func getBool(m map[string]interface{}, key string, def bool) bool {
	v, ok := m[key]
	if !ok {
		return def
	}
	switch t := v.(type) {
	case bool:
		return t
	case string:
		s := strings.ToLower(strings.TrimSpace(t))
		return (s == "1" || s == "true" || s == "yes" || s == "y" || s == "on")
	case int:
		return t != 0
	case int64:
		return t != 0
	case float64:
		return t != 0
	default:
		return def
	}
}
