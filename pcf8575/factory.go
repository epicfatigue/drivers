// factory.go
//
// PCF8575 driver factory for reef-pi.
//
// This file integrates the PCF8575 (16-bit I2C GPIO expander) into reef-pi's HAL:
//
//   - Declares driver metadata (name/description/capabilities)
//   - Exposes UI configuration parameters
//   - Validates configuration
//   - Constructs a driver instance and initializes the chip to a safe state
//
// Notes about PCF8575 behavior (important for correct expectations):
//   - The PCF8575 has *no registers*. A 16-bit write sets the output latch for all pins.
//   - Writing a '1' to a pin bit "releases" the pin (quasi-input / pulled high by weak source).
//   - Writing a '0' drives the pin LOW strongly.
//   - Reading returns the current pin level (which depends on latch + external circuitry).
//
// Driver design decisions here:
//   - We keep a "shadow" 16-bit latch of the last value written.
//   - All I2C transactions are serialized with a mutex so concurrent reads/writes
//     cannot interleave (reef-pi can call pins concurrently).
//   - A "safe default" of 0xFFFF is applied at startup (release all pins).
//
package pcf8575

import (
	"encoding/json"
	"fmt"
	"log"
	"strconv"
	"strings"
	"sync"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	paramAddress = "Address" // string, e.g. "0x20"
	paramDebug   = "Debug"   // bool
)

type factory struct {
	meta       hal.Metadata
	parameters []hal.ConfigParameter
}

var (
	f    *factory
	once sync.Once
)

func Factory() hal.DriverFactory {
	once.Do(func() {
		f = &factory{
			meta: hal.Metadata{
				Name:        "pcf8575",
				Description: "PCF8575 16-bit I2C GPIO expander (16 pins). Bit=1 releases (HIGH/input-ish), bit=0 drives LOW.",
				Capabilities: []hal.Capability{
					hal.DigitalInput,
					hal.DigitalOutput,
				},
			},
			parameters: []hal.ConfigParameter{
				{Name: paramAddress, Type: hal.String, Order: 0, Default: "0x20"},
				{Name: paramDebug, Type: hal.Boolean, Order: 1, Default: false},
			},
		}
	})
	return f
}

func (f *factory) Metadata() hal.Metadata               { return f.meta }
func (f *factory) GetParameters() []hal.ConfigParameter { return f.parameters }

// parseAddr accepts "0x20" style hex or "32" style decimal.
// Returns a 7-bit I2C address byte.
func parseAddr(s string) (byte, error) {
	s = strings.TrimSpace(strings.ToLower(s))
	if s == "" {
		return 0, fmt.Errorf("empty address")
	}
	if strings.HasPrefix(s, "0x") {
		v, err := strconv.ParseUint(s[2:], 16, 8)
		return byte(v), err
	}
	v, err := strconv.ParseUint(s, 10, 8)
	return byte(v), err
}

func (f *factory) ValidateParameters(params map[string]interface{}) (bool, map[string][]string) {
	errs := make(map[string][]string)

	addrStr, _ := params[paramAddress].(string)
	addrStr = strings.TrimSpace(addrStr)
	if addrStr == "" {
		errs[paramAddress] = append(errs[paramAddress], "is required (e.g. 0x20)")
	} else {
		addr, err := parseAddr(addrStr)
		if err != nil {
			errs[paramAddress] = append(errs[paramAddress], "must be a valid I2C address like 0x20..0x27")
		} else if addr > 127 {
			errs[paramAddress] = append(errs[paramAddress], "must be a 7-bit address (0..127)")
		}
	}

	if v, ok := params[paramDebug]; ok {
		if _, ok := v.(bool); !ok {
			errs[paramDebug] = append(errs[paramDebug], "must be boolean")
		}
	}

	if len(errs) > 0 {
		return false, errs
	}
	return true, nil
}

func (f *factory) NewDriver(params map[string]interface{}, bus interface{}) (hal.Driver, error) {
	// Defensive validation (reef-pi may call ValidateParameters separately; don't rely on it).
	if ok, failures := f.ValidateParameters(params); !ok {
		return nil, fmt.Errorf(hal.ToErrorString(failures))
	}

	i2cBus, ok := bus.(i2c.Bus)
	if !ok {
		return nil, fmt.Errorf("pcf8575: expected i2c.Bus, got %T", bus)
	}

	addrStr, _ := params[paramAddress].(string)
	addr, err := parseAddr(addrStr)
	if err != nil {
		return nil, fmt.Errorf("pcf8575: invalid Address %q: %w", addrStr, err)
	}

	debug := false
	if v, ok := params[paramDebug]; ok {
		b, ok := v.(bool)
		if !ok {
			return nil, fmt.Errorf("pcf8575: %s must be boolean", paramDebug)
		}
		debug = b
	}

	// Optional: log config when debug is enabled (keeps journal clean by default).
	if debug {
		if b, err := json.MarshalIndent(params, "", "  "); err == nil {
			log.Printf("pcf8575 NewDriver params:\n%s", string(b))
		}
	}

	hw := New(addr, i2cBus)

	d := &pcf8575Driver{
		hwDriver: hw,
		addr:     addr,
		shadow:   0xFFFF, // safe default: release all pins (HIGH/input-ish)
		invert:   false,  // (kept for future; currently not user-configurable)
		debug:    debug,
		meta:     f.meta,
	}

	// Initialize hardware to safe state (all released/high).
	// This prevents accidental LOW outputs on boot.
	if err := d.hwDriver.Write16(d.shadow); err != nil {
		return nil, fmt.Errorf("pcf8575 addr=0x%02X init write shadow=0x%04X failed: %w", d.addr, d.shadow, err)
	}

	// Create 16 pins (0..15).
	for i := 0; i < 16; i++ {
		d.pins = append(d.pins, &pcf8575Pin{driver: d, pin: i})
	}

	if d.debug {
		log.Printf("pcf8575 init addr=0x%02X shadow=0x%04X (all released/high)", d.addr, d.shadow)
	}

	return d, nil
}
