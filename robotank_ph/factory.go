// factory.go
//
// Robo-Tank pH driver factory.
//
// This file wires the driver into reef-pi's HAL driver system:
//   - Declares driver metadata (name/description/capabilities)
//   - Exposes configuration parameters in the UI
//   - Validates user configuration
//   - Constructs the Driver instance with safe defaults
//
// Important design decisions:
//   - Delay is FIXED (not user-configurable). The Robo-Tank firmware needs a
//     stable write->read delay; exposing it just creates misconfiguration risk.
//   - "Calibration" is software-only using observed anchors (Obs4/Obs7/Obs10).
//   - Temperature compensation is intentionally NOT supported (board returns pH,
//     not raw electrode mV).
//
package robotank_ph

import (
	"encoding/json"
	"errors"
	"log"
	"strconv"
	"strings"
	"sync"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

// factory implements hal.DriverFactory.
// reef-pi will call Factory() to obtain an instance.
type factory struct {
	meta       hal.Metadata
	parameters []hal.ConfigParameter
}

const (
	// UI parameter order/layout: Debug, Address, Obs4, Obs7, Obs10

	// Debug enables verbose logging to help diagnose I2C/protocol issues.
	debugParam = "Debug"

	// Address is the 7-bit I2C address of the Robo-Tank board.
	addressParam = "Address"

	// Obs4/Obs7/Obs10 are OBSERVED readings:
	// the pH value REPORTED BY THE BOARD while the probe is sitting in a known
	// buffer solution (4/7/10).
	//
	// These are NOT the true pH values. The true values are fixed constants
	// inside driver.go (truePH4/truePH7/truePH10).
	//
	// Use -1 to disable an anchor.
	obs4Param  = "Obs4"
	obs7Param  = "Obs7"
	obs10Param = "Obs10"
)

// Singleton factory instance (driver factories are typically singletons).
var f *factory
var once sync.Once

// Factory returns the singleton driver factory.
// reef-pi calls this during driver discovery/registration.
func Factory() hal.DriverFactory {
	once.Do(func() {
		f = &factory{
			meta: hal.Metadata{
				Name:         driverName,
				Description:  "Robo-Tank pH circuit (I2C ASCII protocol: H/R). Software trim via Obs4/Obs7/Obs10.",
				Capabilities: []hal.Capability{hal.AnalogInput},
			},
			parameters: []hal.ConfigParameter{
				// Basic runtime diagnostics
				{Name: debugParam, Type: hal.Boolean, Order: 0, Default: false},

				// I2C address (defaults to Robo-Tank typical address)
				{Name: addressParam, Type: hal.Integer, Order: 1, Default: 0x62},

				// -1 means disabled (no anchor)
				{Name: obs4Param, Type: hal.Decimal, Order: 2, Default: -1.0},
				{Name: obs7Param, Type: hal.Decimal, Order: 3, Default: -1.0},
				{Name: obs10Param, Type: hal.Decimal, Order: 4, Default: -1.0},
			},
		}
	})
	return f
}

// Metadata returns static driver metadata used by reef-pi UI and driver registry.
func (f *factory) Metadata() hal.Metadata { return f.meta }

// GetParameters returns the configuration schema shown in the reef-pi UI.
func (f *factory) GetParameters() []hal.ConfigParameter { return f.parameters }

// ValidateParameters checks that the user configuration is safe and meaningful.
//
// Rules enforced:
//   - Address is required and must be 0..127 (7-bit I2C)
//   - At least one anchor is enabled (Obs4/Obs7/Obs10 != -1)
//   - Enabled anchors must be in the plausible pH range 0..14
func (f *factory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
	failures := map[string][]string{}

	// --- Address validation ---
	addrV, ok := parameters[addressParam]
	if !ok {
		failures[addressParam] = []string{"Address is required"}
		return false, failures
	}
	addr, ok := toInt(addrV)
	if !ok || addr < 0 || addr > 127 {
		failures[addressParam] = []string{"Address must be an integer 0..127"}
	}

	// --- Anchor validation ---
	obs4 := getFloat(parameters, obs4Param, -1)
	obs7 := getFloat(parameters, obs7Param, -1)
	obs10 := getFloat(parameters, obs10Param, -1)

	enabled := 0
	for _, v := range []float64{obs4, obs7, obs10} {
		if v != -1 {
			enabled++
			if v < 0 || v > 14 {
				failures["Obs"] = append(
					failures["Obs"],
					"Obs values must be -1 or in range 0..14 (these are observed pH readings)",
				)
			}
		}
	}

	// Without at least one anchor, calibration is effectively undefined for this driver.
	if enabled == 0 {
		failures["Obs"] = append(
			failures["Obs"],
			"Set at least one of Obs4/Obs7/Obs10. Best practice: set Obs7 and one of Obs4/Obs10.",
		)
	}

	return len(failures) == 0, failures
}

// NewDriver constructs a driver instance after validating parameters.
//
// Notes:
//   - Delay is fixed at fixedReadDelay (defined in driver.go) to avoid
//     misconfiguration. The Robo-Tank firmware requires a stable write->read delay.
//   - Firmware() is queried only when Debug is enabled to avoid extra I2C traffic.
func (f *factory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
	// Defensive: validate again (reef-pi may call Validate separately, but don't rely on it).
	if valid, failures := f.ValidateParameters(parameters); !valid {
		return nil, errors.New(hal.ToErrorString(failures))
	}

	// Log config for debugging/support (useful when users paste logs)
	if b, err := json.MarshalIndent(parameters, "", "  "); err == nil {
		log.Printf("robotank_ph NewDriver parameters:\n%s", string(b))
	}

	// Parse parameters
	addr, _ := toInt(parameters[addressParam])
	debug := getBool(parameters, debugParam, false)

	obs4 := getFloat(parameters, obs4Param, -1)
	obs7 := getFloat(parameters, obs7Param, -1)
	obs10 := getFloat(parameters, obs10Param, -1)

	// Instantiate driver
	d := &Driver{
		addr:  byte(addr),
		bus:   hardwareResources.(i2c.Bus),
		debug: debug,

		// Fixed, known-safe delay for Robo-Tank firmware. See driver.go.
		delay: fixedReadDelay,

		// Software calibration anchors (observed readings)
		obs4:  obs4,
		obs7:  obs7,
		obs10: obs10,

		meta: f.meta,
	}
	d.pin = &phPin{d: d}

	log.Printf(
		"robotank_ph init addr=0x%02X delay=%v debug=%v obs(4=%.4f 7=%.4f 10=%.4f)",
		d.addr, d.delay, d.debug, d.obs4, d.obs7, d.obs10,
	)

	// Optional: query firmware/ident string (only in debug mode)
	if d.debug {
		if fw, err := d.Firmware(); err == nil {
			log.Printf("robotank_ph addr=0x%02X H=%q", d.addr, fw)
		} else {
			log.Printf("robotank_ph addr=0x%02X H error: %v", d.addr, err)
		}
	}

	return d, nil
}

// ----------------- helpers -----------------

// getInt reads an integer parameter from the config map.
// reef-pi may provide values as float64 (JSON) or string, so we normalize.
func getInt(m map[string]interface{}, key string, def int) int {
	v, ok := m[key]
	if !ok {
		return def
	}
	if i, ok := toInt(v); ok {
		return i
	}
	return def
}

// getBool reads a boolean parameter from the config map.
// reef-pi may provide values as bool, number, or string ("true"/"false"/"1"/"0").
func getBool(m map[string]interface{}, key string, def bool) bool {
	v, ok := m[key]
	if !ok {
		return def
	}
	if b, ok := toBool(v); ok {
		return b
	}
	return def
}

// getFloat reads a float parameter from the config map.
// reef-pi may provide values as float64, int, or string.
func getFloat(m map[string]interface{}, key string, def float64) float64 {
	v, ok := m[key]
	if !ok {
		return def
	}
	if f, ok := toFloat(v); ok {
		return f
	}
	return def
}

// toInt normalizes various types into an int.
// reef-pi often passes JSON numbers as float64.
func toInt(v interface{}) (int, bool) {
	switch t := v.(type) {
	case int:
		return t, true
	case int64:
		return int(t), true
	case float64:
		return int(t), true
	case string:
		s := strings.TrimSpace(t)
		if i, err := strconv.Atoi(s); err == nil {
			return i, true
		}
		if f, err := strconv.ParseFloat(s, 64); err == nil {
			return int(f), true
		}
		return 0, false
	default:
		return 0, false
	}
}

// toFloat normalizes various types into a float64.
func toFloat(v interface{}) (float64, bool) {
	switch t := v.(type) {
	case float64:
		return t, true
	case float32:
		return float64(t), true
	case int:
		return float64(t), true
	case int64:
		return float64(t), true
	case string:
		s := strings.TrimSpace(t)
		f, err := strconv.ParseFloat(s, 64)
		if err != nil {
			return 0, false
		}
		return f, true
	default:
		return 0, false
	}
}

// toBool normalizes various types into a bool.
func toBool(v interface{}) (bool, bool) {
	switch t := v.(type) {
	case bool:
		return t, true
	case int:
		return t != 0, true
	case int64:
		return t != 0, true
	case float64:
		return t != 0, true
	case string:
		s := strings.ToLower(strings.TrimSpace(t))
		switch s {
		case "1", "true", "yes", "y", "on":
			return true, true
		case "0", "false", "no", "n", "off":
			return false, true
		default:
			return false, false
		}
	default:
		return false, false
	}
}
