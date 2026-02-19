// factory.go
package robotank_conductivity

import (
	"encoding/json"
	"errors"
	"log"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

type factory struct {
	meta       hal.Metadata
	parameters []hal.ConfigParameter
}

const (
	addressParam   = "Address"
	absDRODIParam  = "AbsD_RODI"
	absDStdParam   = "AbsD_Std"
	alphaPerCParam = "AlphaPerC"
	debugParam     = "Debug"
)

// fixed, non-configurable read delay
const fixedDelayMs = 200

var f *factory
var once sync.Once

func Factory() hal.DriverFactory {
	once.Do(func() {
		f = &factory{
			meta: hal.Metadata{
				Name:        driverName,
				Description: "Robo-Tank conductivity circuit (µS/cm + ppt). Assumes 25°C reference and 53,000 µS/cm standard calibration solution.",
				Capabilities: []hal.Capability{
					hal.AnalogInput,
				},
			},
			parameters: []hal.ConfigParameter{
				{
					Name:        addressParam,
					Type:        hal.Integer,
					Order:       0,
					Default:     106,
					Description: "I²C 7-bit address of the Robo-Tank conductivity board.",
				},
				{
					Name:        absDRODIParam,
					Type:        hal.Decimal,
					Order:       1,
					Default:     1010,
					Description: "Absolute |U−V| reading (mV) measured in RO/DI water.",
				},
				{
					Name:        absDStdParam,
					Type:        hal.Decimal,
					Order:       2,
					Default:     24.328,
					Description: "Absolute |U−V| reading (mV) measured in 53,000 µS/cm calibration solution at 25°C.",
				},
				{
					Name:        alphaPerCParam,
					Type:        hal.Decimal,
					Order:       3,
					Default:     fixedAlphaPerC,
					Description: "Temperature coefficient (per °C) used for compensation to 25°C.",
				},
				{
					Name:        debugParam,
					Type:        hal.Boolean,
					Order:       4,
					Default:     false,
					Description: "Enable verbose logging of raw readings, temperature compensation, and scaling calculations.",
				},
			},
		}
	})
	return f
}

func (f *factory) Metadata() hal.Metadata { return f.meta }
func (f *factory) GetParameters() []hal.ConfigParameter {
	return f.parameters
}

func (f *factory) ValidateParameters(parameters map[string]interface{}) (bool, map[string][]string) {
  failures := make(map[string][]string)

  address, ok := getAny(parameters, addressParam)
  if !ok {
    failures[addressParam] = append(failures[addressParam], "Address parameter is required")
    return false, failures
  }

  val, ok := toInt(address)
  if !ok {
    failures[addressParam] = append(failures[addressParam], "Address must be an integer")
  } else if val < 0 || val > 127 {
    failures[addressParam] = append(failures[addressParam], "Address must be 0..127 (7-bit)")
  }

  absRODI := getFloatAny(parameters, f.defaultFloatParam(absDRODIParam, 0), absDRODIParam)
  absSTD  := getFloatAny(parameters, f.defaultFloatParam(absDStdParam, 0),  absDStdParam)

  if absRODI <= 0 {
    failures[absDRODIParam] = append(failures[absDRODIParam], "AbsD_RODI must be > 0")
  }
  if absSTD <= 0 {
    failures[absDStdParam] = append(failures[absDStdParam], "AbsD_Std must be > 0")
  }
  if absRODI > 0 && absSTD > 0 && absRODI == absSTD {
    failures[absDStdParam] = append(failures[absDStdParam], "AbsD_RODI and AbsD_Std must be different")
  }

  alpha := getFloatAny(parameters, f.defaultFloatParam(alphaPerCParam, fixedAlphaPerC), alphaPerCParam)
  if alpha < 0 {
    failures[alphaPerCParam] = append(failures[alphaPerCParam], "AlphaPerC must be >= 0")
  } else if alpha > 0.05 {
    failures[alphaPerCParam] = append(failures[alphaPerCParam], "AlphaPerC is unusually high (expected ~0.0 to 0.05 per °C)")
  }

  return len(failures) == 0, failures
}


func (f *factory) NewDriver(parameters map[string]interface{}, hardwareResources interface{}) (hal.Driver, error) {
  if valid, failures := f.ValidateParameters(parameters); !valid {
    return nil, errors.New(hal.ToErrorString(failures))
  }

  if b, err := json.MarshalIndent(parameters, "", "  "); err == nil {
    log.Printf("robotank_cond NewDriver parameters:\n%s", string(b))
  }

  bus, ok := hardwareResources.(i2c.Bus)
  if !ok {
    return nil, errors.New("robotank_cond: expected i2c.Bus hardware resource")
  }

  addrRaw, _ := getAny(parameters, addressParam)
  addrInt, _ := toInt(addrRaw)

  absRODI := getFloatAny(parameters, f.defaultFloatParam(absDRODIParam, 0), absDRODIParam)
  absSTD  := getFloatAny(parameters, f.defaultFloatParam(absDStdParam, 0),  absDStdParam)

  alphaPerC := getFloatAny(parameters, f.defaultFloatParam(alphaPerCParam, fixedAlphaPerC), alphaPerCParam)

  debug := getBoolAny(parameters, f.defaultBoolParam(debugParam, false), debugParam)


  refUS := fixedRefUS
  refTempC := fixedRefTempC

  d := &RoboTankConductivity{
    addr:      byte(addrInt),
    bus:       bus,
    delay:     time.Duration(fixedDelayMs) * time.Millisecond,
    absDFresh: absRODI,
    absDStd:   absSTD,

    refUS:     refUS,
    refTempC:  refTempC,
    alphaPerC: alphaPerC,

    tempC:     refTempC,
    tempValid: false,

    debug: debug,
    meta:  f.meta,
  }

  d.pins = []*rtPin{
    {parent: d, ch: 0},
    {parent: d, ch: 1},
  }

  log.Printf(
    "robotank_cond init addr=%d AbsD_RODI=%.3f AbsD_Std=%.3f RefUS=%.1f(fixed) RefTempC=%.2f(fixed) Alpha=%.6f(config) TempValid=%v TempC=%.2f(init) Delay=%v Debug=%v",
    d.addr, d.absDFresh, d.absDStd, d.refUS, d.refTempC, d.alphaPerC, d.tempValid, d.tempC, d.delay, d.debug,
  )

  return d, nil
}


// ----------------- helpers -----------------

// normKey makes lookups resilient to "AbsD_RODI" vs "Absd_rodi" vs "absd-rodi" etc.
func normKey(s string) string {
	s = strings.ToLower(strings.TrimSpace(s))
	s = strings.ReplaceAll(s, "_", "")
	s = strings.ReplaceAll(s, "-", "")
	s = strings.ReplaceAll(s, " ", "")
	return s
}

// getAny matches keys case-insensitively and ignoring _ - spaces.
func getAny(m map[string]interface{}, keys ...string) (interface{}, bool) {
	if len(m) == 0 {
		return nil, false
	}

	// Build normalized index once per call (cheap: maps are small)
	idx := make(map[string]interface{}, len(m))
	for k, v := range m {
		idx[normKey(k)] = v
	}

	for _, k := range keys {
		if v, ok := idx[normKey(k)]; ok {
			return unwrapValue(v), true
		}
	}
	return nil, false
}

func getFloatAny(m map[string]interface{}, def float64, keys ...string) float64 {
	v, ok := getAny(m, keys...)
	if !ok {
		return def
	}
	if f, ok := toFloat(v); ok {
		return f
	}
	return def
}

func getIntAny(m map[string]interface{}, def int, keys ...string) int {
	v, ok := getAny(m, keys...)
	if !ok {
		return def
	}
	if i, ok := toInt(v); ok {
		return i
	}
	if f, ok := toFloat(v); ok {
		return int(f)
	}
	return def
}

func getBoolAny(m map[string]interface{}, def bool, keys ...string) bool {
	v, ok := getAny(m, keys...)
	if !ok {
		return def
	}
	if b, ok := toBool(v); ok {
		return b
	}
	return def
}

func toInt(v interface{}) (int, bool) {
	v = unwrapValue(v)
	switch t := v.(type) {
	case int:
		return t, true
	case int64:
		return int(t), true
	case float64:
		return int(t), true
	case json.Number:
		i, err := t.Int64()
		if err != nil {
			return 0, false
		}
		return int(i), true
	case string:
		s := strings.TrimSpace(t)
		s = strings.ReplaceAll(s, ",", ".")
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

func toFloat(v interface{}) (float64, bool) {
	v = unwrapValue(v)
	switch t := v.(type) {
	case float64:
		return t, true
	case float32:
		return float64(t), true
	case int:
		return float64(t), true
	case int64:
		return float64(t), true
	case json.Number:
		f, err := t.Float64()
		if err != nil {
			return 0, false
		}
		return f, true
	case string:
		s := strings.TrimSpace(t)
		s = strings.ReplaceAll(s, ",", ".")
		f, err := strconv.ParseFloat(s, 64)
		if err != nil {
			return 0, false
		}
		return f, true
	default:
		return 0, false
	}
}

func toBool(v interface{}) (bool, bool) {
	v = unwrapValue(v)
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

func unwrapValue(v interface{}) interface{} {
	if m, ok := v.(map[string]interface{}); ok {
		for _, k := range []string{"value", "Value", "current", "Current", "val", "Val"} {
			if vv, ok := m[k]; ok {
				return vv
			}
		}
	}
	return v
}


func (f *factory) defaultFloatParam(name string, fallback float64) float64 {
	for _, p := range f.parameters {
		if normKey(p.Name) == normKey(name) {
			if fv, ok := toFloat(p.Default); ok {
				return fv
			}
		}
	}
	return fallback
}

func (f *factory) defaultIntParam(name string, fallback int) int {
	for _, p := range f.parameters {
		if normKey(p.Name) == normKey(name) {
			if iv, ok := toInt(p.Default); ok {
				return iv
			}
			if fv, ok := toFloat(p.Default); ok {
				return int(fv)
			}
		}
	}
	return fallback
}

func (f *factory) defaultBoolParam(name string, fallback bool) bool {
	for _, p := range f.parameters {
		if normKey(p.Name) == normKey(name) {
			if bv, ok := toBool(p.Default); ok {
				return bv
			}
		}
	}
	return fallback
}
