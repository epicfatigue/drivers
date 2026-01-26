// driver.go
//
// ADS1115 TDS driver implementation.
//
// This driver reads one ADS1115 single-ended channel (AINx vs GND) and produces:
//
//   raw ADC counts -> volts_raw (from ADS1115 gain scaling)
//   -> clamp to [0..ClampV] (single-ended expectation)
//   -> volts_ref (temperature normalized to RefTempC) IF DoTempComp enabled
//   -> TDS = (TdsK * volts_ref) + TdsOffset
//
// Key design points:
// - Snapshot() provides signals & meta so the Chemistry snapshot UI and calibration wizard
//   can see both "observed" and "primary" values.
// - Temperature is injected through SetTemperatureC() if Chemistry has a temp sensor set.
// - Temp compensation is OPTIONAL (checkbox). If enabled but temperature is missing,
//   we assume RefTempC (so normalization becomes a no-op).
//
// Temperature normalization model (conductivity-style):
//   volts_ref = volts_measured / (1 + alpha*(T - RefTempC))
//
// Notes:
// - α (alpha) is typically ~0.02 per °C for conductivity/TDS probes.
// - If your measured signal is not actually proportional to conductivity, alpha may not help.
// - The calibration wizard should use the "observed" key (volts) which becomes volts@RefTempC when enabled.
//
package ads1115tds

import (
	"encoding/binary"
	"fmt"
	"log"
	"math"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	driverType = "ads1115-tds" // must match driversMap key / UI type
	driverName = "ADS1115 TDS"

	// ADS1115 registers
	regConversion = 0x00
	regConfig     = 0x01

	// OS / Mode
	configOsSingle   uint16 = 0x8000
	configModeSingle uint16 = 0x0100

	// Data rate (SPS)
	configDataRate860 uint16 = 0x00E0 // 860 SPS (max)

	// Comparator: disabled
	configComparatorModeTraditional   uint16 = 0x0000
	configComparitorNonLatching       uint16 = 0x0000
	configComparitorPolarityActiveLow uint16 = 0x0000
	configComparitorQueueNone         uint16 = 0x0003

	// conversion poll limits (ADS1115 @ 860SPS is ~1.2ms)
	convTimeout  = 50 * time.Millisecond
	convPollWait = 200 * time.Microsecond

	// Reasonable "stale temperature" threshold for warning logs
	tempStaleWarn = 2 * time.Minute
)

var logBusTypeOnce sync.Once

// --- Gain constants (PGA / full-scale range) ---
const (
	configGainTwoThirds uint16 = 0x0000 // +/- 6.144V
	configGainOne       uint16 = 0x0200 // +/- 4.096V
	configGainTwo       uint16 = 0x0400 // +/- 2.048V
	configGainFour      uint16 = 0x0600 // +/- 1.024V
	configGainEight     uint16 = 0x0800 // +/- 0.512V
	configGainSixteen   uint16 = 0x0A00 // +/- 0.256V
)

// --- Mux (single-ended AINx vs GND) ---
const (
	configMuxSingle0 uint16 = 0x4000 // AIN0
	configMuxSingle1 uint16 = 0x5000 // AIN1
	configMuxSingle2 uint16 = 0x6000 // AIN2
	configMuxSingle3 uint16 = 0x7000 // AIN3
)

// muxForChannel returns mux bits for single-ended AINx vs GND.
func muxForChannel(ch int) (uint16, bool) {
	switch ch {
	case 0:
		return configMuxSingle0, true
	case 1:
		return configMuxSingle1, true
	case 2:
		return configMuxSingle2, true
	case 3:
		return configMuxSingle3, true
	default:
		return 0, false
	}
}

// fsVoltsForGain returns ADS1115 full-scale voltage for the selected PGA gain setting.
func fsVoltsForGain(gain uint16) (float64, bool) {
	switch gain {
	case configGainTwoThirds:
		return 6.144, true
	case configGainOne:
		return 4.096, true
	case configGainTwo:
		return 2.048, true
	case configGainFour:
		return 1.024, true
	case configGainEight:
		return 0.512, true
	case configGainSixteen:
		return 0.256, true
	default:
		return 0, false
	}
}

func gainLabel(gain uint16) string {
	switch gain {
	case configGainTwoThirds:
		return "2/3 (±6.144V)"
	case configGainOne:
		return "1 (±4.096V)"
	case configGainTwo:
		return "2 (±2.048V)"
	case configGainFour:
		return "4 (±1.024V)"
	case configGainEight:
		return "8 (±0.512V)"
	case configGainSixteen:
		return "16 (±0.256V)"
	default:
		return fmt.Sprintf("0x%04X (unknown)", gain)
	}
}

// Driver provides one AnalogInput pin (single channel per driver instance).
type Driver struct {
	meta hal.Metadata
	pin  *tdsChannel
}

func (d *Driver) Name() string           { return driverName }
func (d *Driver) Metadata() hal.Metadata { return d.meta }
func (d *Driver) Close() error           { return nil }

// Pins returns pins for the requested capability.
func (d *Driver) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		return []hal.Pin{d.pin}, nil
	default:
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
}

func (d *Driver) AnalogInputPins() []hal.AnalogInputPin { return []hal.AnalogInputPin{d.pin} }

// AnalogInputPin returns the configured channel pin if it matches n.
func (d *Driver) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if d.pin.Number() == n {
		return d.pin, nil
	}
	return nil, fmt.Errorf("%s: no analog input channel %d", driverName, n)
}

// TemperatureSetter is an optional interface the core can use to inject temperature.
// Your Chemistry subsystem can type-assert the pin to this interface and call SetTemperatureC.
type TemperatureSetter interface {
	SetTemperatureC(tempC float64)
}

// tdsChannel is a single ADS1115 input channel interpreted as TDS.
type tdsChannel struct {
	bus     i2c.Bus
	address byte
	channel int

	mux        uint16
	gainConfig uint16

	// Calibration coefficients for the final linear conversion.
	tdsK      float64
	tdsOffset float64

	// Clamp voltage to match your hardware range (usually 3.3 or 5.0).
	clampV float64

	// Temperature compensation coefficient (per °C), e.g. 0.02
	alphaPerC float64

	// Temperature compensation settings
	doTempComp bool    // checkbox
	refTempC   float64 // reference temperature (typically 25C)

	// Latest injected temperature (°C) and last update time (for staleness warnings)
	tempC         float64
	tempUpdatedAt time.Time
	tempMu        sync.Mutex

	debug bool
	meta  hal.Metadata
}

func newTdsChannel(
	b i2c.Bus,
	address byte,
	channelNum int,
	mux uint16,
	gain uint16,
	tdsK float64,
	tdsOffset float64,
	clampV float64,
	alphaPerC float64,
	doTempComp bool,
	refTempC float64,
	debug bool,
	meta hal.Metadata,
) *tdsChannel {
	c := &tdsChannel{
		bus:        b,
		address:    address,
		channel:    channelNum,
		mux:        mux,
		gainConfig: gain,
		tdsK:       tdsK,
		tdsOffset:  tdsOffset,
		clampV:     clampV,
		alphaPerC:  alphaPerC,
		doTempComp: doTempComp,
		refTempC:   refTempC,
		debug:      debug,
		meta:       meta,
	}

	// Initialize tempC to refTempC so "temp enabled but not yet injected" behaves nicely.
	c.tempC = refTempC
	return c
}

// SetTemperatureC allows Chemistry to inject a temperature used for normalization.
// This is the "external temperature" hook that matches your RoboTank driver pattern.
func (c *tdsChannel) SetTemperatureC(tempC float64) {
	c.tempMu.Lock()
	old := c.tempC
	c.tempC = tempC
	c.tempUpdatedAt = time.Now()
	c.tempMu.Unlock()

	if c.debug {
		log.Printf("ads1115tds addr=0x%02X ch=%d SetTemperatureC: %.2fC -> %.2fC (DoTempComp=%v RefTempC=%.2f alpha=%.4f)",
			c.address, c.channel, old, tempC, c.doTempComp, c.refTempC, c.alphaPerC)
	}
}

// getTemperatureC returns the latest injected temp and whether it has ever been injected.
func (c *tdsChannel) getTemperatureC() (temp float64, injected bool, updatedAt time.Time) {
	c.tempMu.Lock()
	defer c.tempMu.Unlock()

	// injected is true if we have a non-zero update time.
	if !c.tempUpdatedAt.IsZero() {
		return c.tempC, true, c.tempUpdatedAt
	}

	// If never injected, return refTempC so normalization becomes a no-op.
	return c.refTempC, false, time.Time{}
}

func (c *tdsChannel) dbg(format string, args ...any) {
	if !c.debug {
		return
	}
	log.Printf("ads1115tds addr=0x%02X ch=%d: %s", c.address, c.channel, fmt.Sprintf(format, args...))
}

func (c *tdsChannel) Name() string           { return fmt.Sprintf("%s (AIN%d)", driverName, c.channel) }
func (c *tdsChannel) Number() int            { return c.channel }
func (c *tdsChannel) Close() error           { return nil }
func (c *tdsChannel) Metadata() hal.Metadata { return c.meta }

// Calibrate is a no-op because this driver uses config-linear (TdsK/TdsOffset).
// Use the UI/config to adjust calibration coefficients.
func (c *tdsChannel) Calibrate(_ []hal.Measurement) error { return nil }

func (c *tdsChannel) Value() (float64, error) { return c.Measure() }

// Measure returns the calibrated TDS reading.
func (c *tdsChannel) Measure() (float64, error) {
	raw, voltsRaw, voltsRef, out, dbg, err := c.measureAllDebug()
	if err != nil {
		return 0, err
	}

	c.dbg("SUMMARY raw=%d volts_raw=%.6f volts_ref=%.6f out=%.6f (k=%.6f off=%.6f clamp=%.2fV alpha=%.4f DoTC=%v RefTemp=%.2f)",
		raw, voltsRaw, voltsRef, out, c.tdsK, c.tdsOffset, c.clampV, c.alphaPerC, c.doTempComp, c.refTempC)

	if c.debug {
		for _, line := range dbg {
			c.dbg("%s", line)
		}
	}

	return out, nil
}

// tempNormalize converts observed volts at temperature T into equivalent volts at RefTempC.
// This matches typical conductivity compensation:
//
//   volts_ref = volts_T / (1 + α*(T - RefTempC))
//
// IMPORTANT: This MUST happen before calibration math so the calibration remains stable (when enabled).
func tempNormalize(volts, tempC, alpha, refTempC float64) float64 {
	return volts / (1.0 + alpha*(tempC-refTempC))
}

// measureAllDebug runs the full pipeline and returns detailed debug lines:
//   raw ADC -> volts_raw -> volts_ref -> TDS output
func (c *tdsChannel) measureAllDebug() (
	raw int16,
	voltsRaw float64,
	voltsRef float64,
	out float64,
	lines []string,
	err error,
) {
	lines = []string{}

	// ---------------------------------------------------------------------
	// 1) Perform ADS1115 conversion (raw ADC counts)
	// ---------------------------------------------------------------------
	raw, convLines, err := c.performConversionDebug()
	if err != nil {
		return 0, 0, 0, 0, lines, err
	}
	lines = append(lines, convLines...)

	// ---------------------------------------------------------------------
	// 2) Convert raw ADC -> volts (gain-scaled) then clamp
	// ---------------------------------------------------------------------
	voltsRaw, voltsLines, err := c.rawToVoltsDebug(raw)
	if err != nil {
		return 0, 0, 0, 0, lines, err
	}
	lines = append(lines, voltsLines...)

	// ---------------------------------------------------------------------
	// 3) Optional: Temperature normalize volts to RefTempC
	// ---------------------------------------------------------------------
	temp, injected, updatedAt := c.getTemperatureC()

	voltsRef = voltsRaw
	if c.doTempComp {
		voltsRef = tempNormalize(voltsRaw, temp, c.alphaPerC, c.refTempC)

		// Stale / missing temperature detection (matches your RoboTank behavior)
		if !injected {
			lines = append(lines,
				fmt.Sprintf("TEMP: enabled but temperature has never been injected; using RefTempC=%.2fC (normalization is no-op).", c.refTempC),
			)
		} else {
			age := time.Since(updatedAt)
			if age > tempStaleWarn {
				lines = append(lines,
					fmt.Sprintf("TEMP: WARNING temperature is stale (age=%v, temp=%.2fC). Check temp_sensor_id / temperature subsystem updates.", age, temp),
				)
			}
		}

		lines = append(lines,
			fmt.Sprintf("TEMP: normalize volts -> volts@RefTempC"),
			fmt.Sprintf("TEMP:   DoTempComp=true temp=%.2fC (injected=%v) RefTempC=%.2fC alpha=%.4f",
				temp, injected, c.refTempC, c.alphaPerC),
			fmt.Sprintf("TEMP:   volts_ref = volts / (1 + alpha*(T-RefTempC))"),
			fmt.Sprintf("TEMP:   %.9f -> %.9f", voltsRaw, voltsRef),
		)
	} else {
		lines = append(lines,
			fmt.Sprintf("TEMP: disabled (DoTempComp=false). volts_ref := volts_raw (no normalization)"),
		)
	}

	// ---------------------------------------------------------------------
	// 4) Linear output (calibrated domain)
	// ---------------------------------------------------------------------
	out = (c.tdsK * voltsRef) + c.tdsOffset
	lines = append(lines,
		fmt.Sprintf("TDS: out = (k * volts_ref) + offset"),
		fmt.Sprintf("TDS:   k=%.9f volts_ref=%.9f => k*volts=%.9f", c.tdsK, voltsRef, c.tdsK*voltsRef),
		fmt.Sprintf("TDS:   + offset=%.9f => out=%.9f", c.tdsOffset, out),
	)

	return raw, voltsRaw, voltsRef, out, lines, nil
}

// performConversionDebug starts a single-shot conversion and returns raw ADC counts.
func (c *tdsChannel) performConversionDebug() (int16, []string, error) {
	lines := []string{}

	logBusTypeOnce.Do(func() {
		c.dbg("INJECTED I2C BUS TYPE = %T", c.bus)
	})

	// Build config word:
	// - Single-shot conversion
	// - Single-ended mux AINx vs GND
	// - Selected PGA gain
	// - 860 SPS
	// - Comparator disabled
	config := uint16(
		configOsSingle |
			configModeSingle |
			configComparatorModeTraditional |
			configComparitorNonLatching |
			configComparitorPolarityActiveLow |
			configComparitorQueueNone |
			c.mux |
			c.gainConfig |
			configDataRate860,
	)

	lines = append(lines,
		fmt.Sprintf("ADS: build config register"),
		fmt.Sprintf("ADS:   OS(single)=0x%04X mode(single)=0x%04X datarate(860)=0x%04X comp(disabled bits)=0x%04X",
			configOsSingle, configModeSingle, configDataRate860,
			(configComparatorModeTraditional|configComparitorNonLatching|configComparitorPolarityActiveLow|configComparitorQueueNone),
		),
		fmt.Sprintf("ADS:   mux=0x%04X gain=0x%04X (%s)", c.mux, c.gainConfig, gainLabel(c.gainConfig)),
		fmt.Sprintf("ADS:   FINAL cfg=0x%04X", config),
	)

	c.dbg("write cfg=0x%04X mux=0x%04X gain=0x%04X", config, c.mux, c.gainConfig)

	// Write config register (starts conversion)
	buf := []byte{byte(config >> 8), byte(config)}
	if c.debug {
		lines = append(lines, fmt.Sprintf("I2C: write reg=0x%02X bytes=%02X %02X", regConfig, buf[0], buf[1]))
	}
	if err := c.bus.WriteToReg(c.address, regConfig, buf); err != nil {
		return 0, lines, fmt.Errorf("ads1115: write config: %w", err)
	}

	// Poll OS bit until conversion complete
	deadline := time.Now().Add(convTimeout)
	cfg := make([]byte, 2)

	polls := 0
	var lastCfg uint16
	start := time.Now()

	for {
		if err := c.bus.ReadFromReg(c.address, regConfig, cfg); err != nil {
			return 0, lines, fmt.Errorf("ads1115: read config: %w", err)
		}
		lastCfg = binary.BigEndian.Uint16(cfg)
		polls++

		if lastCfg&configOsSingle != 0 {
			break
		}
		if time.Now().After(deadline) {
			elapsed := time.Since(start)
			lines = append(lines,
				fmt.Sprintf("ADS: poll OS bit TIMEOUT after %v polls=%d last_cfg=0x%04X (bytes=%02X %02X)",
					elapsed, polls, lastCfg, cfg[0], cfg[1]),
			)
			return 0, lines, fmt.Errorf("ads1115: conversion timeout (last cfg=0x%04X)", lastCfg)
		}
		time.Sleep(convPollWait)
	}

	if c.debug {
		elapsed := time.Since(start)
		lines = append(lines,
			fmt.Sprintf("ADS: poll OS bit DONE polls=%d elapsed=%v last_cfg=0x%04X (bytes=%02X %02X)",
				polls, elapsed, lastCfg, cfg[0], cfg[1]),
		)
	}

	// Read conversion register
	b := make([]byte, 2)
	if err := c.bus.ReadFromReg(c.address, regConversion, b); err != nil {
		return 0, lines, fmt.Errorf("ads1115: read conversion: %w", err)
	}
	raw := int16(binary.BigEndian.Uint16(b))

	lines = append(lines,
		fmt.Sprintf("I2C: read reg=0x%02X bytes=%02X %02X", regConversion, b[0], b[1]),
		fmt.Sprintf("ADC: raw=int16(be16)=0x%04X => %d", uint16(raw), raw),
	)

	c.dbg("conv bytes=%02X %02X raw=%d (0x%04X)", b[0], b[1], raw, uint16(raw))
	return raw, lines, nil
}

// rawToVoltsDebug converts raw ADC counts into volts using the selected gain.
// Then clamps to [0..ClampV] for single-ended usage.
func (c *tdsChannel) rawToVoltsDebug(raw int16) (float64, []string, error) {
	lines := []string{}

	fs, ok := fsVoltsForGain(c.gainConfig)
	if !ok {
		return 0, lines, fmt.Errorf("ads1115: unknown gain config: 0x%04X", c.gainConfig)
	}

	// ADS1115 code range is -32768..32767 for full scale.
	// Use /32768.0 so -32768 maps to -FS and 32767 maps to (FS - 1 LSB).
	rawF := float64(raw)
	voltsUnclamped := (rawF / 32768.0) * fs

	lines = append(lines,
		fmt.Sprintf("VOLTS: full-scale fs=%.6fV from gain=0x%04X (%s)", fs, c.gainConfig, gainLabel(c.gainConfig)),
		fmt.Sprintf("VOLTS: volts_unclamped = (raw / 32768.0) * fs"),
		fmt.Sprintf("VOLTS:   raw=%d => raw/32768=%.9f", raw, rawF/32768.0),
		fmt.Sprintf("VOLTS:   * fs=%.6f => volts_unclamped=%.9f", fs, voltsUnclamped),
	)

	volts := voltsUnclamped

	// Clamp for single-ended expectation.
	// If wiring is truly AINx vs GND and inputs are within range, raw should typically be >= 0.
	clampedHigh := false
	clampedLow := false

	if volts > c.clampV {
		volts = c.clampV
		clampedHigh = true
	}
	if volts < 0 {
		volts = 0
		clampedLow = true
	}

	if c.debug {
		if clampedHigh || clampedLow {
			lines = append(lines,
				fmt.Sprintf("VOLTS: clamp single-ended: clampV=%.3fV low=0V => volts=%.9f (high_clamp=%v low_clamp=%v)",
					c.clampV, volts, clampedHigh, clampedLow),
			)
		} else {
			lines = append(lines, fmt.Sprintf("VOLTS: no clamp applied => volts=%.9f", volts))
		}
	}

	// LSB size for context (FS / 32768)
	lsb := fs / 32768.0
	lines = append(lines, fmt.Sprintf("VOLTS: LSB ~= fs/32768 = %.12f V/count", lsb))

	// If raw is negative and you expect single-ended, call it out.
	if raw < 0 && c.debug {
		lines = append(lines,
			fmt.Sprintf("WARN: raw is negative (%d). For true single-ended AINx vs GND, raw should typically be >=0. Check wiring/reference/mux.", raw),
		)
	}

	// Guard against NaN/Inf
	if math.IsNaN(volts) || math.IsInf(volts, 0) {
		return 0, lines, fmt.Errorf("ads1115: computed volts invalid: %v", volts)
	}

	return volts, lines, nil
}

// Snapshot implements hal.SnapshotCapable so Chemistry can show raw/derived signals and wire the wizard.
func (c *tdsChannel) Snapshot() (hal.Snapshot, error) {
	raw, voltsRaw, voltsRef, out, dbgLines, err := c.measureAllDebug()
	if err != nil {
		return hal.Snapshot{}, err
	}

	// Optional: print breakdown once per snapshot when debug is enabled.
	if c.debug {
		c.dbg("SNAPSHOT breakdown:")
		for _, line := range dbgLines {
			c.dbg("%s", line)
		}
	}

	temp, injected, updatedAt := c.getTemperatureC()
	var tempAgeSec float64
	if !updatedAt.IsZero() {
		tempAgeSec = time.Since(updatedAt).Seconds()
	}

	// UI: primary reading is "value".
	// "volts" is the observed key used by the calibration wizard:
	// - If DoTempComp=true: volts == volts@RefTempC
	// - If DoTempComp=false: volts == volts_raw
	meta := map[string]any{
		"type":    driverType,
		"addr":    c.address,
		"channel": c.channel,
		"gain":    fmt.Sprintf("0x%04X", c.gainConfig),
		"mux":     fmt.Sprintf("0x%04X", c.mux),

		"tdsK":      c.tdsK,
		"tdsOffset": c.tdsOffset,
		"clampV":    c.clampV,

		// Calibration wizard wiring
		"calibration_observed_key": "volts",

		"raw_signal_key":        "volts",
		"primary_signal_key":    "value",
		"secondary_signal_keys": []string{"volts_raw", "raw", "temp_c"},

		"signal_decimals": map[string]any{
			"value":     3,
			"volts":     4,
			"volts_raw": 4,
			"raw":       0,
			"temp_c":    2,
		},

		"display_names": map[string]any{
			"value":     "TDS",
			"volts":     func() string {
				if c.doTempComp {
					return fmt.Sprintf("Observed (V @%.0f°C)", c.refTempC)
				}
				return "Observed (V)"
			}(),
			"volts_raw": "Raw Voltage (V)",
			"raw":       "ADC Raw",
			"temp_c":    "Temperature (°C)",
		},
		"display_help": map[string]any{
			"value":     "TDS computed from observed volts: (TdsK * volts) + TdsOffset. If temp compensation is enabled, volts is normalized to RefTempC.",
			"volts":     "Observed electrical signal used by calibration wizard. If temp compensation is enabled, this is volts normalized to RefTempC; otherwise it's raw volts.",
			"volts_raw": "Raw ADC input voltage after ADS1115 scaling and clamp (single-ended).",
			"raw":       "Raw ADS1115 conversion reading (signed 16-bit).",
			"temp_c":    "Injected temperature from reef-pi temperature subsystem (if configured).",
		},

		"temp_compensation": map[string]any{
			"enabled":        c.doTempComp,
			"model":          "volts_ref = volts / (1 + alpha*(T-RefTempC))",
			"alpha_per_c":    c.alphaPerC,
			"ref_c":          c.refTempC,
			"temp_used_c":    temp,
			"temp_injected":  injected,
			"temp_age_sec":   tempAgeSec,
			"stale_warn_sec": tempStaleWarn.Seconds(),
		},
	}

	notes := []string{}
	if c.doTempComp {
		notes = append(notes, fmt.Sprintf("Temperature compensation ENABLED: volts normalized to %.2f°C before TDS conversion.", c.refTempC))
		if !injected {
			notes = append(notes, "No temperature injected yet; assuming RefTempC (normalization is no-op).")
		} else if !updatedAt.IsZero() && time.Since(updatedAt) > tempStaleWarn {
			notes = append(notes, fmt.Sprintf("WARNING: temperature is stale (age=%v). Check temp sensor updates.", time.Since(updatedAt)))
		}
	} else {
		notes = append(notes, "Temperature compensation DISABLED: volts used as-is (raw volts after clamp).")
	}

	return hal.Snapshot{
		Value: out,
		Unit:  "tds",
		Signals: map[string]hal.Signal{
			// Raw ADC
			"raw": {Now: float64(raw), Unit: "counts"},

			// Electrical domain
			"volts_raw": {Now: voltsRaw, Unit: "V"},
			"volts":     {Now: voltsRef, Unit: "V"}, // observed key used for calibration wizard

			// Temperature used (refTempC if never injected)
			"temp_c": {Now: temp, Unit: "C"},
		},
		Meta:  meta,
		Notes: notes,
	}, nil
}
