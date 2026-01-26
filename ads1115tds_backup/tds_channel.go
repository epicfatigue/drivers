package ads1115tds

import (
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

var logBusTypeOnce sync.Once

const (
	// fixed clamp for any computed voltage (single-ended ADC input should never exceed Pi ADC ref expectation)
	fixedVref = 3.3

	// conversion poll limits (ADS1115 @ 860SPS is ~1.2ms)
	convTimeout  = 50 * time.Millisecond
	convPollWait = 200 * time.Microsecond
)

type tdsChannel struct {
	bus     i2c.Bus
	address byte
	channel int

	mux        uint16
	gainConfig uint16

	tdsK      float64
	tdsOffset float64

	delay time.Duration
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
	delay time.Duration,
	debug bool,
	meta hal.Metadata,
) *tdsChannel {
	return &tdsChannel{
		bus:        b,
		address:    address,
		channel:    channelNum,
		mux:        mux,
		gainConfig: gain,
		tdsK:       tdsK,
		tdsOffset:  tdsOffset,
		delay:      delay,
		debug:      debug,
		meta:       meta,
	}
}

func (c *tdsChannel) dbg(format string, args ...any) {
	if !c.debug {
		return
	}
	log.Printf("ads1115tds addr=0x%02X ch=%d: %s", c.address, c.channel, fmt.Sprintf(format, args...))
}

func (c *tdsChannel) Name() string                             { return fmt.Sprintf("%d", c.channel) }
func (c *tdsChannel) Number() int                              { return c.channel }
func (c *tdsChannel) Close() error                             { return nil }
func (c *tdsChannel) Metadata() hal.Metadata                   { return c.meta }
func (c *tdsChannel) Calibrate(points []hal.Measurement) error { return nil }

// Value returns the PRIMARY output (TDS) so the UI stays consistent.
func (c *tdsChannel) Value() (float64, error) { return c.Measure() }

func (c *tdsChannel) Measure() (float64, error) {
	raw, volts, out, err := c.measureAll()
	if err != nil {
		return 0, err
	}
	c.dbg("raw=%d volts=%.6f out=%.6f (k=%.6f off=%.6f)", raw, volts, out, c.tdsK, c.tdsOffset)
	return out, nil
}

func (c *tdsChannel) measureAll() (raw int16, volts float64, out float64, err error) {
	raw, err = c.performConversion()
	if err != nil {
		return 0, 0, 0, err
	}

	volts, err = c.rawToVolts(raw)
	if err != nil {
		return 0, 0, 0, err
	}

	out = (c.tdsK * volts) + c.tdsOffset
	return raw, volts, out, nil
}

func (c *tdsChannel) performConversion() (int16, error) {
	logBusTypeOnce.Do(func() {
		c.dbg("INJECTED I2C BUS TYPE = %T", c.bus)
	})

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

	c.dbg("write cfg=0x%04X mux=0x%04X gain=0x%04X", config, c.mux, c.gainConfig)

	// write config (starts conversion)
	buf := []byte{byte(config >> 8), byte(config)}
	if err := c.bus.WriteToReg(c.address, regConfig, buf); err != nil {
		return 0, fmt.Errorf("ads1115: write config: %w", err)
	}

	// optional extra settle delay (user-configurable)
	if c.delay > 0 {
		time.Sleep(c.delay)
	}

	// reuse buffer instead of allocating each loop
	cfg := make([]byte, 2)

	// wait for conversion complete by polling OS bit
	deadline := time.Now().Add(convTimeout)
	for {
		if err := c.bus.ReadFromReg(c.address, regConfig, cfg); err != nil {
			return 0, fmt.Errorf("ads1115: read config: %w", err)
		}
		cfgVal := uint16(cfg[0])<<8 | uint16(cfg[1])

		if cfgVal&configOsSingle != 0 {
			break
		}
		if time.Now().After(deadline) {
			return 0, fmt.Errorf("ads1115: conversion timeout (last cfg=0x%04X)", cfgVal)
		}
		time.Sleep(convPollWait)
	}

	// read conversion register
	b := make([]byte, 2)
	if err := c.bus.ReadFromReg(c.address, regConversion, b); err != nil {
		return 0, fmt.Errorf("ads1115: read conversion: %w", err)
	}

	raw := int16(uint16(b[0])<<8 | uint16(b[1]))
	c.dbg("conv bytes=%02X %02X raw=%d (0x%04X)", b[0], b[1], raw, uint16(raw))
	return raw, nil
}

func (c *tdsChannel) rawToVolts(raw int16) (float64, error) {
	fs, ok := fsVoltsForGain(c.gainConfig)
	if !ok {
		return 0, fmt.Errorf("ads1115: unknown gain config: 0x%04X", c.gainConfig)
	}

	volts := (float64(raw) / 32768.0) * fs

	// fixed clamp at 3.3V (and no negatives for single-ended)
	if volts > fixedVref {
		volts = fixedVref
	}
	if volts < 0 {
		volts = 0
	}
	return volts, nil
}

// Snapshot provides driver-defined UI rendering control (no UI hardcoding).
func (c *tdsChannel) Snapshot() (hal.Snapshot, error) {
	raw, volts, out, err := c.measureAll()
	if err != nil {
		return hal.Snapshot{}, err
	}

	// keep the UI tidy: only show volts/raw when user expands derived
	secondary := []string{"volts", "raw"}

	roles := map[string]any{
		"primary":  "Primary (TDS)",
		"observed": "Observed (ADC)",
	}

	names := map[string]any{
		"value": "TDS",
		"raw":   "ADC Raw",
		"volts": "Voltage",
	}

	help := map[string]any{
		"value": "TDS computed from volts using linear model: (TdsK * volts) + TdsOffset.",
		"volts": "ADC input voltage after ADS1115 scaling (clamped for single-ended use).",
		"raw":   "Raw ADS1115 conversion reading (signed 16-bit).",
	}

	meta := map[string]any{
		"addr":    c.address,
		"channel": c.channel,
		"gain":    fmt.Sprintf("0x%04X", c.gainConfig),
		"mux":     fmt.Sprintf("0x%04X", c.mux),

		"tdsK":      c.tdsK,
		"tdsOffset": c.tdsOffset,

		// generic UI keys
		"raw_signal_key":     "raw",
		"primary_signal_key": "value",
		"secondary_signal_keys": secondary,

		"signal_decimals": map[string]any{
			"value": 3,  // TDS
			"volts": 4,  // volts
			"raw":   0,  // integer
		},

		"display_roles": roles,
		"display_names": names,
		"display_help":  help,
	}

	return hal.Snapshot{
		Value: out,
		Unit:  "tds", // keep generic; change to "ppm" if you want UI units
		Signals: map[string]hal.Signal{
			"raw":   {Now: float64(raw), Unit: "counts"},
			"volts": {Now: volts, Unit: "V"},
		},
		Meta: meta,
	}, nil
}
