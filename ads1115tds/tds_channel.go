package ads1115tds

import (
	"fmt"
	"log"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

// Toggle this while debugging. Set false once stable.
var debugADS1115TDS = true

// ADS1115-only TDS channel (single-ended AINx -> raw -> volts -> linear TDS)
type tdsChannel struct {
	bus     i2c.Bus
	address byte
	channel int

	mux        uint16
	gainConfig uint16
	delay      time.Duration // optional extra delay (can be 0)

	// Conversion knobs
	vref      float64
	tdsK      float64
	tdsOffset float64
}

func newTdsChannel(
	b i2c.Bus,
	address byte,
	channelNum int,
	mux uint16,
	gain uint16,
	delay time.Duration,
	vref float64,
	tdsK float64,
	tdsOffset float64,
) *tdsChannel {
	return &tdsChannel{
		bus:        b,
		address:    address,
		channel:    channelNum,
		mux:        mux,
		gainConfig: gain,
		delay:      delay,
		vref:       vref,
		tdsK:       tdsK,
		tdsOffset:  tdsOffset,
	}
}

func (c *tdsChannel) dbg(format string, args ...any) {
	if !debugADS1115TDS {
		return
	}
	log.Printf("ads1115tds addr=0x%02X ch=%d: %s", c.address, c.channel, fmt.Sprintf(format, args...))
}

func (c *tdsChannel) Name() string { return fmt.Sprintf("%d", c.channel) }
func (c *tdsChannel) Number() int  { return c.channel }
func (c *tdsChannel) Close() error { return nil }

// We use TdsK/TdsOffset as calibration knobs for now.
func (c *tdsChannel) Calibrate(points []hal.Measurement) error { return nil }

// Value returns RAW ADC code (signed int16) as float64.
func (c *tdsChannel) Value() (float64, error) {
	raw, err := c.performConversion()
	if err != nil {
		return 0, err
	}
	// Raw code is useful for debugging (reef-pi may call Value() for graphs)
	return float64(raw), nil
}

// Measure returns a linear conversion of volts: (tdsK*volts)+tdsOffset.
// (You can set k=1, offset=0 to effectively expose volts.)
func (c *tdsChannel) Measure() (float64, error) {
	raw, err := c.performConversion()
	if err != nil {
		return 0, err
	}

	volts, err := c.rawToVolts(raw)
	if err != nil {
		return 0, err
	}

	out := (c.tdsK * volts) + c.tdsOffset
	c.dbg("raw=%d volts=%.6f out=%.6f (k=%.4f off=%.4f)", raw, volts, out, c.tdsK, c.tdsOffset)
	return out, nil
}

func (c *tdsChannel) performConversion() (int16, error) {
	// Build ADS1115 config (single-shot)
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

	// Write config (big-endian)
	buf := []byte{byte(config >> 8), byte(config)}
	if err := c.bus.WriteToReg(c.address, regConfig, buf); err != nil {
		return 0, fmt.Errorf("ads1115: write config: %w", err)
	}

	// Optional extra delay (in addition to polling)
	if c.delay > 0 {
		time.Sleep(c.delay)
	}

	// Wait for conversion complete by polling OS bit (bit 15 goes 1 when done)
	deadline := time.Now().Add(250 * time.Millisecond)
	var lastCfg uint16

	for {
		if time.Now().After(deadline) {
			c.dbg("timeout waiting for OS bit, last cfg=0x%04X", lastCfg)
			return 0, fmt.Errorf("ads1115: conversion timeout")
		}

		verify := make([]byte, 2)
		if err := c.bus.ReadFromReg(c.address, regConfig, verify); err != nil {
			return 0, fmt.Errorf("ads1115: read config: %w", err)
		}

		lastCfg = uint16(verify[0])<<8 | uint16(verify[1])
		if (lastCfg & 0x8000) != 0 { // OS bit set => done
			break
		}

		time.Sleep(2 * time.Millisecond)
	}

	// Read conversion register
	b := make([]byte, 2)
	if err := c.bus.ReadFromReg(c.address, regConversion, b); err != nil {
		return 0, fmt.Errorf("ads1115: read conversion: %w", err)
	}

	raw := int16(uint16(b[0])<<8 | uint16(b[1]))
	c.dbg("read conv raw=%d (0x%04X)", raw, uint16(raw))
	return raw, nil
}

func (c *tdsChannel) rawToVolts(raw int16) (float64, error) {
	fs, ok := fsVoltsForGain(c.gainConfig)
	if !ok {
		return 0, fmt.Errorf("ads1115: unknown gain config: 0x%04x", c.gainConfig)
	}

	// ADS1115 full-scale: 32768 counts (signed)
	volts := (float64(raw) / 32768.0) * fs

	// Optional clamp to supplied Vref
	if c.vref > 0 && volts > c.vref {
		volts = c.vref
	}
	return volts, nil
}
