package ads1115tds

import (
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

var debugADS1115TDS = true
var logBusTypeOnce sync.Once

const (
	// fixed clamp for any computed voltage
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
}

func newTdsChannel(
	b i2c.Bus,
	address byte,
	channelNum int,
	mux uint16,
	gain uint16,
	tdsK float64,
	tdsOffset float64,
) *tdsChannel {
	return &tdsChannel{
		bus:        b,
		address:    address,
		channel:    channelNum,
		mux:        mux,
		gainConfig: gain,
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

func (c *tdsChannel) Name() string                             { return fmt.Sprintf("%d", c.channel) }
func (c *tdsChannel) Number() int                              { return c.channel }
func (c *tdsChannel) Close() error                             { return nil }
func (c *tdsChannel) Calibrate(points []hal.Measurement) error { return nil }

func (c *tdsChannel) Value() (float64, error) {
	raw, err := c.performConversion()
	if err != nil {
		return 0, err
	}
	return float64(raw), nil
}

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

	c.dbg("conv bytes=%02X %02X", b[0], b[1])

	raw := int16(uint16(b[0])<<8 | uint16(b[1]))
	c.dbg("read conv raw=%d (0x%04X)", raw, uint16(raw))
	return raw, nil
}

func (c *tdsChannel) rawToVolts(raw int16) (float64, error) {
	fs, ok := fsVoltsForGain(c.gainConfig)
	if !ok {
		return 0, fmt.Errorf("ads1115: unknown gain config: 0x%04x", c.gainConfig)
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
