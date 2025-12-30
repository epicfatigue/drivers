package ads1115tds

import (
	"encoding/binary"
	"errors"
	"fmt"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

// ADS1115-only TDS channel (single-ended voltage -> TDS via K + offset)
type tdsChannel struct {
	bus        i2c.Bus
	address    byte
	mux        uint16
	channel    int
	gainConfig uint16
	delay      time.Duration

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
) (*tdsChannel, error) {
	return &tdsChannel{
		bus:        b,
		address:    address,
		mux:        mux,
		channel:    channelNum,
		gainConfig: gain,
		delay:      delay,
		vref:       vref,
		tdsK:       tdsK,
		tdsOffset:  tdsOffset,
	}, nil
}

func (c *tdsChannel) Name() string { return fmt.Sprintf("%d", c.channel) }
func (c *tdsChannel) Number() int  { return c.channel }
func (c *tdsChannel) Close() error { return nil }

// No point-calibration for now (you’re using K/Offset knobs)
func (c *tdsChannel) Calibrate(points []hal.Measurement) error { return nil }

// Value returns RAW ADC code (signed int16) as float64 for debugging.
func (c *tdsChannel) Value() (float64, error) {
	raw, err := c.performConversion()
	if err != nil {
		return 0, err
	}
	return float64(raw), nil
}

// Measure returns "TDS" (whatever units your K/offset imply).
func (c *tdsChannel) Measure() (float64, error) {
	raw, err := c.performConversion()
	if err != nil {
		return 0, err
	}

	volts, err := c.rawToVolts(raw)
	if err != nil {
		return 0, err
	}

	// Simple linear conversion for now:
	// TDS = K * volts + offset
	return (c.tdsK * volts) + c.tdsOffset, nil
}

func (c *tdsChannel) performConversion() (int16, error) {
	// Build ADS1115 config word
	config := uint16(0)
	config |= configOsSingle
	config |= c.mux
	config |= c.gainConfig
	config |= configModeSingle
	config |= configDataRate860
	config |= configComparatorModeTraditional
	config |= configComparitorNonLatching
	config |= configComparitorPolarityActiveLow
	config |= configComparitorQueueNone

	// Write config
	var buf [2]byte
	binary.BigEndian.PutUint16(buf[:], config)
	if err := c.bus.WriteToReg(c.address, regConfig, buf[:]); err != nil {
		return 0, err
	}

	// Wait for conversion (simple + reliable)
	time.Sleep(c.delay)

	// Optional sanity check: read back config but ignore OS bit (it changes during conversion)
	var verify [2]byte
	if err := c.bus.ReadFromReg(c.address, regConfig, verify[:]); err != nil {
		return 0, err
	}
	written := binary.BigEndian.Uint16(buf[:])
	readback := binary.BigEndian.Uint16(verify[:])
	if (written &^ configOsSingle) != (readback &^ configOsSingle) {
		return 0, errors.New("ads1115 config mismatch (masked OS bit)")
	}

	// Read conversion
	var out [2]byte
	if err := c.bus.ReadFromReg(c.address, regConversion, out[:]); err != nil {
		return 0, err
	}

	// ADS1115 conversion register is signed big-endian
	raw := int16(int16(out[0])<<8 | int16(out[1]))
	return raw, nil
}

func (c *tdsChannel) rawToVolts(raw int16) (float64, error) {
	// Full-scale voltage depends on gain setting
	var fs float64
	switch c.gainConfig {
	case configGainTwoThirds:
		fs = 6.144
	case configGainOne:
		fs = 4.096
	case configGainTwo:
		fs = 2.048
	case configGainFour:
		fs = 1.024
	case configGainEight:
		fs = 0.512
	case configGainSixteen:
		fs = 0.256
	default:
		return 0, fmt.Errorf("unknown gain config: 0x%04x", c.gainConfig)
	}

	// ADS1115: signed 16-bit where full-scale is 32768 counts
	volts := (float64(raw) / 32768.0) * fs

	// Single-ended should not be negative; clamp small negative noise.
	if volts < 0 {
		volts = 0
	}

	// Optional clamp to Vref if provided (helps if frontend cannot exceed Vref)
	if c.vref > 0 && volts > c.vref {
		volts = c.vref
	}
	return volts, nil
}
