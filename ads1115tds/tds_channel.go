package ads1115tds

import (
	"encoding/binary"
	"errors"
	"fmt"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

// ADS1115-only TDS channel (single-ended AINx -> raw -> volts -> linear TDS)
type tdsChannel struct {
	bus     i2c.Bus
	address byte
	channel int

	mux        uint16
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
	return float64(raw), nil
}

// Measure returns TDS (units depend on your chosen linear conversion).
func (c *tdsChannel) Measure() (float64, error) {
	raw, err := c.performConversion()
	if err != nil {
		return 0, err
	}
	volts, err := c.rawToVolts(raw)
	if err != nil {
		return 0, err
	}
	return (c.tdsK * volts) + c.tdsOffset, nil
}

func (c *tdsChannel) performConversion() (int16, error) {
	// Build ADS1115 config (single-shot)
	config := uint16(configOsSingle |
		configModeSingle |
		configComparatorModeTraditional |
		configComparitorNonLatching |
		configComparitorPolarityActiveLow |
		configComparitorQueueNone |
		c.mux |
		c.gainConfig |
		configDataRate860) // ADS1115 max SPS

	buf := make([]byte, 2)
	binary.BigEndian.PutUint16(buf, config)

	// Write config
	if err := c.bus.WriteToReg(c.address, regConfig, buf); err != nil {
		return 0, err
	}

	time.Sleep(c.delay)

	// Read back config, ignore OS bit (it changes during conversion)
	verify := make([]byte, 2)
	if err := c.bus.ReadFromReg(c.address, regConfig, verify); err != nil {
		return 0, err
	}
	written := binary.BigEndian.Uint16(buf)
	readback := binary.BigEndian.Uint16(verify)

	if (written &^ configOsSingle) != (readback &^ configOsSingle) {
		return 0, errors.New("ads1115: config mismatch (masked OS bit)")
	}

	// Read conversion register
	b := make([]byte, 2)
	if err := c.bus.ReadFromReg(c.address, regConversion, b); err != nil {
		return 0, err
	}

	// ADS1115: signed 16-bit big-endian
	v := int16(int16(b[0])<<8 | int16(b[1]))
	return v, nil
}

func (c *tdsChannel) rawToVolts(raw int16) (float64, error) {
	fs, ok := fsVoltsForGain(c.gainConfig)
	if !ok {
		return 0, fmt.Errorf("ads1115: unknown gain config: 0x%04x", c.gainConfig)
	}

	// ADS1115 full-scale: 32768 counts
	volts := (float64(raw) / 32768.0) * fs

	// Optional clamp to supplied Vref
	if c.vref > 0 && volts > c.vref {
		volts = c.vref
	}
	return volts, nil
}
