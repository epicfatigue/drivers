package ads1115tds

import (
	"encoding/binary"
	"errors"
	"fmt"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

// ADS1115-only analog input pin that reports:
//   Value()   -> raw ADC code (int16 as float64)
//   Measure() -> "TDS" computed as: TDS = (TdsK * Volts) + TdsOffset
//
// Temperature compensation can be added later by adjusting the voltage->TDS mapping.
type tdsChannel struct {
	bus     i2c.Bus
	busMu   *sync.Mutex
	address byte

	channel    int
	muxConfig  uint16
	gainConfig uint16

	// conversion behaviour
	dataRate uint16
	delay    time.Duration

	// conversion knobs
	vref      float64
	tdsK      float64
	tdsOffset float64
}

func newTdsChannel(
	b i2c.Bus,
	busMu *sync.Mutex,
	address byte,
	channelNum int,
	mux uint16,
	gain uint16,
	dataRate uint16,
	delay time.Duration,
	vref float64,
	tdsK float64,
	tdsOffset float64,
) (*tdsChannel, error) {
	if channelNum < 0 || channelNum > 3 {
		return nil, fmt.Errorf("ads1115: invalid channel %d", channelNum)
	}
	if busMu == nil {
		busMu = &sync.Mutex{}
	}
	return &tdsChannel{
		bus:        b,
		busMu:      busMu,
		address:    address,
		channel:    channelNum,
		muxConfig:  mux,
		gainConfig: gain,
		dataRate:   dataRate,
		delay:      delay,
		vref:       vref,
		tdsK:       tdsK,
		tdsOffset:  tdsOffset,
	}, nil
}

func (c *tdsChannel) Name() string { return fmt.Sprintf("%d", c.channel) }
func (c *tdsChannel) Number() int  { return c.channel }
func (c *tdsChannel) Close() error { return nil }

// We’re using TdsK/TdsOffset as the calibration knobs for now.
func (c *tdsChannel) Calibrate(points []hal.Measurement) error { return nil }

// Value returns RAW ADC code (signed 16-bit) as float64.
func (c *tdsChannel) Value() (float64, error) {
	raw, err := c.performConversion()
	if err != nil {
		return 0, err
	}
	return float64(raw), nil
}

// Measure returns the derived TDS value using the linear mapping.
func (c *tdsChannel) Measure() (float64, error) {
	raw, err := c.performConversion()
	if err != nil {
		return 0, err
	}

	volts, err := c.rawToVolts(raw)
	if err != nil {
		return 0, err
	}

	// Clamp to sane range for single-ended “voltage” readings
	if volts < 0 {
		volts = 0
	}
	if c.vref > 0 && volts > c.vref {
		volts = c.vref
	}

	return (c.tdsK * volts) + c.tdsOffset, nil
}

func (c *tdsChannel) performConversion() (int16, error) {
	// ADS1115 single-shot, no comparator.
	// NOTE: configOsSingle is a “start conversion” bit in single-shot mode.
	config := configOsSingle |
		c.muxConfig |
		c.gainConfig |
		configModeSingle |
		c.dataRate |
		configComparatorModeTraditional |
		configComparatorNonLatching |
		configComparatorPolarityActiveLow |
		configComparatorQueueNone

	var cfg [2]byte
	binary.BigEndian.PutUint16(cfg[:], config)

	c.busMu.Lock()
	defer c.busMu.Unlock()

	// Write config
	if err := c.bus.WriteToReg(c.address, regConfig, cfg[:]); err != nil {
		return 0, err
	}

	time.Sleep(c.delay)

	// Optional safety: read-back config, but OS bit can change, so mask it out.
	var verify [2]byte
	if err := c.bus.ReadFromReg(c.address, regConfig, verify[:]); err != nil {
		return 0, err
	}
	written := binary.BigEndian.Uint16(cfg[:])
	readback := binary.BigEndian.Uint16(verify[:])

	if (written &^ configOsSingle) != (readback &^ configOsSingle) {
		return 0, errors.New("ads1115: config mismatch (masked OS bit)")
	}

	// Read conversion
	var b [2]byte
	if err := c.bus.ReadFromReg(c.address, regConversion, b[:]); err != nil {
		return 0, err
	}

	// Signed 16-bit big-endian
	raw := int16(int16(b[0])<<8 | int16(b[1]))
	return raw, nil
}

func (c *tdsChannel) rawToVolts(raw int16) (float64, error) {
	// PGA full-scale voltage
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
		return 0, fmt.Errorf("ads1115: unknown gain config: 0x%04x", c.gainConfig)
	}

	// ADS1115 uses 16-bit signed; full scale corresponds to 32768 counts.
	return (float64(raw) / 32768.0) * fs, nil
}
