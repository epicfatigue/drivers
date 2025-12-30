package ads1115tds

import (
	"fmt"
	"log"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

var debugADS1115TDS = true

type tdsChannel struct {
	bus     i2c.Bus
	address byte
	channel int

	mux        uint16
	gainConfig uint16
	delay      time.Duration

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
	// single-shot config
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

	buf := []byte{byte(config >> 8), byte(config)}
	if err := c.bus.WriteToReg(c.address, regConfig, buf); err != nil {
		return 0, fmt.Errorf("ads1115: write config: %w", err)
	}

	// Give conversion time. 860 SPS is ~1.2ms, but add margin.
	time.Sleep(10 * time.Millisecond)
	if c.delay > 0 {
		time.Sleep(c.delay)
	}

	b := make([]byte, 2)
	if err := c.bus.ReadFromReg(c.address, regConversion, b); err != nil {
		return 0, fmt.Errorf("ads1115: read conversion: %w", err)
	}

	c.dbg("conv bytes=%02X %02X", b[0], b[1])

	raw := int16(uint16(b[0])<<8 | uint16(b[1]))
	c.dbg("read conv raw=%d (0x%04X)", raw, uint16(raw))
	return raw, nil
}

// Correct ADS1115 gain->FSR mapping
func fsVoltsForGain(gain uint16) (float64, bool) {
	switch gain {
	case 0x0000:
		return 6.144, true
	case 0x0200:
		return 4.096, true
	case 0x0400:
		return 2.048, true
	case 0x0600:
		return 1.024, true
	case 0x0800:
		return 0.512, true
	case 0x0A00:
		return 0.256, true
	default:
		return 0, false
	}
}

func (c *tdsChannel) rawToVolts(raw int16) (float64, error) {
	fs, ok := fsVoltsForGain(c.gainConfig)
	if !ok {
		return 0, fmt.Errorf("ads1115: unknown gain config: 0x%04x", c.gainConfig)
	}

	volts := (float64(raw) / 32768.0) * fs

	if c.vref > 0 && volts > c.vref {
		volts = c.vref
	}
	return volts, nil
}
