package orb

import (
	"fmt"
	"math"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	driverName = "orb"
	chName     = "0"
)

type Driver struct {
	addr          byte
	bus           i2c.Bus
	delay         time.Duration
	adcMax        int
	vref          float64
	orpRefVoltage float64
	orpRefmV      float64
	meta          hal.Metadata
}

// readADC reads 2 bytes, converts to 14-bit ADC (>>2), then voltage.
func (d *Driver) readADC() (int, float64, error) {
	buf, err := d.bus.ReadBytes(d.addr, 2)
	if err != nil {
		return 0, 0, err
	}
	if len(buf) != 2 {
		return 0, 0, fmt.Errorf("unexpected ADC length: %d", len(buf))
	}

	// raw is 16-bit container; ADC is top 14 bits (shift right by 2)
	raw := (int(buf[0]) << 8) | int(buf[1])
	adc := raw >> 2

	if adc < 0 || adc > d.adcMax {
		// Don't hard-fail, but flag weird values
		return adc, float64(adc) / float64(d.adcMax) * d.vref, nil
	}

	voltage := (float64(adc) / float64(d.adcMax)) * d.vref
	return adc, voltage, nil
}

func (d *Driver) voltageToORPmV(voltage float64) float64 {
	orp := d.orpRefmV + (voltage-d.orpRefVoltage)*1000.0
	// keep 0.1mV resolution like your python round(orp, 1)
	return math.Round(orp*10.0) / 10.0
}

/*
hal.Pin / hal.AnalogInputPin
*/

// Name is the channel name reef-pi expects.
func (d *Driver) Name() string { return chName }
func (d *Driver) Number() int  { return 0 }

// Value returns ORP in mV (already converted using current reference point)
func (d *Driver) Value() (float64, error) {
	_, v, err := d.readADC()
	if err != nil {
		return math.NaN(), err
	}
	if d.delay > 0 {
		time.Sleep(d.delay)
	}
	return d.voltageToORPmV(v), nil
}

// Measure returns ORP in mV (same as Value)
func (d *Driver) Measure() (float64, error) {
	return d.Value()
}

// Calibrate updates the reference point.
// Convention here:
//   Expected = known ORP in mV
//   Observed = measured voltage in V
func (d *Driver) Calibrate(points []hal.Measurement) error {
	if len(points) == 0 {
		return fmt.Errorf("no calibration points provided")
	}
	// Use the last point as the active reference (common/expected behavior)
	p := points[len(points)-1]
	d.orpRefmV = p.Expected
	d.orpRefVoltage = p.Observed
	return nil
}

func (d *Driver) Close() error { return nil }

/*
hal.Driver / hal.AnalogInputDriver
*/

func (d *Driver) Metadata() hal.Metadata { return d.meta }

func (d *Driver) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		return []hal.Pin{d}, nil
	default:
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
}

func (d *Driver) AnalogInputPins() []hal.AnalogInputPin {
	return []hal.AnalogInputPin{d}
}

func (d *Driver) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if n != 0 {
		return nil, fmt.Errorf("orb driver has only one valid channel: 0. Asked:%d", n)
	}
	return d, nil
}
