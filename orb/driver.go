package orb

import (
	"fmt"

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
	adcMax        int
	vref          float64
	orpRefVoltage float64
	orpRefmV      float64
	meta          hal.Metadata
}

func (d *Driver) Metadata() hal.Metadata { return d.meta }

func (d *Driver) Name() string  { return chName }
func (d *Driver) Number() int   { return 0 }
func (d *Driver) Close() error  { return nil }
func (d *Driver) Measure() (float64, error) {
	v, err := d.Value()
	if err != nil {
		return 0, err
	}
	// ORP(mV) = ORP_ref_mV + (V - Vref_voltage)*1000
	return d.orpRefmV + (v-d.orpRefVoltage)*1000.0, nil
}

// Value returns the *voltage* computed from the ORB ADC reading.
func (d *Driver) Value() (float64, error) {
	if d.adcMax <= 0 {
		return 0, fmt.Errorf("invalid ADCMax: %d", d.adcMax)
	}
	buf, err := d.bus.ReadBytes(d.addr, 2)
	if err != nil {
		return 0, err
	}
	if len(buf) != 2 {
		return 0, fmt.Errorf("unexpected ADC read length: %d", len(buf))
	}

	// Module returns 2 bytes. Raw is 16-bit, then >>2 gives 14-bit ADC.
	raw := (int(buf[0]) << 8) | int(buf[1])
	adc := raw >> 2

	voltage := (float64(adc) / float64(d.adcMax)) * d.vref
	return voltage, nil
}

// Calibrate updates the reference point.
// Expected = known ORP (mV), Observed = measured voltage (V)
func (d *Driver) Calibrate(ms []hal.Measurement) error {
	if len(ms) == 0 {
		return fmt.Errorf("no calibration points provided")
	}
	// Use last point (common pattern)
	m := ms[len(ms)-1]
	d.orpRefmV = m.Expected
	d.orpRefVoltage = m.Observed
	return nil
}

func (d *Driver) AnalogInputPin(u int) (hal.AnalogInputPin, error) {
	if u != 0 {
		return nil, fmt.Errorf("orb driver has only one valid channel: 0. Asked:%d", u)
	}
	return d, nil
}

func (d *Driver) AnalogInputPins() []hal.AnalogInputPin {
	return []hal.AnalogInputPin{d}
}

func (d *Driver) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		return []hal.Pin{d}, nil
	default:
		return nil, fmt.Errorf("unsupported capability:%s", cap.String())
	}
}
