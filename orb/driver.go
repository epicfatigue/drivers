package orb

import (
	"fmt"
	"math"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	driverName  = "orb"
	channelName = "0"
)

type ORB struct {
	addr byte
	bus  i2c.Bus
	meta hal.Metadata

	adcMax        int
	vref          float64
	orpRefVoltage float64
	orpRefmV      float64
}

// ----- AnalogInputPin -----

func (o *ORB) Name() string { return channelName }
func (o *ORB) Number() int  { return 0 }

func (o *ORB) Value() (float64, error) {
	// ORB returns 2 bytes. We convert to 14-bit ADC by >> 2.
	buf, err := o.bus.ReadBytes(o.addr, 2)
	if err != nil {
		return math.NaN(), err
	}
	if len(buf) != 2 {
		return math.NaN(), fmt.Errorf("unexpected ADC data length: %d", len(buf))
	}

	raw := (uint16(buf[0]) << 8) | uint16(buf[1])
	adc := int(raw >> 2) // 14-bit value

	if o.adcMax <= 0 {
		return math.NaN(), fmt.Errorf("invalid ADCMax: %d", o.adcMax)
	}
	if o.vref <= 0 {
		return math.NaN(), fmt.Errorf("invalid Vref: %f", o.vref)
	}

	voltage := (float64(adc) / float64(o.adcMax)) * o.vref
	return voltage, nil
}

func (o *ORB) Measure() (float64, error) {
	v, err := o.Value()
	if err != nil {
		return 0, err
	}
	// ORP in mV using offset-based calibration
	orp := o.orpRefmV + (v-o.orpRefVoltage)*1000.0
	return orp, nil
}

// Calibrate expects:
// - Expected: known ORP (mV)
// - Observed: measured voltage (V)
func (o *ORB) Calibrate(ms []hal.Measurement) error {
	if len(ms) == 0 {
		return fmt.Errorf("no calibration points provided")
	}
	// Use the last point provided
	m := ms[len(ms)-1]

	if m.Observed <= 0 {
		return fmt.Errorf("observed voltage must be > 0, got: %f", m.Observed)
	}
	// Expected is ORP in mV; allow any float
	o.orpRefVoltage = m.Observed
	o.orpRefmV = m.Expected
	return nil
}

func (o *ORB) Close() error { return nil }

// ----- Driver -----

func (o *ORB) Metadata() hal.Metadata { return o.meta }

func (o *ORB) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		return []hal.Pin{o}, nil
	default:
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
}

func (o *ORB) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if n != 0 {
		return nil, fmt.Errorf("orb driver has only one valid channel: 0. Asked:%d", n)
	}
	return o, nil
}

func (o *ORB) AnalogInputPins() []hal.AnalogInputPin {
	return []hal.AnalogInputPin{o}
}
