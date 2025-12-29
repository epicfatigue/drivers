package orb

import (
	"fmt"
	"math"
	"sync"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	driverName = "orb"
	chName     = "0"
)

// Orb implements BOTH:
// - hal.Driver (AnalogInputDriver)
// - hal.AnalogInputPin
type Orb struct {
	mu sync.RWMutex

	addr byte
	bus  i2c.Bus

	adcMax int
	vref   float64

	// ORP calibration reference:
	// orp(mV) = refmV + (voltage - refVoltage) * 1000
	refVoltage float64
	refmV      float64

	meta hal.Metadata
}

func (o *Orb) Metadata() hal.Metadata { return o.meta }

func (o *Orb) Name() string   { return chName }
func (o *Orb) Number() int    { return 0 }
func (o *Orb) Close() error   { return nil }
func (o *Orb) Measure() (float64, error) {
	v, err := o.Value()
	if err != nil {
		return 0, err
	}

	o.mu.RLock()
	refV := o.refVoltage
	refmV := o.refmV
	o.mu.RUnlock()

	orp := refmV + (v-refV)*1000.0
	return math.Round(orp*10) / 10, nil // 0.1mV resolution like your python
}

func (o *Orb) Value() (float64, error) {
	buf, err := o.bus.ReadBytes(o.addr, 2)
	if err != nil {
		return math.NaN(), err
	}
	if len(buf) != 2 {
		return math.NaN(), fmt.Errorf("unexpected ADC data length: %d", len(buf))
	}

	raw := uint16(buf[0])<<8 | uint16(buf[1])
	adc := raw >> 2 // 14-bit ADC stored left-shifted by 2 in 16-bit payload

	o.mu.RLock()
	adcMax := o.adcMax
	vref := o.vref
	o.mu.RUnlock()

	if adcMax <= 0 {
		return math.NaN(), fmt.Errorf("invalid ADCMax: %d", adcMax)
	}

	voltage := (float64(adc) / float64(adcMax)) * vref
	return voltage, nil
}

// Calibrate updates the ORP reference point.
// We interpret: Expected = known ORP in mV, Observed = measured voltage in V
func (o *Orb) Calibrate(points []hal.Measurement) error {
	if len(points) == 0 {
		return fmt.Errorf("no calibration points provided")
	}

	// use the first point (simple 1-point offset calibration)
	p := points[0]

	o.mu.Lock()
	o.refVoltage = p.Observed
	o.refmV = p.Expected
	o.mu.Unlock()

	return nil
}

// ---- Driver interface helpers ----

func (o *Orb) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		return []hal.Pin{o}, nil
	default:
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
}

func (o *Orb) AnalogInputPins() []hal.AnalogInputPin {
	return []hal.AnalogInputPin{o}
}

func (o *Orb) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if n != 0 {
		return nil, fmt.Errorf("orb driver has only one valid channel: 0. Asked:%d", n)
	}
	return o, nil
}
