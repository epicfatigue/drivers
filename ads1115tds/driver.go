package ads1115tds

import (
	"fmt"

	"github.com/reef-pi/hal"
)

type Driver struct {
	channels []hal.AnalogInputPin
	meta     hal.Metadata
}

func (d *Driver) Metadata() hal.Metadata { return d.meta }

// New hal.Driver interface signature
func (d *Driver) Pins(_ hal.Capability) ([]hal.Pin, error) {
	pins := make([]hal.Pin, 0, len(d.channels))
	for _, ch := range d.channels {
		pins = append(pins, ch)
	}
	return pins, nil
}

func (d *Driver) AnalogInputPins() []hal.AnalogInputPin { return d.channels }

// IMPORTANT: match by pin Number()
func (d *Driver) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	for _, ch := range d.channels {
		if ch.Number() == n {
			return ch, nil
		}
	}
	return nil, fmt.Errorf("ads1115-tds: no analog input channel %d", n)
}

func (d *Driver) Close() error { return nil }
