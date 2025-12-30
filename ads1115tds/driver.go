package ads1115tds

import (
	"fmt"

	"github.com/reef-pi/hal"
)

type Driver struct {
	channels []hal.AnalogInputPin
	meta     hal.Metadata
}

// ---- hal.Driver interface ----

func (d *Driver) Metadata() hal.Metadata {
	return d.meta
}

// Pins returns all pins exposed by this driver.
// reef-pi expects this even if pins are analog-only.
func (d *Driver) Pins() []hal.Pin {
	pins := make([]hal.Pin, 0, len(d.channels))
	for _, ch := range d.channels {
		pins = append(pins, ch)
	}
	return pins
}

// ---- Analog input support ----

func (d *Driver) AnalogInputPins() []hal.AnalogInputPin {
	return d.channels
}

// IMPORTANT: match by pin Number() (channel index)
func (d *Driver) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	for _, ch := range d.channels {
		if ch.Number() == n {
			return ch, nil
		}
	}
	return nil, fmt.Errorf("ads1115-tds: no analog input channel %d", n)
}

func (d *Driver) Close() error {
	return nil
}
