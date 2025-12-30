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

func (d *Driver) AnalogInputPins() []hal.AnalogInputPin { return d.channels }

// IMPORTANT: match by pin Number() (so if your pin is channel 2, you request 2)
func (d *Driver) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	for _, ch := range d.channels {
		if ch.Number() == n {
			return ch, nil
		}
	}
	return nil, fmt.Errorf("ads1115-tds: no analog input channel %d", n)
}

func (d *Driver) Close() error { return nil }
