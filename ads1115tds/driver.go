package ads1115tds

import (
	"fmt"

	"github.com/reef-pi/hal"
)

type driver struct {
	channels []hal.AnalogInputPin
	meta     hal.Metadata
}

func (d *driver) Metadata() hal.Metadata {
	return d.meta
}

func (d *driver) AnalogInputPins() []hal.AnalogInputPin {
	return d.channels
}

func (d *driver) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if n < 0 || n > 3 {
		return nil, fmt.Errorf("ads1115 has no channel %d", n)
	}
	return d.channels[n], nil
}

func (d *driver) Close() error {
	return nil
}
