package ads1115tds

import (
	"fmt"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

type Driver struct {
	bus      i2c.Bus
	address  byte
	channels []*tdsChannel
}

func (d *Driver) Metadata() hal.Metadata {
	return hal.Metadata{
		Name:        "ads1115-tds",
		Description: "ADS1115 analog TDS driver (single channel, linear volts->tds)",
		Capabilities: []hal.Capability{
			hal.AnalogInput,
		},
	}
}

func (d *Driver) Close() error {
	if d.bus != nil {
		return d.bus.Close()
	}
	return nil
}

func (d *Driver) Pins(cap hal.Capability) ([]hal.Pin, error) {
	if cap != hal.AnalogInput {
		return []hal.Pin{}, nil
	}
	out := make([]hal.Pin, 0, len(d.channels))
	for _, ch := range d.channels {
		out = append(out, ch)
	}
	return out, nil
}

func (d *Driver) AnalogInputPins() []hal.AnalogInputPin {
	out := make([]hal.AnalogInputPin, 0, len(d.channels))
	for _, ch := range d.channels {
		out = append(out, ch)
	}
	return out
}

func (d *Driver) AnalogInputPin(pin int) (hal.AnalogInputPin, error) {
	for _, ch := range d.channels {
		if ch.Number() == pin {
			return ch, nil
		}
	}
	return nil, fmt.Errorf("ads1115-tds: analog pin not found: %d", pin)
}
