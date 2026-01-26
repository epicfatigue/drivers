package ads1115tds

import (
	"fmt"

	"github.com/reef-pi/hal"
)

const driverName = "ADS1115 TDS"

// Driver provides one or more AnalogInput pins (channels).
type Driver struct {
	channels []hal.AnalogInputPin
	meta     hal.Metadata
}

func (d *Driver) Name() string           { return driverName }
func (d *Driver) Metadata() hal.Metadata { return d.meta }
func (d *Driver) Close() error           { return nil }

func (d *Driver) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		pins := make([]hal.Pin, 0, len(d.channels))
		for _, ch := range d.channels {
			pins = append(pins, ch)
		}
		return pins, nil
	default:
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
}

func (d *Driver) AnalogInputPins() []hal.AnalogInputPin { return d.channels }

// IMPORTANT: match by pin Number()
func (d *Driver) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	for _, ch := range d.channels {
		if ch.Number() == n {
			return ch, nil
		}
	}
	return nil, fmt.Errorf("ads1115tds: no analog input channel %d", n)
}
