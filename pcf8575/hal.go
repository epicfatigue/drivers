// hal.go
//
// reef-pi HAL glue for PCF8575.
//
// This file provides:
//   - pin objects implementing hal.DigitalInputPin and hal.DigitalOutputPin
//   - a driver implementing hal.DigitalInputDriver and hal.DigitalOutputDriver
//
// Concurrency / atomicity:
//   - All I2C interactions are protected by a mutex (d.mu).
//   - We keep the lock across multi-step operations like "release -> write -> read"
//     so concurrent outlet writes cannot interleave and break input semantics.
//
package pcf8575

import (
	"fmt"
	"log"
	"sort"
	"sync"

	"github.com/reef-pi/hal"
)

// pcf8575Pin represents one bit on the expander (0..15).
type pcf8575Pin struct {
	driver *pcf8575Driver
	pin    int
}

func (p *pcf8575Pin) Name() string { return fmt.Sprintf("PCF8575:%d", p.pin) }
func (p *pcf8575Pin) Number() int  { return p.pin }
func (p *pcf8575Pin) Close() error { return nil }

func (p *pcf8575Pin) Read() (bool, error) {
	return p.driver.readPin(p.pin)
}

func (p *pcf8575Pin) Write(b bool) error {
	return p.driver.writePin(p.pin, b)
}

func (p *pcf8575Pin) LastState() bool {
	return p.driver.lastLatched(p.pin)
}

// pcf8575Driver is the reef-pi driver instance for one chip at one I2C address.
type pcf8575Driver struct {
	hwDriver *PCF8575

	// I2C address (for better logs)
	addr byte

	// Serialize ALL interactions with the chip.
	mu sync.Mutex

	// shadow holds the last "latch" we believe is on the device.
	// bit=1 => released/high (input-ish)
	// bit=0 => driven low
	shadow uint16

	// invert allows "active-low" semantics (if true: Write(true) drives LOW).
	// Not currently exposed in factory parameters (kept for compatibility/future).
	invert bool

	// debug enables verbose log messages.
	debug bool

	// meta is provided by factory (so UI name/desc stays consistent).
	meta hal.Metadata

	pins []*pcf8575Pin
}

func (d *pcf8575Driver) Close() error { return d.hwDriver.Close() }
func (d *pcf8575Driver) Metadata() hal.Metadata {
	if d.meta.Name != "" {
		return d.meta
	}
	// fallback
	return hal.Metadata{
		Name:        "pcf8575",
		Description: "Supports one PCF8575 16-bit I2C GPIO expander",
		Capabilities: []hal.Capability{
			hal.DigitalInput,
			hal.DigitalOutput,
		},
	}
}

// -----------------------------------------------------------------------------
// Required by hal.DigitalInputDriver / hal.DigitalOutputDriver
// -----------------------------------------------------------------------------

func (d *pcf8575Driver) DigitalInputPins() []hal.DigitalInputPin {
	out := make([]hal.DigitalInputPin, len(d.pins))
	for i, p := range d.pins {
		out[i] = p
	}
	return out
}

func (d *pcf8575Driver) DigitalOutputPins() []hal.DigitalOutputPin {
	out := make([]hal.DigitalOutputPin, len(d.pins))
	for i, p := range d.pins {
		out[i] = p
	}
	return out
}

func (d *pcf8575Driver) DigitalInputPin(n int) (hal.DigitalInputPin, error) {
	if n < 0 || n >= len(d.pins) {
		return nil, fmt.Errorf("pcf8575 addr=0x%02X: invalid pin %d", d.addr, n)
	}
	return d.pins[n], nil
}

func (d *pcf8575Driver) DigitalOutputPin(n int) (hal.DigitalOutputPin, error) {
	if n < 0 || n >= len(d.pins) {
		return nil, fmt.Errorf("pcf8575 addr=0x%02X: invalid pin %d", d.addr, n)
	}
	return d.pins[n], nil
}

// Optional (some parts of reef-pi may still call this).
func (d *pcf8575Driver) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.DigitalInput, hal.DigitalOutput:
		var pins []hal.Pin
		for _, p := range d.pins {
			pins = append(pins, p)
		}
		sort.Slice(pins, func(i, j int) bool { return pins[i].Name() < pins[j].Name() })
		return pins, nil
	default:
		return nil, fmt.Errorf("pcf8575 addr=0x%02X: unsupported capability: %s", d.addr, cap.String())
	}
}

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------

// lastLatched returns the last *latched* state (shadow) for a pin.
// IMPORTANT: this is not the same thing as the *actual level* on the pin.
// Use Read() to get the actual pin level.
func (d *pcf8575Driver) lastLatched(pin int) bool {
	d.mu.Lock()
	defer d.mu.Unlock()

	high := (d.shadow & (1 << pin)) != 0
	if d.invert {
		return !high
	}
	return high
}

// readPin reads the actual pin level.
//
// PCF8575 nuance:
//   - To treat a pin as input, you must "release" it (write bit=1).
//   - Then you can read the 16-bit port to see the current level.
//
// Atomicity:
//   We keep the lock held across release->write->read so concurrent writes
//   cannot change shadow or outputs mid-read.
func (d *pcf8575Driver) readPin(pin int) (bool, error) {
	if pin < 0 || pin > 15 {
		return false, fmt.Errorf("pcf8575 addr=0x%02X: read invalid pin=%d", d.addr, pin)
	}

	d.mu.Lock()
	defer d.mu.Unlock()

	mask := uint16(1 << pin)

	// Release pin for input semantics.
	prevShadow := d.shadow
	d.shadow |= mask

	if d.debug {
		log.Printf("pcf8575 addr=0x%02X read pin=%d: release bit (shadow 0x%04X -> 0x%04X)",
			d.addr, pin, prevShadow, d.shadow)
	}

	// Apply shadow to hardware before reading.
	if err := d.hwDriver.Write16(d.shadow); err != nil {
		return false, fmt.Errorf("pcf8575 addr=0x%02X read pin=%d: write shadow=0x%04X failed: %w",
			d.addr, pin, d.shadow, err)
	}

	// Read current port level.
	v, err := d.hwDriver.Read16()
	if err != nil {
		return false, fmt.Errorf("pcf8575 addr=0x%02X read pin=%d: read16 failed: %w",
			d.addr, pin, err)
	}

	level := (v & mask) != 0

	if d.debug {
		log.Printf("pcf8575 addr=0x%02X read pin=%d: port=0x%04X level=%v (shadow=0x%04X)",
			d.addr, pin, v, level, d.shadow)
	}

	return level, nil
}

// writePin applies reef-pi "on/off" semantics to the PCF8575 latch semantics.
//
// By default (invert=false):
//   - on=true  => released/high (bit=1)
//   - on=false => drive low (bit=0)
//
// If invert=true (active-low):
//   - on=true  => drive low (bit=0)
//   - on=false => release/high (bit=1)
func (d *pcf8575Driver) writePin(pin int, on bool) error {
	if pin < 0 || pin > 15 {
		return fmt.Errorf("pcf8575 addr=0x%02X: write invalid pin=%d", d.addr, pin)
	}

	released := on
	if d.invert {
		released = !on
	}

	if d.debug {
		log.Printf("pcf8575 addr=0x%02X write pin=%d on=%v invert=%v => released(bit=1)=%v",
			d.addr, pin, on, d.invert, released)
	}

	return d.setBitReleased(pin, released)
}

// setBitReleased updates shadow and writes the full 16-bit value to the chip.
func (d *pcf8575Driver) setBitReleased(pin int, released bool) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	mask := uint16(1 << pin)
	prev := d.shadow

	if released {
		d.shadow |= mask
	} else {
		d.shadow &^= mask
	}

	if d.debug {
		log.Printf("pcf8575 addr=0x%02X latch pin=%d released=%v: shadow 0x%04X -> 0x%04X",
			d.addr, pin, released, prev, d.shadow)
	}

	if err := d.hwDriver.Write16(d.shadow); err != nil {
		return fmt.Errorf("pcf8575 addr=0x%02X write pin=%d: write shadow=0x%04X failed: %w",
			d.addr, pin, d.shadow, err)
	}

	return nil
}
