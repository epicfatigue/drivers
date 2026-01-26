// pcf8575.go
//
// Low-level PCF8575 I2C access.
//
// The PCF8575 has no internal registers. Every write is 2 bytes (LSB first)
// setting the latch for all 16 pins; every read is 2 bytes (LSB first)
// returning the current port level.
//
// Higher-level semantics (shadow state, pin release vs drive low, inversion, etc.)
// are implemented in hal.go.
//
package pcf8575

import (
	"fmt"

	"github.com/reef-pi/rpi/i2c"
)

type PCF8575 struct {
	addr byte
	bus  i2c.Bus
}

func New(addr byte, bus i2c.Bus) *PCF8575 {
	return &PCF8575{addr: addr, bus: bus}
}

// Write16 writes the 16-bit latch value (LSB first).
// bit=1 => release/high (input-ish); bit=0 => drive low.
func (p *PCF8575) Write16(v uint16) error {
	b := []byte{byte(v & 0xFF), byte((v >> 8) & 0xFF)}
	return p.bus.WriteBytes(p.addr, b)
}

// Read16 reads the current pin levels (LSB first).
func (p *PCF8575) Read16() (uint16, error) {
	// reef-pi i2c.Bus signature:
	//   ReadBytes(addr byte, n int) ([]byte, error)
	b, err := p.bus.ReadBytes(p.addr, 2)
	if err != nil {
		return 0, err
	}
	if len(b) < 2 {
		return 0, fmt.Errorf("pcf8575 addr=0x%02X: short read: got %d bytes", p.addr, len(b))
	}
	return uint16(b[0]) | (uint16(b[1]) << 8), nil
}

func (p *PCF8575) Close() error { return nil }

// ReleaseAll releases all pins (writes 0xFFFF).
func (p *PCF8575) ReleaseAll() error { return p.Write16(0xFFFF) }
