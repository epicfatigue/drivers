package aliexpress_orp

import (
	"errors"
	"fmt"
	"log"
	"strings"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	driverName = "AliExpress I2C ORP (ADC→mV)"

	adcOffsetBinaryMid = 0x20000000
	adcScale           = 536870912.0 // 2^29

	// Timing tuning (cheap modules often need breathing room)
	minI2CGap       = 35 * time.Millisecond  // minimum spacing between I2C transactions
	settleAfterRead = 2 * time.Millisecond   // small settle delay after successful read
	cacheMaxAge     = 250 * time.Millisecond // serve Snapshot from cache to avoid double-hit

	retryDelay = 20 * time.Millisecond // wait before retry on transient error
)

var (
	addrMuMu sync.Mutex
	addrMu   = map[byte]*sync.Mutex{}
)

func lockForAddr(addr byte) *sync.Mutex {
	addrMuMu.Lock()
	defer addrMuMu.Unlock()
	if m, ok := addrMu[addr]; ok {
		return m
	}
	m := &sync.Mutex{}
	addrMu[addr] = m
	return m
}

// AliExpressORP exposes a single analog channel:
// 0 = ORP in mV (observed electrode mV + configured offset)
type AliExpressORP struct {
	addr byte
	bus  i2c.Bus
	meta hal.Metadata

	vrefV  float64
	offset float64 // mV offset applied after reading raw mV
	debug  bool

	pins []*orpPin

	// Optional extra protection if your i2c.Bus implementation is not thread-safe.
	// The GLOBAL per-address lock above is the important one for same-address devices.
	mu sync.Mutex

	// Timing + caching to prevent "read then snapshot" hammering
	lastXferAt   time.Time
	lastSampleAt time.Time
	lastMV       float64
	lastRaw      []byte
	lastCode     int32
}

type orpPin struct {
	parent *AliExpressORP
	ch     int
}

// Optional: silence "pin does not implement TemperatureSetter" logs.
// ORP driver intentionally ignores temperature injection.
func (p *orpPin) SetTemperatureC(tempC float64) {}

// ---------------- Low-level ADC read ----------------

func isTransientI2C(err error) bool {
	if err == nil {
		return false
	}
	s := strings.ToLower(err.Error())
	return strings.Contains(s, "remote i/o error") ||
		strings.Contains(s, "input/output error") ||
		strings.Contains(s, "eremoteio") ||
		strings.Contains(s, "eio")
}

// enforceMinGap ensures this device is not hit too quickly in succession.
// This matters even if you serialize access: some devices need recovery time.
func (d *AliExpressORP) enforceMinGap(minGap time.Duration) {
	if d.lastXferAt.IsZero() {
		return
	}
	elapsed := time.Since(d.lastXferAt)
	if elapsed < minGap {
		time.Sleep(minGap - elapsed)
	}
}

func (d *AliExpressORP) readObservedMV() (mv float64, raw []byte, adcCode int32, err error) {
	// Global lock per address prevents collisions across multiple driver instances.
	lock := lockForAddr(d.addr)
	lock.Lock()
	defer lock.Unlock()

	// Local lock keeps this instance safe too (harmless, sometimes useful).
	d.mu.Lock()
	defer d.mu.Unlock()

	// 1) Cache: if a fresh sample exists, return it (prevents /read + /snapshot double-hit)
	if !d.lastSampleAt.IsZero() && time.Since(d.lastSampleAt) < cacheMaxAge {
		if d.debug {
			log.Printf("aliexpress_orp addr=0x%02X cache hit age=%v mv=%.2f",
				d.addr, time.Since(d.lastSampleAt), d.lastMV)
		}
		return d.lastMV, append([]byte(nil), d.lastRaw...), d.lastCode, nil
	}

	// 2) Rate-limit actual I2C transactions to this device
	d.enforceMinGap(minI2CGap)

	// 3) Attempt read with one retry on transient error
	var lastErr error
	for attempt := 1; attempt <= 2; attempt++ {
		d.lastXferAt = time.Now()

		payload, e := d.bus.ReadBytes(d.addr, 3)
		if e != nil {
			lastErr = e
			if d.debug {
				log.Printf("aliexpress_orp addr=0x%02X read attempt=%d error=%v", d.addr, attempt, e)
			}
			if attempt == 1 && isTransientI2C(e) {
				time.Sleep(retryDelay)
				continue
			}
			return 0, nil, 0, e
		}

		if len(payload) != 3 {
			lastErr = fmt.Errorf("short i2c read: got %d bytes, want 3", len(payload))
			if d.debug {
				log.Printf("aliexpress_orp addr=0x%02X read attempt=%d error=%v payload=% X", d.addr, attempt, lastErr, payload)
			}
			if attempt == 1 {
				time.Sleep(10 * time.Millisecond)
				continue
			}
			return 0, payload, 0, lastErr
		}

		// Common “bus floating / no device / collision” signature
		if payload[0] == 0xFF && payload[1] == 0xFF && payload[2] == 0xFF {
			lastErr = errors.New("invalid payload: all 0xFF")
			if d.debug {
				log.Printf("aliexpress_orp addr=0x%02X read attempt=%d error=%v payload=% X", d.addr, attempt, lastErr, payload)
			}
			if attempt == 1 {
				time.Sleep(10 * time.Millisecond)
				continue
			}
			return 0, payload, 0, lastErr
		}

		code := adcI2C24ToCode(payload)
		v := adcCodeToVolts(code, d.vrefV)
		mv := v * 1000.0

		// 4) Cache last good sample (Snapshot can reuse it)
		d.lastSampleAt = time.Now()
		d.lastMV = mv
		d.lastRaw = append([]byte(nil), payload...)
		d.lastCode = code

		// 5) Small settle delay (helps cheap boards)
		time.Sleep(settleAfterRead)

		return mv, payload, code, nil
	}

	return 0, nil, 0, lastErr
}

func adcI2C24ToCode(b []byte) int32 {
	u32 := uint32(b[0])<<24 | uint32(b[1])<<16 | uint32(b[2])<<8
	u32 >>= 2
	u32 &= 0x3FFFFFFF
	return int32(u32)
}

func adcCodeToVolts(code int32, vref float64) float64 {
	signed := float64(int64(code) - int64(adcOffsetBinaryMid))
	return (signed / adcScale) * vref
}

// ---------------- orpPin: hal.AnalogInputPin ----------------

func (p *orpPin) Value() (float64, error) {
	mv, raw, code, err := p.parent.readObservedMV()
	if err != nil {
		if p.parent.debug {
			log.Printf("aliexpress_orp addr=0x%02X read error: %v", p.parent.addr, err)
		}
		return 0, err
	}

	out := mv + p.parent.offset

	if p.parent.debug {
		log.Printf("aliexpress_orp addr=0x%02X raw=% X adc=0x%08X observed_mv=%.2f offset=%.2f out=%.2f",
			p.parent.addr, raw, uint32(code), mv, p.parent.offset, out)
	}
	return out, nil
}

func (p *orpPin) Measure() (float64, error) { return p.Value() }

// Calibrate uses a simple offset model:
// offset = Expected - Observed
// Expected = known ORP solution (mV), Observed = observed_mv from snapshot.
// If Observed is 0, read live.
func (p *orpPin) Calibrate(ms []hal.Measurement) error {
	for _, m := range ms {
		exp := m.Expected
		obs := m.Observed

		if obs == 0 {
			mv, _, _, err := p.parent.readObservedMV()
			if err != nil {
				return err
			}
			obs = mv
		}

		p.parent.offset = exp - obs
		log.Printf("aliexpress_orp calibrated offset=%.2f (expected=%.2f observed=%.2f)", p.parent.offset, exp, obs)
	}
	return nil
}

func (p *orpPin) Name() string           { return driverName + " (mV)" }
func (p *orpPin) Number() int            { return p.ch }
func (p *orpPin) Close() error           { return nil }
func (p *orpPin) Metadata() hal.Metadata { return p.parent.meta }

// Snapshot (contract-compliant)
func (p *orpPin) Snapshot() (hal.Snapshot, error) {
	mv, raw, code, err := p.parent.readObservedMV()
	if err != nil {
		return hal.Snapshot{}, err
	}
	out := mv + p.parent.offset

	meta := map[string]any{
		"channel": p.ch,

		// Calibration wiring
		"calibration_observed_key": "observed_mv",
		"raw_signal_key":           "observed_mv",
		"primary_signal_key":       "value",
		"secondary_signal_keys":    []string{"offset_mv", "adc_code"},

		"display_roles": map[string]any{
			"primary":  "Primary (ORP)",
			"observed": "Observed (electrode mV)",
		},
		"display_names": map[string]any{
			"value":       "ORP (mV, calibrated)",
			"observed_mv": "Electrode (mV)",
			"offset_mv":   "Offset (mV)",
			"adc_code":    "ADC code (offset-binary)",
			"raw_hex":     "Raw bytes (hex)",
		},
		"display_help": map[string]any{
			"observed_mv": "Raw physical electrode millivolts from the I2C ADC module. Calibration adjusts via Offset.",
			"offset_mv":   "Software offset applied: ORP = observed_mv + offset.",
		},
		"signal_decimals": map[string]any{
			"value":       1,
			"observed_mv": 2,
			"offset_mv":   2,
			"adc_code":    0,
		},

		// Temperature handling (explicit!)
		"temp_compensation": map[string]any{
			"enabled": false,
			"reason":  "ORP is reported in mV; temperature compensation is not applied by this driver.",
			"ref_c":   25.0,
		},
	}

	return hal.Snapshot{
		Value: out,
		Unit:  "mV",
		Signals: map[string]hal.Signal{
			"observed_mv": {Now: mv, Unit: "mV"},
			"offset_mv":   {Now: p.parent.offset, Unit: "mV"},
			"adc_code":    {Now: float64(code), Unit: ""},
			"raw_hex":     {Now: 0, Unit: fmt.Sprintf("% X", raw)},
		},
		Meta: meta,
		Notes: []string{
			"Driver reports raw electrode mV from hardware; calibration is software offset only.",
			"Driver includes min-gap + cache + retry to avoid I2C timing failures during calibration UI.",
			"If you run pH + ORP drivers at the same I2C address, a global per-address lock prevents read collisions.",
		},
	}, nil
}

// ---------------- hal.Driver plumbing ----------------

func (d *AliExpressORP) Name() string           { return driverName }
func (d *AliExpressORP) Close() error           { return nil }
func (d *AliExpressORP) Metadata() hal.Metadata { return d.meta }

func (d *AliExpressORP) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if n != 0 {
		return nil, fmt.Errorf("%s supports only channel 0 (mV). Asked:%d", driverName, n)
	}
	return d.pins[0], nil
}

func (d *AliExpressORP) AnalogInputPins() []hal.AnalogInputPin {
	return []hal.AnalogInputPin{d.pins[0]}
}

func (d *AliExpressORP) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		return []hal.Pin{d.pins[0]}, nil
	default:
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
}
