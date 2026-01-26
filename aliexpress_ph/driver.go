package aliexpress_ph

import (
	"errors"
	"fmt"
	"log"
	"math"
	"strings"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	driverName = "AliExpress I2C pH (ADC→mV→pH)"

	// ADC.cpp constants (offset-binary, mid-scale = 0V)
	adcOffsetBinaryMid = 0x20000000
	adcScale           = 536870912.0 // 2^29

	// Ideal Nernst slope magnitude at 25C, mV per pH
	idealSlope25C = 59.16
	refTempK25C   = 298.15 // 25C in Kelvin

	// Timing tuning (cheap modules often need breathing room)
	minI2CGap       = 35 * time.Millisecond  // minimum spacing between I2C transactions
	settleAfterRead = 2 * time.Millisecond   // small settle delay after successful read
	cacheMaxAge     = 250 * time.Millisecond // serve Snapshot from cache to avoid double-hit
	retryDelay      = 20 * time.Millisecond  // wait before retry on transient error
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

// AliExpressPH exposes a single analog channel:
// 0 = pH (computed from observed electrode mV + calibration anchors)
type AliExpressPH struct {
	addr byte
	bus  i2c.Bus
	meta hal.Metadata

	// Conversion / calibration parameters
	vrefV float64 // ADC Vref (V), Arduino sketch uses 2.5

	// Calibration anchors stored in mV at buffer pH values
	ph7mV  float64
	ph4mV  float64
	ph10mV float64

	// Optional slope override at 25C (mV per pH, typically negative)
	slopeOverride float64

	// Temperature compensation (explicit, disabled by default)
	doTempComp    bool
	refTempC      float64 // reference temp (typically 25C)
	tempC         float64 // injected by temp subsystem
	tempUpdatedAt time.Time

	debug bool

	// one pin
	pins []*phPin

	// Local instance lock (helpful if bus impl isn’t thread-safe)
	mu sync.Mutex

	// Timing + caching to prevent "read then snapshot" hammering
	lastXferAt   time.Time
	lastSampleAt time.Time
	lastMV       float64
	lastRaw      []byte
	lastCode     int32
}

type phPin struct {
	parent *AliExpressPH
	ch     int // only 0
}

// Allow Chemistry subsystem to inject live temperature via pin type-assertion.
func (p *phPin) SetTemperatureC(tempC float64) { p.parent.SetTemperatureC(tempC) }

// SetTemperatureC stores injected temperature. We keep timestamps for staleness warnings in Snapshot.
func (d *AliExpressPH) SetTemperatureC(tempC float64) {
	old := d.tempC
	d.tempC = tempC
	d.tempUpdatedAt = time.Now()
	if d.debug {
		log.Printf("aliexpress_ph addr=0x%02X SetTemperatureC: %.2fC -> %.2fC (doTempComp=%v refTempC=%.2f)",
			d.addr, old, d.tempC, d.doTempComp, d.refTempC)
	}
}

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
func (d *AliExpressPH) enforceMinGap(minGap time.Duration) {
	if d.lastXferAt.IsZero() {
		return
	}
	elapsed := time.Since(d.lastXferAt)
	if elapsed < minGap {
		time.Sleep(minGap - elapsed)
	}
}

// readObservedMV reads 3 bytes from the module and converts to electrode mV.
// This is the ONLY raw physical quantity the hardware provides.
func (d *AliExpressPH) readObservedMV() (mv float64, raw []byte, adcCode int32, err error) {
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
			log.Printf("aliexpress_ph addr=0x%02X cache hit age=%v mv=%.2f",
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
				log.Printf("aliexpress_ph addr=0x%02X read attempt=%d error=%v", d.addr, attempt, e)
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
				log.Printf("aliexpress_ph addr=0x%02X read attempt=%d error=%v payload=% X", d.addr, attempt, lastErr, payload)
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
				log.Printf("aliexpress_ph addr=0x%02X read attempt=%d error=%v payload=% X", d.addr, attempt, lastErr, payload)
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

// Matches ADC.cpp behaviour proven by your Python test:
// u32 = (b0<<24)|(b1<<16)|(b2<<8); u32>>=2; u32&=0x3FFFFFFF
func adcI2C24ToCode(b []byte) int32 {
	u32 := uint32(b[0])<<24 | uint32(b[1])<<16 | uint32(b[2])<<8
	u32 >>= 2
	u32 &= 0x3FFFFFFF
	return int32(u32)
}

func adcCodeToVolts(code int32, vref float64) float64 {
	// ADC.cpp:
	// adc_code -= 0x20000000
	// voltage = (adc_code/2^29)*vref
	signed := float64(int64(code) - int64(adcOffsetBinaryMid))
	return (signed / adcScale) * vref
}

// ---------------- Calibration math ----------------

// slope25C chooses the slope at 25C (mV per pH), preferring:
// 1) slopeOverride (if non-zero)
// 2) PH4/PH7 anchors if available
// 3) PH10/PH7 anchors if available
// 4) ideal fallback (-59.16 mV/pH)
func (d *AliExpressPH) slope25C(debugLog bool) float64 {
	if d.slopeOverride != 0 {
		if debugLog {
			log.Printf("aliexpress_ph addr=0x%02X slope: using override %.4f mV/pH @25C", d.addr, d.slopeOverride)
		}
		return d.slopeOverride
	}

	if d.ph4mV != 0 {
		// slope = (mV4 - mV7)/(4 - 7)
		s := (d.ph4mV - d.ph7mV) / (4.0 - 7.0)
		if debugLog {
			log.Printf("aliexpress_ph addr=0x%02X slope: from PH4/PH7 = %.4f mV/pH (PH4=%.2f PH7=%.2f)",
				d.addr, s, d.ph4mV, d.ph7mV)
		}
		return s
	}
	if d.ph10mV != 0 {
		// slope = (mV10 - mV7)/(10 - 7)
		s := (d.ph10mV - d.ph7mV) / (10.0 - 7.0)
		if debugLog {
			log.Printf("aliexpress_ph addr=0x%02X slope: from PH10/PH7 = %.4f mV/pH (PH10=%.2f PH7=%.2f)",
				d.addr, s, d.ph10mV, d.ph7mV)
		}
		return s
	}

	// Typical electrode: higher pH => lower mV => negative slope
	if debugLog {
		log.Printf("aliexpress_ph addr=0x%02X slope: fallback ideal %.4f mV/pH @25C", d.addr, -idealSlope25C)
	}
	return -idealSlope25C
}

// slopeAtTemp applies Nernst scaling if enabled.
// IMPORTANT: we only compensate because we have raw physical mV and we are not double-applying hardware compensation.
func (d *AliExpressPH) slopeAtTemp(slope25 float64) (slope float64, enabled bool, reason string) {
	if !d.doTempComp {
		return slope25, false, "disabled by configuration"
	}

	// We allow operation even if temperature is stale; Snapshot notes will warn.
	tk := d.tempC + 273.15
	if tk <= 0 {
		return slope25, false, "invalid temperature; using 25C slope"
	}
	s := slope25 * (tk / refTempK25C)
	return s, true, ""
}

// mvToPH converts observed electrode mV to pH using:
// pH = 7 + (mV - mV7)/slope
func (d *AliExpressPH) mvToPH(mv float64, debugLog bool) (ph float64, slopeUsed float64) {
	s25 := d.slope25C(debugLog)
	slope, _, _ := d.slopeAtTemp(s25)

	// Guard
	if slope == 0 || math.IsNaN(slope) || math.IsInf(slope, 0) {
		slope = -idealSlope25C
	}

	ph = 7.0 + ((mv - d.ph7mV) / slope)
	return ph, slope
}

// ---------------- phPin: hal.AnalogInputPin ----------------

func (p *phPin) Value() (float64, error) {
	mv, raw, code, err := p.parent.readObservedMV()
	if err != nil {
		if p.parent.debug {
			log.Printf("aliexpress_ph addr=0x%02X read error: %v", p.parent.addr, err)
		}
		return 0, err
	}

	ph, slope := p.parent.mvToPH(mv, p.parent.debug)

	if p.parent.debug {
		log.Printf("aliexpress_ph addr=0x%02X raw=% X adc=0x%08X observed_mv=%.2f PH7=%.2f slope=%.4f tempC=%.2f -> pH=%.4f",
			p.parent.addr, raw, uint32(code), mv, p.parent.ph7mV, slope, p.parent.tempC, ph)
	}

	// Soft clamp (optional; prevents UI spikes)
	if ph < 0 {
		ph = 0
	}
	if ph > 14 {
		ph = 14
	}
	return ph, nil
}

func (p *phPin) Measure() (float64, error) { return p.Value() }

// Calibrate accepts measurements where:
// - Expected = buffer pH (typically 4, 7, 10)
// - Observed = observed electrode mV (the calibration wizard uses meta wiring keys)
// If Observed is 0, we will read live observed mV for convenience/back-compat.
func (p *phPin) Calibrate(ms []hal.Measurement) error {
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

		switch {
		case exp == 7:
			p.parent.ph7mV = obs
			log.Printf("aliexpress_ph calibrated PH7_mV=%.2f", obs)
		case exp == 4:
			p.parent.ph4mV = obs
			log.Printf("aliexpress_ph calibrated PH4_mV=%.2f", obs)
		case exp == 10:
			p.parent.ph10mV = obs
			log.Printf("aliexpress_ph calibrated PH10_mV=%.2f", obs)
		default:
			return fmt.Errorf("%s: unsupported calibration Expected=%.3f (use 4,7,10 for pH buffers)", driverName, exp)
		}
	}
	return nil
}

func (p *phPin) Name() string           { return driverName + " (pH)" }
func (p *phPin) Number() int            { return p.ch }
func (p *phPin) Close() error           { return nil }
func (p *phPin) Metadata() hal.Metadata { return p.parent.meta }

// Snapshot implements your required UI + calibration contract.
func (p *phPin) Snapshot() (hal.Snapshot, error) {
	mv, raw, code, err := p.parent.readObservedMV()
	if err != nil {
		return hal.Snapshot{}, err
	}
	ph, slope := p.parent.mvToPH(mv, false)

	// temp-comp meta
	s25 := p.parent.slope25C(false)
	sT, enabled, reason := p.parent.slopeAtTemp(s25)
	if enabled && reason == "" {
		reason = "Nernst slope scaled by absolute temperature"
	}

	notes := []string{}
	if p.parent.doTempComp {
		if p.parent.tempUpdatedAt.IsZero() {
			notes = append(notes, "Temp compensation enabled but temperature has never been injected; results may be off.")
		} else if age := time.Since(p.parent.tempUpdatedAt); age > 2*time.Minute {
			notes = append(notes, fmt.Sprintf("Temperature is stale (age=%v); check temp sensor injection.", age))
		}
	} else {
		notes = append(notes, "Temp compensation disabled (explicit by configuration).")
	}

	meta := map[string]any{
		"channel": p.ch,

		// Calibration wiring
		"calibration_observed_key": "observed_mv",
		"raw_signal_key":           "observed_mv",
		"primary_signal_key":       "value",
		"secondary_signal_keys":    []string{"slope_used", "tempC", "ph7_mV", "ph4_mV", "ph10_mV", "adc_code"},

		"display_roles": map[string]any{
			"primary":  "Primary (pH)",
			"observed": "Observed (electrode mV)",
		},
		"display_names": map[string]any{
			"value":       "pH (calibrated)",
			"observed_mv": "Electrode (mV)",
			"slope_used":  "Slope used (mV/pH)",
			"tempC":       "Temperature (°C)",
			"ph7_mV":      "Anchor: pH7 (mV)",
			"ph4_mV":      "Anchor: pH4 (mV)",
			"ph10_mV":     "Anchor: pH10 (mV)",
			"adc_code":    "ADC code (offset-binary)",
			"raw_hex":     "Raw bytes (hex)",
		},
		"display_help": map[string]any{
			"observed_mv": "Raw physical electrode millivolts from the I2C ADC module. This is what calibration anchors map against.",
			"slope_used":  "Slope (mV per pH) computed from anchors or override; optionally temperature-scaled.",
			"ph7_mV":      "Measured electrode mV in pH 7 buffer (required anchor).",
			"ph4_mV":      "Measured electrode mV in pH 4 buffer (recommended).",
			"ph10_mV":     "Measured electrode mV in pH 10 buffer (optional).",
		},
		"signal_decimals": map[string]any{
			"value":       3,
			"observed_mv": 2,
			"slope_used":  4,
			"tempC":       2,
			"ph7_mV":      2,
			"ph4_mV":      2,
			"ph10_mV":     2,
			"adc_code":    0,
		},

		"temp_compensation": map[string]any{
			"enabled": p.parent.doTempComp && enabled,
			"reason": func() string {
				if !p.parent.doTempComp {
					return "disabled by configuration"
				}
				if reason != "" {
					return reason
				}
				return ""
			}(),
			"ref_c":    p.parent.refTempC,
			"temp_c":   p.parent.tempC,
			"slope_25": s25,
			"slope_t":  sT,
		},
	}

	return hal.Snapshot{
		Value: ph,
		Unit:  "pH",
		Signals: map[string]hal.Signal{
			"observed_mv": {Now: mv, Unit: "mV"},
			"slope_used":  {Now: slope, Unit: "mV/pH"},
			"tempC":       {Now: p.parent.tempC, Unit: "C"},
			"ph7_mV":      {Now: p.parent.ph7mV, Unit: "mV"},
			"ph4_mV":      {Now: p.parent.ph4mV, Unit: "mV"},
			"ph10_mV":     {Now: p.parent.ph10mV, Unit: "mV"},
			"adc_code":    {Now: float64(code), Unit: ""},
			"raw_hex":     {Now: 0, Unit: fmt.Sprintf("% X", raw)},
		},
		Meta: meta,
		Notes: append(notes,
			"Driver includes min-gap + cache + retry to avoid I2C timing failures during calibration UI.",
			"If you run pH + ORP drivers at the same I2C address, a global per-address lock prevents read collisions.",
		),
	}, nil
}

// ---------------- hal.Driver plumbing ----------------

func (d *AliExpressPH) Name() string           { return driverName }
func (d *AliExpressPH) Close() error           { return nil }
func (d *AliExpressPH) Metadata() hal.Metadata { return d.meta }

func (d *AliExpressPH) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if n != 0 {
		return nil, fmt.Errorf("%s supports only channel 0 (pH). Asked:%d", driverName, n)
	}
	return d.pins[0], nil
}

func (d *AliExpressPH) AnalogInputPins() []hal.AnalogInputPin {
	return []hal.AnalogInputPin{d.pins[0]}
}

func (d *AliExpressPH) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		return []hal.Pin{d.pins[0]}, nil
	default:
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
}
