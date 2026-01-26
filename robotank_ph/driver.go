// driver.go
package robotank_ph

import (
	"fmt"
	"log"
	"math"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const driverName = "Robo-Tank pH Circuit"

// Robo-Tank firmware requires a short processing delay between
// command write and response read.
//
// Empirically stable across boards; making this configurable
// only increases misconfiguration risk.
const fixedReadDelay = 300 * time.Millisecond

// Known calibration buffer truths (do not change unless you really use other buffers)
const (
	truePH4  = 4.00
	truePH7  = 7.00
	truePH10 = 10.00
)

// The PCB firmware is described as using ~59.16mV/pH with pH7 = 0mV (25C Nernst slope).
// We cannot read raw mV from the board, but we can optionally print an *implied* mV for debugging.
const phSlopeMvPerPH = 59.16

// Driver exposes a single AnalogInput pin (0) for pH.
// Protocol observed on 0x62:
//   - Write ASCII command + "\x00"
//   - Read 32 bytes
//   - payload[0] == 1 => OK
//   - payload[1:] ASCII float, padded with 0x00 and/or 0xFF
type Driver struct {
	addr  byte
	bus   i2c.Bus
	delay time.Duration
	debug bool

	// Serialize I2C "write cmd -> wait -> read payload" sequences.
	// This prevents concurrent /read and /snapshot callers from interleaving and causing 0xFF payloads.
	mu sync.Mutex

	// Software calibration anchors (OBSERVED readings) from buffer solutions.
	// These are the pH values REPORTED BY THE PCB while the probe sits in known buffers.
	//
	// Example workflow (best practice):
	// - Put probe in pH 7.00 buffer, wait stable, note reading => set Obs7 to that number.
	// - Put probe in pH 4.00 (or 10.00), wait stable, note reading => set Obs4 / Obs10.
	//
	// Use -1 to disable.
	obs4  float64
	obs7  float64
	obs10 float64

	meta hal.Metadata
	pin  *phPin
}

type phPin struct {
	d *Driver
}

func (p *phPin) Name() string           { return driverName + " (pH)" }
func (p *phPin) Number() int            { return 0 }
func (p *phPin) Close() error           { return nil }
func (p *phPin) Metadata() hal.Metadata { return p.d.meta }

func (p *phPin) Measure() (float64, error) { return p.Value() }

func (p *phPin) Value() (float64, error) {
	raw, err := p.d.readFloat("R")
	if err != nil {
		if p.d.debug {
			log.Printf("robotank_ph addr=0x%02X read error: %v", p.d.addr, err)
		}
		return 0, err
	}

	cal := p.d.applyCalibration(raw)

	if p.d.debug {
		mv := phToImpliedMv(raw)
		mvCal := phToImpliedMv(cal)
		log.Printf(
			"robotank_ph addr=0x%02X raw=%.4f (~%.2fmV) cal=%.4f (~%.2fmV) obs(4=%.4f 7=%.4f 10=%.4f)",
			p.d.addr, raw, mv, cal, mvCal, p.d.obs4, p.d.obs7, p.d.obs10,
		)
	}

	return cal, nil
}

// Snapshot implements hal.SnapshotCapable (used by chemistry snapshot.go).
// This is what makes the calibration wizard show Observed + Driver meta.
func (p *phPin) Snapshot() (hal.Snapshot, error) {
	// Read raw pH reported by the Robo-Tank board.
	// This call is serialized internally (d.mu) to protect the I2C transaction.
	raw, err := p.d.readFloat("R")
	if err != nil {
		if p.d.debug {
			log.Printf("robotank_ph addr=0x%02X snapshot read error: %v", p.d.addr, err)
		}
		return hal.Snapshot{}, err
	}

	// Apply software calibration anchors (Obs4 / Obs7 / Obs10).
	// No temperature compensation is applied here (by design).
	cal := p.d.applyCalibration(raw)

	// ---------------------------------------------------------------------
	// Signals
	// ---------------------------------------------------------------------
	// Drivers expose instantaneous values only.
	// Rolling averages are added later by the chemistry subsystem.
	signals := map[string]hal.Signal{
		// Raw board-reported pH.
		// Used by the calibration wizard as the "Observed" signal.
		"observed": {
			Now:  raw,
			Unit: "pH",
		},

		// Diagnostic-only derived value.
		// This is NOT raw electrode mV.
		// It is back-calculated assuming the board's fixed
		// 59.16 mV/pH slope at 25 °C.
		"implied_mv": {
			Now:  phToImpliedMv(raw),
			Unit: "mV",
		},
	}

	// ---------------------------------------------------------------------
	// Meta: UI + calibration contract
	// ---------------------------------------------------------------------
	meta := map[string]interface{}{
		// Identifies which signal represents the observed (pre-calibration) value
		"calibration_observed_key": "observed",

		// Explicit wiring for the calibration wizard UI
		"primary_signal_key": "value",
		"raw_signal_key":     "observed",

		// Derived signals shown collapsed by default
		"secondary_signal_keys": []string{"implied_mv"},

		// Human-friendly labels
		"display_roles": map[string]interface{}{
			"primary":  "Primary",
			"observed": "Observed",
		},
		"display_names": map[string]interface{}{
			"value":      "pH",
			"observed":   "Observed (raw)",
			"implied_mv": "Implied mV @25°C",
		},
		"display_help": map[string]interface{}{
			"value":      "Calibrated pH after applying Obs4/Obs7/Obs10 anchors.",
			"observed":   "Raw pH as reported by the Robo-Tank board before software calibration.",
			"implied_mv": "Diagnostic only. Derived assuming 59.16 mV/pH at 25 °C. Not raw electrode mV.",
		},
		"signal_decimals": map[string]interface{}{
			"value":      3,
			"observed":   3,
			"implied_mv": 1,
		},

		// -----------------------------------------------------------------
		// Temperature handling (explicitly disabled)
		// -----------------------------------------------------------------
		"temp_compensation": map[string]interface{}{
			"enabled": false,
			"reason":  "Board outputs pH using fixed 59.16 mV/pH; raw electrode mV not available",
			"ref_c":   25.0,
		},

		// Calibration transparency
		"obs4":    p.d.obs4,
		"obs7":    p.d.obs7,
		"obs10":   p.d.obs10,
		"address": fmt.Sprintf("0x%02X", p.d.addr),
	}

	// Informational note only — never alters readings
	notes := []string{
		"Temperature compensation disabled: board uses fixed 59.16 mV/pH (25 °C reference)",
	}

	return hal.Snapshot{
		Value:   cal, // calibrated pH
		Unit:    "pH",
		Signals: signals,
		Meta:    meta,
		Notes:   notes,
	}, nil
}


// Optional: reef-pi generic calibration workflow hook.
// NOTE: We can't persist changes back into the driver config DB from here reliably,
// so this just acts as a no-op (returns nil).
// You should set Obs4/Obs7/Obs10 in the driver configuration UI.
func (p *phPin) Calibrate(ms []hal.Measurement) error {
	// Intentionally a no-op. Calibration anchors are config-driven.
	return nil
}

// ---- hal.Driver ----

func (d *Driver) Name() string           { return driverName }
func (d *Driver) Close() error           { return nil }
func (d *Driver) Metadata() hal.Metadata { return d.meta }

func (d *Driver) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if n != 0 {
		return nil, fmt.Errorf("%s supports only channel 0", driverName)
	}
	return d.pin, nil
}

func (d *Driver) AnalogInputPins() []hal.AnalogInputPin {
	return []hal.AnalogInputPin{d.pin}
}

func (d *Driver) Pins(cap hal.Capability) ([]hal.Pin, error) {
	if cap != hal.AnalogInput {
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
	return []hal.Pin{d.pin}, nil
}

// ---- Calibration math ----

type anchor struct {
	truePH float64 // true buffer pH (4/7/10)
	obsPH  float64 // observed board reading in that buffer
}

// enabledAnchors returns enabled (truePH, obsPH) pairs sorted by truePH ascending.
func (d *Driver) enabledAnchors() []anchor {
	var as []anchor
	if d.obs4 != -1 {
		as = append(as, anchor{truePH: truePH4, obsPH: d.obs4})
	}
	if d.obs7 != -1 {
		as = append(as, anchor{truePH: truePH7, obsPH: d.obs7})
	}
	if d.obs10 != -1 {
		as = append(as, anchor{truePH: truePH10, obsPH: d.obs10})
	}
	sort.Slice(as, func(i, j int) bool { return as[i].truePH < as[j].truePH })
	return as
}

type mapDebug struct {
	den float64
	t   float64
	y   float64
}

func boolSuffix(b bool, s string) string {
	if b {
		return s
	}
	return ""
}

func clampPH(v, lo, hi float64) (float64, bool) {
	if v < lo {
		return lo, true
	}
	if v > hi {
		return hi, true
	}
	return v, false
}

// linearMapDbg maps x from [x1..x2] to [y1..y2], returning debug info.
// If x1==x2 (degenerate), returns y1 and t=0.
func linearMapDbg(x, x1, x2, y1, y2 float64) mapDebug {
	den := (x2 - x1)
	if math.Abs(den) < 1e-9 {
		return mapDebug{den: den, t: 0, y: y1}
	}
	t := (x - x1) / den
	y := y1 + t*(y2-y1)
	return mapDebug{den: den, t: t, y: y}
}

// applyCalibration converts a raw pH from the PCB into a corrected pH using:
// - 1 point: offset
// - 2 points: linear map (scale + offset)
// - 3 points: piecewise linear (4–7, 7–10)
// If no anchors are set, returns raw unchanged.
func (d *Driver) applyCalibration(raw float64) float64 {
	// Safety clamp on RAW (this is before any calibration)
	rawIn := raw
	if raw < -1 {
		raw = -1
	}
	if raw > 15 {
		raw = 15
	}
	if d.debug && raw != rawIn {
		log.Printf("robotank_ph cal: raw clamp %.6f -> %.6f (pre-cal safety clamp)", rawIn, raw)
	}

	as := d.enabledAnchors()
	if len(as) == 0 {
		if d.debug {
			log.Printf("robotank_ph cal: no anchors enabled -> cal=raw (%.6f)", raw)
		}
		return raw
	}

	if d.debug {
		parts := make([]string, 0, len(as))
		for _, a := range as {
			parts = append(parts,
				fmt.Sprintf("(%0.2f->%0.4f, implied_mV_true=%+.2f, implied_mV_obs=%+.2f)",
					a.truePH, a.obsPH,
					phToImpliedMv(a.truePH),
					phToImpliedMv(a.obsPH),
				),
			)
		}
		log.Printf("robotank_ph cal: anchors enabled n=%d %s", len(as), strings.Join(parts, " "))
	}

	// 1-point: offset only
	if len(as) == 1 {
		off := as[0].truePH - as[0].obsPH
		outPre := raw + off
		out, clamped := clampPH(outPre, 0, 14)

		if d.debug {
			log.Printf(
				"robotank_ph cal: MODE=1pt offset=true-obs => off=%.6f (true=%.2f obs=%.6f) raw=%.6f => raw+off=%.6f%s",
				off, as[0].truePH, as[0].obsPH, raw, outPre, boolSuffix(clamped, " (clamped 0..14)"),
			)
			log.Printf("robotank_ph cal: RESULT cal=%.6f", out)
		}
		return out
	}

	// 2-point: scale + offset (linear map)
	if len(as) == 2 {
		dbg := linearMapDbg(raw, as[0].obsPH, as[1].obsPH, as[0].truePH, as[1].truePH)
		out, clamped := clampPH(dbg.y, 0, 14)

		if d.debug {
			scale := 0.0
			if math.Abs(dbg.den) >= 1e-9 {
				scale = (as[1].truePH - as[0].truePH) / dbg.den
			}
			offset := as[0].truePH - scale*as[0].obsPH

			log.Printf("robotank_ph cal: MODE=2pt linear map obs->[true]")
			log.Printf("robotank_ph cal:   x(raw)=%.6f", raw)
			log.Printf("robotank_ph cal:   x1=obs@true%.2f=%.6f  x2=obs@true%.2f=%.6f  den(x2-x1)=%.9f",
				as[0].truePH, as[0].obsPH, as[1].truePH, as[1].obsPH, dbg.den)
			log.Printf("robotank_ph cal:   y1=true=%.2f y2=true=%.2f  t=(x-x1)/den=%.9f", as[0].truePH, as[1].truePH, dbg.t)
			log.Printf("robotank_ph cal:   y= y1 + t*(y2-y1) => %.6f%s", dbg.y, boolSuffix(clamped, " (clamped 0..14)"))
			log.Printf("robotank_ph cal:   line form y=scale*x+offset => scale=%.9f offset=%.9f", scale, offset)
			log.Printf("robotank_ph cal: RESULT cal=%.6f", out)
		}
		return out
	}

	// 3-point: piecewise around the middle anchor (truePH7)
	// anchors sorted by truePH: [4,7,10]
	a0, a1, a2 := as[0], as[1], as[2]

	// Decide segment based on observed pH7 reading
	left := raw <= a1.obsPH
	seg := "7-10"
	if left {
		seg = "4-7"
	}

	var dbg mapDebug
	var x1, x2, y1, y2 float64
	if left {
		x1, x2, y1, y2 = a0.obsPH, a1.obsPH, a0.truePH, a1.truePH
		dbg = linearMapDbg(raw, x1, x2, y1, y2)
	} else {
		x1, x2, y1, y2 = a1.obsPH, a2.obsPH, a1.truePH, a2.truePH
		dbg = linearMapDbg(raw, x1, x2, y1, y2)
	}

	out, clamped := clampPH(dbg.y, 0, 14)

	if d.debug {
		log.Printf("robotank_ph cal: MODE=3pt piecewise (segment=%s chosen by raw<=obs@7? raw=%.6f obs7=%.6f => %v)",
			seg, raw, a1.obsPH, left)

		if left {
			log.Printf("robotank_ph cal:   segment anchors: true4=%.2f obs4=%.6f  true7=%.2f obs7=%.6f",
				a0.truePH, a0.obsPH, a1.truePH, a1.obsPH)
		} else {
			log.Printf("robotank_ph cal:   segment anchors: true7=%.2f obs7=%.6f  true10=%.2f obs10=%.6f",
				a1.truePH, a1.obsPH, a2.truePH, a2.obsPH)
		}

		log.Printf("robotank_ph cal:   den=%.9f t=%.9f y=%.6f%s",
			dbg.den, dbg.t, dbg.y, boolSuffix(clamped, " (clamped 0..14)"))

		den := (x2 - x1)
		scale := 0.0
		if math.Abs(den) >= 1e-9 {
			scale = (y2 - y1) / den
		}
		offset := y1 - scale*x1

		log.Printf("robotank_ph cal:   line form y=scale*x+offset => scale=%.9f offset=%.9f", scale, offset)
		log.Printf("robotank_ph cal: RESULT cal=%.6f", out)
	}

	return out
}

// Debug helper only: implied mV under the designer's convention.
func phToImpliedMv(ph float64) float64 {
	return (7.0 - ph) * phSlopeMvPerPH
}

// ---- I2C helpers ----

// allFF returns true if every byte is 0xFF.
func allFF(b []byte) bool {
	if len(b) == 0 {
		return false
	}
	for _, v := range b {
		if v != 0xFF {
			return false
		}
	}
	return true
}

func (d *Driver) command(cmd string) error {
	if d.debug {
		log.Printf("robotank_ph addr=0x%02X write cmd=%q", d.addr, cmd)
	}
	if err := d.bus.WriteBytes(d.addr, []byte(cmd+"\x00")); err != nil {
		return err
	}
	time.Sleep(d.delay)
	return nil
}

func (d *Driver) readASCII() (string, error) {
	payload, err := d.bus.ReadBytes(d.addr, 32)
	if err != nil {
		return "", err
	}

	if d.debug {
		log.Printf("robotank_ph addr=0x%02X read payload=% X", d.addr, payload)
	}

	if len(payload) == 0 {
		return "", fmt.Errorf("empty payload")
	}

	// Some devices/bus errors manifest as all 0xFF. Retry once.
	if payload[0] == 0xFF && allFF(payload) {
		time.Sleep(50 * time.Millisecond)
		payload, err = d.bus.ReadBytes(d.addr, 32)
		if err != nil {
			return "", err
		}
		if d.debug {
			log.Printf("robotank_ph addr=0x%02X read retry payload=% X", d.addr, payload)
		}
		if len(payload) == 0 {
			return "", fmt.Errorf("empty payload (after retry)")
		}
	}

	if payload[0] != 1 {
		return "", fmt.Errorf("status=%d payload=%v", payload[0], payload)
	}

	b := payload[1:]

	// cut at first NUL
	for i, v := range b {
		if v == 0x00 {
			b = b[:i]
			break
		}
	}

	// trim trailing 0xFF
	for len(b) > 0 && b[len(b)-1] == 0xFF {
		b = b[:len(b)-1]
	}

	s := strings.TrimSpace(string(b))
	if d.debug {
		log.Printf("robotank_ph addr=0x%02X read ascii=%q", d.addr, s)
	}

	return s, nil
}

func (d *Driver) readFloat(cmd string) (float64, error) {
	// Critical: serialize the *whole* "write -> wait -> read" transaction
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.command(cmd); err != nil {
		return 0, err
	}
	resp, err := d.readASCII()
	if err != nil {
		return 0, err
	}
	v, err := strconv.ParseFloat(resp, 64)
	if err != nil {
		return 0, fmt.Errorf("parse float cmd=%q resp=%q: %w", cmd, resp, err)
	}
	return v, nil
}

// Optional: useful for startup debug logging
func (d *Driver) Firmware() (string, error) {
	// Firmware call also needs to be serialized
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.command("H"); err != nil {
		return "", err
	}
	return d.readASCII()
}
