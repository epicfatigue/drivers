// driver.go
package robotank_conductivity

import (
	"fmt"
	"log"
	"math"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	driverName = "Robo-Tank Conductivity Circuit"

	// Fixed constants for THIS driver
	fixedRefUS    = 53000.0
	fixedRefTempC = 25.0

	// Default (user can override via factory parameter AlphaPerC)
	fixedAlphaPerC = 0.0015

	// If we haven't received a temp update in this long, stop using it
	tempStaleAfter = 2 * time.Minute
	
)

// firstNumRe finds the first number-like token in a response string.
// Handles things like: "U,14.322", "14.322,OK", "U=14,322", "OK 14.322"
var firstNumRe = regexp.MustCompile(`[-+]?\d+(?:[.,]\d+)?`)

// RoboTankConductivity exposes 2 analog channels:
// 0 = conductivity (uS/cm) compensated to 25C when temperature is available
// 1 = salinity (ppt) derived from channel 0
//
// Calibration assumptions:
// - Reference temperature is fixed at 25°C
// - Standard solution is fixed at 53,000 µS/cm
type RoboTankConductivity struct {
	addr  byte
	bus   i2c.Bus
	delay time.Duration
	meta  hal.Metadata

	// Serialize *all* I2C command/response sequences and guard shared state.
	mu sync.Mutex

	// Calibration + conversion settings (loaded from factory parameters)
	absDFresh float64 // AbsD in RODI (maps -> 0 uS)
	absDStd   float64 // AbsD in 53,000 uS standard (maps -> RefUS)

	// Conversion constants
	refUS     float64 // fixed at 53000 uS
	alphaPerC float64 // configurable (AlphaPerC)

	// Fixed reference temperature for compensation
	refTempC float64 // fixed at 25C

	// temperature (injected by reef-pi temp subsystem)
	// If temp is -1, it is ignored and treated as 25C.
	tempC         float64
	tempUpdatedAt time.Time
	tempValid     bool

	debug bool

	// two pins (channels 0 and 1)
	pins []*rtPin
}

// rtPin is a lightweight wrapper that exposes channel 0/1
type rtPin struct {
	parent *RoboTankConductivity
	ch     int // 0=uS/cm, 1=ppt
}

// Implement TemperatureSetter on the pin, forwarding to the parent driver.
func (p *rtPin) SetTemperatureC(tempC float64) { p.parent.SetTemperatureC(tempC) }

// ---------------- I2C helpers ----------------

func (d *RoboTankConductivity) drain() {
	_, _ = d.bus.ReadBytes(d.addr, 32)
}

func (d *RoboTankConductivity) command(cmd string) error {
	d.drain()
	if err := d.bus.WriteBytes(d.addr, []byte(cmd+"\x00")); err != nil {
		return err
	}
	time.Sleep(d.delay)
	return nil
}

func (d *RoboTankConductivity) read() (string, error) {
	payload, err := d.bus.ReadBytes(d.addr, 32)
	if err != nil {
		return "", err
	}
	if len(payload) == 0 {
		return "", fmt.Errorf("empty i2c payload")
	}

	if d.debug {
		log.Printf("robotank_cond addr=%d raw payload: % X", d.addr, payload)
	}

	if payload[0] != 1 {
		return "", fmt.Errorf("device status=%d payload=%v", payload[0], payload)
	}

	b := payload[1:]

	for i, v := range b {
		if v == 0x00 {
			b = b[:i]
			break
		}
	}

	for len(b) > 0 && b[len(b)-1] == 0xFF {
		b = b[:len(b)-1]
	}

	return strings.TrimSpace(string(b)), nil
}

// parseFirstFloat extracts the first parseable float out of a response string.
// This tolerates formats like:
// "14.322"
// "U=14.322"
// "14.322,OK"
// "U,14.322"
// "U=14,322"
func parseFirstFloat(resp string) (float64, error) {
	m := firstNumRe.FindString(resp)
	if m == "" {
		return 0, fmt.Errorf("no float found in resp=%q", resp)
	}
	m = strings.ReplaceAll(m, ",", ".")
	v, err := strconv.ParseFloat(m, 64)
	if err != nil {
		return 0, fmt.Errorf("bad float %q in resp=%q: %w", m, resp, err)
	}
	return v, nil
}

func (d *RoboTankConductivity) readFloat(cmd string) (float64, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.command(cmd); err != nil {
		return 0, err
	}

	var lastErr error
	for i := 0; i < 6; i++ {
		resp, err := d.read()
		if err != nil {
			lastErr = err
			time.Sleep(50 * time.Millisecond)
			continue
		}

		if d.debug {
			log.Printf("robotank_cond addr=%d cmd=%q resp=%q", d.addr, cmd, resp)
		}

		v, err := parseFirstFloat(resp)
		if err == nil {
			return v, nil
		}

		lastErr = err
		time.Sleep(50 * time.Millisecond)
	}

	return 0, fmt.Errorf("cmd=%q: %v", cmd, lastErr)
}

// ---------------- Board API ----------------

func (d *RoboTankConductivity) TestHigh() (float64, error) { return d.readFloat("U") }
func (d *RoboTankConductivity) TestLow() (float64, error)  { return d.readFloat("V") }

func (d *RoboTankConductivity) Firmware() (string, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.command("H"); err != nil {
		return "", err
	}
	return d.read()
}

func (d *RoboTankConductivity) SetWaterType(wt int) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	return d.command(fmt.Sprintf("W,%d", wt))
}

// ---------------- Temperature hook ----------------

func (d *RoboTankConductivity) SetTemperatureC(tempC float64) {
	d.mu.Lock()
	defer d.mu.Unlock()

	d.tempUpdatedAt = time.Now()

	// Sentinel: -1 means "unknown", assume ref temp (25C) and don't compensate.
	if tempC < 0 {
		d.tempValid = false
		d.tempC = d.refTempC
		if d.debug {
			log.Printf("robotank_cond addr=%d SetTemperatureC: invalid/sentinel %.2f -> assuming %.2fC (no temp comp)",
				d.addr, tempC, d.refTempC)
		}
		return
	}

	old := d.tempC
	d.tempC = tempC
	d.tempValid = true

	if d.debug {
		log.Printf("robotank_cond addr=%d SetTemperatureC: %.2fC -> %.2fC (refTempC=%.2f alpha=%.6f)",
			d.addr, old, d.tempC, d.refTempC, d.alphaPerC)
	}
}

// ---------------- Math / conversion ----------------

func (d *RoboTankConductivity) absDiff() (ad, u, v float64, err error) {
	u, err = d.TestHigh()
	if err != nil {
		return 0, 0, 0, err
	}
	v, err = d.TestLow()
	if err != nil {
		return 0, 0, 0, err
	}
	ad = math.Abs(u - v)
	return ad, u, v, nil
}

func (d *RoboTankConductivity) usFromAbsD(ad float64) (float64, error) {
	if d.absDFresh <= 0 || d.absDStd <= 0 {
		return 0, fmt.Errorf("%s: missing calibration (AbsD_RODI and AbsD_Std must be set)", driverName)
	}
	if d.absDFresh == d.absDStd {
		return 0, fmt.Errorf("%s: invalid calibration (AbsD_RODI == AbsD_Std)", driverName)
	}

	// From your observations: absD is BIG in fresh and SMALL in salt.
	x := (d.absDFresh - ad) / (d.absDFresh - d.absDStd)

	// Clamp: allow slight overshoot but prevent spikes
	if x < 0 {
		x = 0
	}
	if x > 1.2 {
		x = 1.2
	}

	return x * d.refUS, nil
}

// Convert measured uS at current temp to uS at refTempC using linear coefficient
// uS_ref = uS_meas / (1 + alpha*(tempC-refTempC))
func (d *RoboTankConductivity) tempCompToRef(us float64) float64 {
	// Copy temp state under lock to avoid races, and to make the logic deterministic.
	d.mu.Lock()
	tempValid := d.tempValid
	tempC := d.tempC
	updatedAt := d.tempUpdatedAt
	refTempC := d.refTempC
	alpha := d.alphaPerC
	debug := d.debug
	addr := d.addr
	d.mu.Unlock()

	if !tempValid {
		if debug {
			log.Printf("robotank_cond addr=%d tempComp: no valid temp -> assume %.2fC (returning us_meas=%.2f)",
				addr, refTempC, us)
		}
		return us
	}

	// If chemistry stops injecting temp (e.g. temp_sensor_id=-1) but we never got a sentinel,
	// refuse to keep using an old buffered value forever.
	if updatedAt.IsZero() {
		log.Printf("robotank_cond addr=%d WARNING: tempValid=true but tempUpdatedAt=zero -> disabling temp comp, assume %.2fC",
			addr, refTempC)
		// also update driver state so Snapshot/UI reflects reality
		d.mu.Lock()
		d.tempValid = false
		d.tempC = d.refTempC
		d.mu.Unlock()
		return us
	}

	age := time.Since(updatedAt)
	if age > tempStaleAfter {
		log.Printf("robotank_cond addr=%d WARNING: temp stale (age=%v, tempC=%.2f) -> disabling temp comp, assume %.2fC",
			addr, age, tempC, refTempC)
		d.mu.Lock()
		d.tempValid = false
		d.tempC = d.refTempC
		d.mu.Unlock()
		return us
	} else if debug {
		log.Printf("robotank_cond addr=%d temp age=%v (tempC=%.2f)", addr, age, tempC)
	}

	den := 1.0 + alpha*(tempC-refTempC)
	if den <= 0.1 {
		if debug {
			log.Printf("robotank_cond addr=%d tempComp: den clamped (den=%.5f)", addr, den)
		}
		den = 0.1
	}

	usRef := us / den

	if debug {
		log.Printf("robotank_cond addr=%d tempComp: us_meas=%.2f at %.2fC -> us_ref=%.2f at %.2fC (den=%.5f alpha=%.6f)",
			addr, us, tempC, usRef, refTempC, den, alpha)
	}
	return usRef
}

// pptFromUS converts uS@refTempC to ppt using a reef convention:
// 53000 uS/cm -> 35.0 ppt
// ppt = usRef * (35 / 53000)
func (d *RoboTankConductivity) pptFromUS(usRef float64) float64 {
	return usRef * (35.0 / d.refUS)
}

func (d *RoboTankConductivity) compute() (usRef, u, v, ad float64, err error) {
	ad, u, v, err = d.absDiff()
	if err != nil {
		return 0, 0, 0, 0, err
	}

	// Read shared state for logging under lock (avoid races)
	d.mu.Lock()
	absFresh := d.absDFresh
	absStd := d.absDStd
	refUS := d.refUS
	refTempC := d.refTempC
	tempValid := d.tempValid
	tempC := d.tempC
	debug := d.debug
	addr := d.addr
	alpha := d.alphaPerC
	d.mu.Unlock()

	if debug {
		log.Printf("robotank_cond addr=%d raw U=%.3f V=%.3f |d|=%.3f (AbsD_RODI=%.6f AbsD_Std=%.6f RefUS=%.1f(fixed) RefTempC=%.2f(fixed) TempValid=%v TempC=%.2f)",
			addr, u, v, ad, absFresh, absStd, refUS, refTempC, tempValid, tempC)
	}

	us, err := d.usFromAbsD(ad)
	if err != nil {
		return 0, u, v, ad, err
	}

	usRef = d.tempCompToRef(us)

	// log pre/post temp compensation so you can scrape/correlate from logs
	if debug {
		// Expected compensation factor when applied (note: tempCompToRef can disable comp if temp is stale)
		den := 1.0
		if tempValid {
			den = 1.0 + alpha*(tempC-refTempC)
		}

		ppt := d.pptFromUS(usRef)
		compApplied := math.Abs(usRef-us) > 0.0001

		log.Printf("robotank_cond addr=%d us_meas=%.1f den=%.5f us_ref=%.1f ppt=%.3f compApplied=%v (tempC=%.2f valid=%v refTempC=%.2f alpha=%.6f)",
			addr, us, den, usRef, ppt, compApplied, tempC, tempValid, refTempC, alpha)
	}

	return usRef, u, v, ad, nil
}

// ---------------- rtPin: hal.AnalogInputPin ----------------

func (p *rtPin) Value() (float64, error) {
	usRef, u, v, ad, err := p.parent.compute()
	if err != nil {
		if p.parent.debug {
			log.Printf("robotank_cond addr=%d ch=%d compute error: %v", p.parent.addr, p.ch, err)
		}
		return 0, err
	}

	ppt := p.parent.pptFromUS(usRef)

	if p.parent.debug {
		log.Printf("robotank_cond addr=%d ch=%d U=%.3f V=%.3f |d|=%.3f temp=%.2fC(valid=%v) us@%.1fC=%.1f ppt=%.3f",
			p.parent.addr, p.ch, u, v, ad, p.parent.tempC, p.parent.tempValid, p.parent.refTempC, usRef, ppt)
	}

	if p.ch == 0 {
		return usRef, nil
	}
	return ppt, nil
}

func (p *rtPin) Measure() (float64, error) { return p.Value() }

// Calibrate supports (on channel 0 only):
// Expected=0      -> RODI point (AbsD_RODI)
// Expected>0      -> standard point (AbsD_Std) using 53,000 µS/cm solution @ 25°C
func (p *rtPin) Calibrate(ms []hal.Measurement) error {
	if p.ch != 0 {
		return fmt.Errorf("%s: calibrate using channel 0 (uS/cm)", driverName)
	}

	for _, m := range ms {
		exp := m.Expected
		obs := m.Observed

		// If Observed is zero, fallback to live absDiff.
		if obs == 0 {
			ad, _, _, err := p.parent.absDiff()
			if err != nil {
				return err
			}
			obs = ad
		}

		switch {
		case exp == 0:
			p.parent.mu.Lock()
			p.parent.absDFresh = obs
			p.parent.mu.Unlock()
			log.Printf("robotank_cond calibrated RODI absD=%.6f (assume %.1fC)", obs, p.parent.refTempC)

		case exp > 0:
			p.parent.mu.Lock()
			p.parent.absDStd = obs
			p.parent.mu.Unlock()
			log.Printf("robotank_cond calibrated STD absD=%.6f (assume %.1fC, std=%.0f uS/cm)",
				obs, p.parent.refTempC, p.parent.refUS)

		default:
			return fmt.Errorf("%s: unsupported calibration Expected=%.3f (use 0 for RODI, >0 for standard)", driverName, exp)
		}
	}

	return nil
}

func (p *rtPin) Name() string {
	if p.ch == 0 {
		return driverName + " (uS/cm)"
	}
	return driverName + " (ppt)"
}

func (p *rtPin) Number() int { return p.ch }

// REQUIRED by your fork (hal.Pin / hal.AnalogInputPin)
func (p *rtPin) Close() error { return nil }

// Safe to include; some forks require Metadata on pins
func (p *rtPin) Metadata() hal.Metadata { return p.parent.meta }

// Snapshot Function
func (p *rtPin) Snapshot() (hal.Snapshot, error) {
	usRef, u, v, ad, err := p.parent.compute()
	if err != nil {
		return hal.Snapshot{}, err
	}
	ppt := p.parent.pptFromUS(usRef)

	var primary float64
	var unit string
	if p.ch == 0 {
		primary = usRef
		unit = "uS/cm"
	} else {
		primary = ppt
		unit = "ppt"
	}

	secondary := func() []string {
		if p.ch == 0 {
			return []string{"ppt", "tempC", "U", "V"}
		}
		return []string{"us_ref", "tempC", "U", "V"}
	}()

	roles := map[string]any{
		"primary": func() string {
			if p.ch == 0 {
				return "Primary (Conductivity)"
			}
			return "Primary (Salinity)"
		}(),
		"observed": "Observed (raw |U−V|)",
	}

	names := map[string]any{
		"value": func() string {
			if p.ch == 0 {
				return "Conductivity (uS/cm @ 25°C)"
			}
			return "Salinity (ppt)"
		}(),
		"abs_d":  "|U−V| (mV)",
		"U":      "U (mV)",
		"V":      "V (mV)",
		"tempC":  "Temperature (°C)",
		"us_ref": "Conductivity (uS/cm @ 25°C)",
		"ppt":    "Salinity (ppt)",
	}

	help := map[string]any{
		"abs_d":  "Raw differential used for calibration/conversion (absolute difference of U and V).",
		"us_ref": "Conductivity compensated to 25°C when a valid temperature is available. If temp updates stop for >2 minutes, compensation is disabled.",
		"ppt":    "Salinity derived from conductivity using 35 ppt @ 53,000 µS/cm.",
		"tempC":  "Last injected water temperature. If unknown or stale, driver assumes 25°C and disables compensation.",
	}

	meta := map[string]any{
		"channel": p.ch,

		"raw_signal_key":       "abs_d",
		"primary_signal_key":   "value",
		"secondary_signal_keys": secondary,

		"temp_valid": p.parent.tempValid,

		"ui_note": fmt.Sprintf(
			"Assumes %.2f°C reference temperature. Standard calibration solution is %.0f µS/cm. Temp compensation uses AlphaPerC=%.6f and is applied only when temp is available and recent.",
			p.parent.refTempC, p.parent.refUS, p.parent.alphaPerC,
		),

		"signal_decimals": map[string]any{
			"value":  3,
			"abs_d":  3,
			"U":      3,
			"V":      3,
			"tempC":  2,
			"us_ref": 1,
			"ppt":    3,
		},

		"display_roles": roles,
		"display_names": names,
		"display_help":  help,
	}

	s := hal.Snapshot{
		Value: primary,
		Unit:  unit,
		Signals: map[string]hal.Signal{
			"U":      {Now: u, Unit: "mV"},
			"V":      {Now: v, Unit: "mV"},
			"abs_d":  {Now: ad, Unit: "mV"},
			"us_ref": {Now: usRef, Unit: "uS/cm"},
			"ppt":    {Now: ppt, Unit: "ppt"},
			"tempC":  {Now: p.parent.tempC, Unit: "C"},
		},
		Meta: meta,
	}

	return s, nil
}

// ---------------- hal.Driver / plumbing ----------------

func (d *RoboTankConductivity) Name() string           { return driverName }
func (d *RoboTankConductivity) Close() error           { return nil }
func (d *RoboTankConductivity) Metadata() hal.Metadata { return d.meta }

func (d *RoboTankConductivity) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if n < 0 || n > 1 {
		return nil, fmt.Errorf("%s supports channels 0(uS/cm) and 1(ppt). Asked:%d", driverName, n)
	}
	return d.pins[n], nil
}

func (d *RoboTankConductivity) AnalogInputPins() []hal.AnalogInputPin {
	return []hal.AnalogInputPin{d.pins[0], d.pins[1]}
}

func (d *RoboTankConductivity) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		return []hal.Pin{d.pins[0], d.pins[1]}, nil
	default:
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
}
