// driver.go
package robotank_conductivity

import (
	"fmt"
	"log"
	"math"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	driverName = "Robo-Tank Conductivity Circuit"

	// Fixed constants for THIS driver (not user-configurable)
	fixedRefUS     = 53000.0
	fixedAlphaPerC = 0.019
)

// RoboTankConductivity exposes 2 analog channels:
// 0 = conductivity (uS/cm) at RefTempC (if temp compensation enabled)
// 1 = salinity (ppt) derived from channel 0
type RoboTankConductivity struct {
	addr  byte
	bus   i2c.Bus
	delay time.Duration
	meta  hal.Metadata

	// Serialize *all* I2C command/response sequences.
	// This prevents interleaving between:
	// - channel 0 reads and channel 1 reads
	// - snapshot() and read()
	// - multiple API calls arriving concurrently
	mu sync.Mutex

	// Calibration + conversion settings (loaded from factory parameters)
	absDFresh float64 // abs(U-V) in fresh/RODI (maps -> 0 uS)
	absDStd   float64 // abs(U-V) in conductivity standard (maps -> RefUS)

	// Fixed conversion constants (not configurable)
	refUS     float64 // fixed at 53000 uS
	alphaPerC float64 // fixed at 0.019

	// temperature compensation settings
	refTempC float64 // reference temperature for compensation (typically 25C)

	// temperature (injected by reef-pi temp subsystem)
	tempC         float64
	tempUpdatedAt time.Time
	doTempComp    bool

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
// This allows the Chemistry subsystem to type-assert the pin and inject live temperature.
func (p *rtPin) SetTemperatureC(tempC float64) {
	p.parent.SetTemperatureC(tempC)
}

// ---------------- I2C helpers ----------------

// drain tries to clear any stale buffered response on the device.
// Many "EZO-like" firmwares will keep previous output until overwritten.
// We intentionally ignore errors.
func (d *RoboTankConductivity) drain() {
	_, _ = d.bus.ReadBytes(d.addr, 32)
}

func (d *RoboTankConductivity) command(cmd string) error {
	// Clear any previous response before we send a new command.
	// This helps prevent "stale response reuse".
	d.drain()

	// NUL terminator helps EZO-style parsing; harmless for most Arduino firmwares
	if err := d.bus.WriteBytes(d.addr, []byte(cmd+"\x00")); err != nil {
		return err
	}
	time.Sleep(d.delay)
	return nil
}

// read returns EZO-style response:
// payload[0]=status (1=OK), payload[1:]=ASCII text (NUL padded)
// Some devices also include trailing 0xFF filler; strip it.
//
// the debug logs will show you the raw payload in hex so we can adapt parsing.
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

	// Status byte expectation (EZO-like)
	if payload[0] != 1 {
		return "", fmt.Errorf("device status=%d payload=%v", payload[0], payload)
	}

	b := payload[1:]

	// cut at first NUL
	for i, v := range b {
		if v == 0x00 {
			b = b[:i]
			break
		}
	}

	// trim trailing 0xFF fillers
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
func parseFirstFloat(resp string) (float64, error) {
	// Split on common separators; keep numeric tokens
	fields := strings.FieldsFunc(resp, func(r rune) bool {
		switch r {
		case ',', ' ', '\t', '\r', '\n', '=':
			return true
		default:
			return false
		}
	})
	for _, f := range fields {
		f = strings.TrimSpace(f)
		if f == "" {
			continue
		}
		// tolerate comma decimal
		f = strings.ReplaceAll(f, ",", ".")
		if v, err := strconv.ParseFloat(f, 64); err == nil {
			return v, nil
		}
	}
	return 0, fmt.Errorf("no float found in resp=%q", resp)
}

// readFloat sends a command then reads/parses a float response.
// Includes retry logic for timing jitter or transient stale reads.
func (d *RoboTankConductivity) readFloat(cmd string) (float64, error) {
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
	old := d.tempC
	d.tempC = tempC
	d.tempUpdatedAt = time.Now()

	if d.debug {
		log.Printf("robotank_cond addr=%d SetTemperatureC: %.2fC -> %.2fC (doTempComp=%v refTempC=%.2f alpha=%.4f(fixed))",
			d.addr, old, d.tempC, d.doTempComp, d.refTempC, d.alphaPerC)
	}
}

// ---------------- Math / conversion ----------------

// absDiff reads U and V and returns |U-V|.
//
// CRITICAL: this locks across BOTH command/reads so that multiple callers
// cannot interleave the protocol and accidentally swap or duplicate responses.
func (d *RoboTankConductivity) absDiff() (ad, u, v float64, err error) {
	d.mu.Lock()
	defer d.mu.Unlock()

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

// usFromAbsD maps abs(U-V) to conductivity uS/cm using 2-point linear calibration:
// absDFresh -> 0 uS
// absDStd   -> refUS (fixed 53000)
func (d *RoboTankConductivity) usFromAbsD(ad float64) (float64, error) {
	if d.absDFresh <= 0 || d.absDStd <= 0 {
		return 0, fmt.Errorf("%s: missing calibration (AbsD_Fresh and AbsD_Std must be set)", driverName)
	}
	if d.absDFresh == d.absDStd {
		return 0, fmt.Errorf("%s: invalid calibration (AbsD_Fresh == AbsD_Std)", driverName)
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
	if !d.doTempComp {
		if d.debug {
			log.Printf("robotank_cond addr=%d tempComp: disabled (returning us_meas=%.2f)", d.addr, us)
		}
		return us
	}

	// Stale / missing temperature detection
	if d.tempUpdatedAt.IsZero() {
		log.Printf("robotank_cond addr=%d WARNING: temp compensation enabled but temperature has never been set (tempC=%.2f). Using current tempC anyway.",
			d.addr, d.tempC)
	} else {
		age := time.Since(d.tempUpdatedAt)
		if age > 2*time.Minute {
			log.Printf("robotank_cond addr=%d WARNING: temp is stale (age=%v, tempC=%.2f). Check temp_sensor_id / temperature subsystem updates.",
				d.addr, age, d.tempC)
		} else if d.debug {
			log.Printf("robotank_cond addr=%d temp age=%v (tempC=%.2f)", d.addr, age, d.tempC)
		}
	}

	den := 1.0 + d.alphaPerC*(d.tempC-d.refTempC)
	if den <= 0.1 {
		if d.debug {
			log.Printf("robotank_cond addr=%d tempComp: den clamped (den=%.5f)", d.addr, den)
		}
		den = 0.1
	}

	usRef := us / den

	if d.debug {
		log.Printf("robotank_cond addr=%d tempComp: us_meas=%.2f at %.2fC -> us_ref=%.2f at %.2fC (den=%.5f alpha=%.4f(fixed))",
			d.addr, us, d.tempC, usRef, d.refTempC, den, d.alphaPerC)
	}
	return usRef
}

// pptFromUS converts uS@refTempC to ppt using a reef convention:
// 53000 uS/cm -> 35.0 ppt
// ppt = usRef * (35 / 53000)
func (d *RoboTankConductivity) pptFromUS(usRef float64) float64 {
	return usRef * (35.0 / d.refUS)
}

// compute returns uS@refTempC (and debug raw values)
func (d *RoboTankConductivity) compute() (usRef, u, v, ad float64, err error) {
	ad, u, v, err = d.absDiff()
	if err != nil {
		return 0, 0, 0, 0, err
	}

	if d.debug {
		log.Printf("robotank_cond addr=%d raw U=%.3f V=%.3f |d|=%.3f (AbsD_Fresh=%.6f AbsD_Std=%.6f RefUS=%.1f(fixed))",
			d.addr, u, v, ad, d.absDFresh, d.absDStd, d.refUS)
	}

	us, err := d.usFromAbsD(ad)
	if err != nil {
		return 0, u, v, ad, err
	}

	usRef = d.tempCompToRef(us)
	return usRef, u, v, ad, nil
}

// ---------------- rtPin: hal.AnalogInputPin ----------------

// Value returns channel-specific reading
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
		log.Printf("robotank_cond addr=%d ch=%d U=%.3f V=%.3f |d|=%.3f temp=%.2fC us@%.1fC=%.1f ppt=%.3f",
			p.parent.addr, p.ch, u, v, ad, p.parent.tempC, p.parent.refTempC, usRef, ppt)
	}

	if p.ch == 0 {
		return usRef, nil
	}
	return ppt, nil
}

func (p *rtPin) Measure() (float64, error) { return p.Value() }

// Calibrate supports (on channel 0 only):
// Expected=0     -> fresh point (AbsD_Fresh)
// Expected>0     -> standard point at fixed standard (AbsD_Std)
//
// NOTE: RefUS is fixed at 53000, so Expected is still used by the caller UI,
// but the driver treats this point as “the standard solution point”.
func (p *rtPin) Calibrate(ms []hal.Measurement) error {
	if p.ch != 0 {
		return fmt.Errorf("%s: calibrate using channel 0 (uS/cm)", driverName)
	}

	for _, m := range ms {
		exp := m.Expected
		obs := m.Observed

		// If Observed is zero, fallback to fresh absDiff for backward compatibility
		if obs == 0 {
			ad, _, _, err := p.parent.absDiff()
			if err != nil {
				return err
			}
			obs = ad
		}

		switch {
		case exp == 0:
			p.parent.absDFresh = obs
			log.Printf("robotank_cond calibrated FRESH absD=%.6f", obs)

		case exp > 0:
			p.parent.absDStd = obs
			log.Printf("robotank_cond calibrated STD (fixed RefUS=%.1f) exp=%.1f absD=%.6f",
				p.parent.refUS, exp, obs)

		default:
			return fmt.Errorf("%s: unsupported calibration Expected=%.3f (use 0 for fresh, >0 for standard uS)", driverName, exp)
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

	// Secondary ordering:
	secondary := func() []string {
		if p.ch == 0 {
			return []string{"ppt", "tempC", "U", "V"}
		}
		return []string{"us_ref", "tempC", "U", "V"}
	}()

	// Driver-defined role labels (UI uses these headings)
	roles := map[string]any{
		"primary": func() string {
			if p.ch == 0 {
				return "Primary (Conductivity)"
			}
			return "Primary (Salinity)"
		}(),
		"observed": "Observed (raw |U−V|)",
	}

	// Driver-defined human labels for each signal key
	names := map[string]any{
		"value": func() string {
			if p.ch == 0 {
				return "Conductivity (uS/cm @ ref temp)"
			}
			return "Salinity (ppt)"
		}(),
		"abs_d":  "|U−V| (mV)",
		"U":      "U (mV)",
		"V":      "V (mV)",
		"tempC":  "Temperature (°C)",
		"us_ref": "Conductivity (uS/cm @ ref temp)",
		"ppt":    "Salinity (ppt)",
	}

	help := map[string]any{
		"abs_d":  "Raw differential used for calibration/conversion (absolute difference of U and V).",
		"us_ref": "Conductivity, temperature-compensated to RefTempC when enabled.",
		"ppt":    "Salinity derived from conductivity using 35 ppt @ RefUS (fixed).",
		"tempC":  "Last temperature injected by reef-pi temperature subsystem.",
	}

	// MINIMAL meta: only what the UI needs
	meta := map[string]any{
		"channel":    p.ch,
		"doTempComp": p.parent.doTempComp,

		// legacy (keep for older clients)
		"calibration_observed_key": "abs_d",

		// explicit keys for generic UI
		"raw_signal_key":     "abs_d",
		"primary_signal_key": "value",

		// derived ordering (UI must not sort)
		"secondary_signal_keys": secondary,

		// decimals (UI must not infer)
		"signal_decimals": map[string]any{
			"value":  3,
			"abs_d":  3,
			"U":      3,
			"V":      3,
			"tempC":  2,
			"us_ref": 1,
			"ppt":    3,
		},

		// driver-defined UI strings (UI must not hardcode)
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

// AnalogInputPin returns requested channel pin
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
