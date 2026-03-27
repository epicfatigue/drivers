// driver.go
package ph_board

import (
	"encoding/binary"
	"errors"
	"fmt"
	"log"
	"math"
	"sort"
	"strings"
	"sync"
	"time"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

const (
	driverName = "phDriver I2C pH (ADC→mV→pH)"

	idealSlope25C = 59.16
	refTempK25C   = 298.15
	fixedVrefV    = 2.048

	minI2CGap       = 35 * time.Millisecond
	settleAfterRead = 2 * time.Millisecond
	cacheMaxAge     = 250 * time.Millisecond
	retryDelay      = 20 * time.Millisecond

	cmdReset = 0x06
	cmdStart = 0x08
	cmdRData = 0x10
	cmdWREG  = 0x40

	configByte = 0x06
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

type phDriver struct {
	addr byte
	bus  i2c.Bus
	meta hal.Metadata

	vrefV float64

	obs7mV  float64
	obs4mV  float64
	obs10mV float64

	slopeOverride float64

	doTempComp    bool
	refTempC      float64
	tempC         float64
	tempUpdatedAt time.Time

	debug bool
	pins  []*phPin

	mu sync.Mutex

	lastXferAt   time.Time
	lastSampleAt time.Time
	lastMV       float64
	lastRaw      []byte
	lastCode     int32
}

type phPin struct {
	parent *phDriver
	ch     int
}

type anchor struct {
	truePH float64
	obsMV  float64
	label  string
}

func (p *phPin) SetTemperatureC(tempC float64) { p.parent.SetTemperatureC(tempC) }

func (d *phDriver) SetTemperatureC(tempC float64) {
	old := d.tempC
	d.tempC = tempC
	d.tempUpdatedAt = time.Now()
	if d.debug {
		log.Printf("pHboard_driver addr=0x%02X SetTemperatureC: %.2fC -> %.2fC (doTempComp=%v refTempC=%.2f)",
			d.addr, old, d.tempC, d.doTempComp, d.refTempC)
	}
}

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

func (d *phDriver) enforceMinGap(minGap time.Duration) {
	if d.lastXferAt.IsZero() {
		return
	}
	elapsed := time.Since(d.lastXferAt)
	if elapsed < minGap {
		time.Sleep(minGap - elapsed)
	}
}

func (d *phDriver) writeCmd(payload ...byte) error {
	d.lastXferAt = time.Now()
	return d.bus.WriteBytes(d.addr, payload)
}

func (d *phDriver) initADC() error {
	lock := lockForAddr(d.addr)
	lock.Lock()
	defer lock.Unlock()

	d.mu.Lock()
	defer d.mu.Unlock()

	if d.debug {
		log.Printf("pHboard_driver addr=0x%02X init: reset, config=0x%02X, start continuous conversion",
			d.addr, configByte)
	}

	if err := d.writeCmd(cmdReset); err != nil {
		return fmt.Errorf("ads1119 reset failed: %w", err)
	}
	time.Sleep(5 * time.Millisecond)

	if err := d.writeCmd(cmdWREG, configByte); err != nil {
		return fmt.Errorf("ads1119 config write failed: %w", err)
	}
	time.Sleep(5 * time.Millisecond)

	if err := d.writeCmd(cmdStart); err != nil {
		return fmt.Errorf("ads1119 start conversion failed: %w", err)
	}
	time.Sleep(15 * time.Millisecond)

	return nil
}

func (d *phDriver) readObservedMV() (mv float64, raw []byte, adcCode int32, err error) {
	lock := lockForAddr(d.addr)
	lock.Lock()
	defer lock.Unlock()

	d.mu.Lock()
	defer d.mu.Unlock()

	if !d.lastSampleAt.IsZero() && time.Since(d.lastSampleAt) < cacheMaxAge {
		if d.debug {
			log.Printf("pHboard_driver addr=0x%02X cache hit age=%v mv=%.2f",
				d.addr, time.Since(d.lastSampleAt), d.lastMV)
		}
		return d.lastMV, append([]byte(nil), d.lastRaw...), d.lastCode, nil
	}

	d.enforceMinGap(minI2CGap)

	var lastErr error
	for attempt := 1; attempt <= 2; attempt++ {
		payload := make([]byte, 2)
		e := d.bus.ReadFromReg(d.addr, cmdRData, payload)
		d.lastXferAt = time.Now()

		if e != nil {
			lastErr = e
			if d.debug {
				log.Printf("pHboard_driver addr=0x%02X read attempt=%d error=%v", d.addr, attempt, e)
			}
			if attempt == 1 && isTransientI2C(e) {
				time.Sleep(retryDelay)
				continue
			}
			return 0, nil, 0, e
		}

		if len(payload) != 2 {
			lastErr = fmt.Errorf("short i2c read: got %d bytes, want 2", len(payload))
			if d.debug {
				log.Printf("pHboard_driver addr=0x%02X read attempt=%d error=%v payload=% X", d.addr, attempt, lastErr, payload)
			}
			if attempt == 1 {
				time.Sleep(10 * time.Millisecond)
				continue
			}
			return 0, payload, 0, lastErr
		}

		if payload[0] == 0xFF && payload[1] == 0xFF {
			lastErr = errors.New("invalid payload: all 0xFF")
			if d.debug {
				log.Printf("pHboard_driver addr=0x%02X read attempt=%d error=%v payload=% X", d.addr, attempt, lastErr, payload)
			}
			if attempt == 1 {
				time.Sleep(10 * time.Millisecond)
				continue
			}
			return 0, payload, 0, lastErr
		}

		code := adcI2C16ToCode(payload)
		v := adcCodeToVolts16(code, d.vrefV)
		mv = v * 1000.0

		d.lastSampleAt = time.Now()
		d.lastMV = mv
		d.lastRaw = append([]byte(nil), payload...)
		d.lastCode = code

		time.Sleep(settleAfterRead)

		return mv, payload, code, nil
	}

	return 0, nil, 0, lastErr
}

func adcI2C16ToCode(b []byte) int32 {
	return int32(int16(binary.BigEndian.Uint16(b)))
}

func adcCodeToVolts16(code int32, vref float64) float64 {
	return (float64(code) / 32768.0) * vref
}

func clampPH(v float64) float64 {
	if v < 0 {
		return 0
	}
	if v > 14 {
		return 14
	}
	return v
}

func linearMap(x, x1, x2, y1, y2 float64) float64 {
	den := x2 - x1
	if math.Abs(den) < 1e-9 {
		return y1
	}
	t := (x - x1) / den
	return y1 + t*(y2-y1)
}

func (d *phDriver) enabledAnchors() []anchor {
	var as []anchor

	if d.obs4mV != -1 {
		as = append(as, anchor{truePH: 4.0, obsMV: d.obs4mV, label: "pH4"})
	}
	if d.obs7mV != -1 {
		as = append(as, anchor{truePH: 7.0, obsMV: d.obs7mV, label: "pH7"})
	}
	if d.obs10mV != -1 {
		as = append(as, anchor{truePH: 10.0, obsMV: d.obs10mV, label: "pH10"})
	}

	sort.Slice(as, func(i, j int) bool {
		return as[i].truePH < as[j].truePH
	})

	return as
}

func (d *phDriver) idealSlope25C(debugLog bool) float64 {
	if d.slopeOverride != 0 {
		if debugLog {
			log.Printf("pHboard_driver addr=0x%02X slope: using override %.4f mV/pH @25C",
				d.addr, d.slopeOverride)
		}
		return d.slopeOverride
	}

	if debugLog {
		log.Printf("pHboard_driver addr=0x%02X slope: fallback ideal %.4f mV/pH @25C",
			d.addr, -idealSlope25C)
	}
	return -idealSlope25C
}

func (d *phDriver) slopeAtTemp(slope25 float64) (slope float64, enabled bool, reason string) {
	if !d.doTempComp {
		return slope25, false, "disabled by configuration"
	}

	tk := d.tempC + 273.15
	if tk <= 0 {
		return slope25, false, "invalid temperature; using 25C slope"
	}

	s := slope25 * (tk / refTempK25C)
	return s, true, ""
}

func (d *phDriver) rawMVToPHIdeal(mv float64, debugLog bool) (ph float64, slopeUsed float64) {
	s25 := d.idealSlope25C(debugLog)
	slope, enabled, reason := d.slopeAtTemp(s25)

	if slope == 0 || math.IsNaN(slope) || math.IsInf(slope, 0) {
		slope = -idealSlope25C
	}

	neutralMV := 0.0
	if d.obs7mV != -1 {
		neutralMV = d.obs7mV
	}

	ph = 7.0 + ((mv - neutralMV) / slope)

	if debugLog {
		if enabled {
			log.Printf("pHboard_driver addr=0x%02X ideal model: temp compensation enabled slope25=%.4f slopeT=%.4f reason=%s",
				d.addr, s25, slope, reason)
		} else {
			log.Printf("pHboard_driver addr=0x%02X ideal model: slope25=%.4f slopeUsed=%.4f reason=%s",
				d.addr, s25, slope, reason)
		}
		log.Printf("pHboard_driver addr=0x%02X ideal equation: pH = 7.0000 + ((%.2f - %.2f) / %.4f) = %.4f",
			d.addr, mv, neutralMV, slope, ph)
	}

	return ph, slope
}

func (d *phDriver) calibratedPHFromMV(mv float64, debugLog bool) (ph float64, slopeUsed float64, mode string) {
	as := d.enabledAnchors()

	idealPH, idealSlope := d.rawMVToPHIdeal(mv, debugLog)

	switch len(as) {
	case 0:
		if debugLog {
			log.Printf("pHboard_driver addr=0x%02X calibration mode: 0-point (ideal model)", d.addr)
		}
		return clampPH(idealPH), idealSlope, "0-point ideal"

	case 1:
		anchorIdeal, _ := d.rawMVToPHIdeal(as[0].obsMV, false)
		offset := as[0].truePH - anchorIdeal
		ph = idealPH + offset
		if debugLog {
			log.Printf("pHboard_driver addr=0x%02X calibration mode: 1-point using %s obsMV=%.2f truePH=%.2f idealAtAnchor=%.4f offset=%.4f",
				d.addr, as[0].label, as[0].obsMV, as[0].truePH, anchorIdeal, offset)
		}
		return clampPH(ph), idealSlope, "1-point offset"

	case 2:
		slope := (as[1].obsMV - as[0].obsMV) / (as[1].truePH - as[0].truePH)
		ph = linearMap(mv, as[0].obsMV, as[1].obsMV, as[0].truePH, as[1].truePH)
		if debugLog {
			log.Printf("pHboard_driver addr=0x%02X calibration mode: 2-point linear using %s=%.2f and %s=%.2f slope=%.4f mV/pH",
				d.addr, as[0].label, as[0].obsMV, as[1].label, as[1].obsMV, slope)
			log.Printf("pHboard_driver addr=0x%02X 2-point equation: pH = map(%.2f, %.2f→%.2f mV, %.2f→%.2f pH) = %.4f",
				d.addr, mv, as[0].obsMV, as[1].obsMV, as[0].truePH, as[1].truePH, ph)
		}
		return clampPH(ph), slope, "2-point linear"

	default:
		a0, a1, a2 := as[0], as[1], as[2]
		var segA, segB anchor

		// With pH probes the slope is normally negative, so higher pH usually means lower mV.
		// Using the midpoint anchor as the breakpoint gives a clean piecewise 3-point map.
		if mv >= a1.obsMV {
			segA, segB = a0, a1
		} else {
			segA, segB = a1, a2
		}

		slope := (segB.obsMV - segA.obsMV) / (segB.truePH - segA.truePH)
		ph = linearMap(mv, segA.obsMV, segB.obsMV, segA.truePH, segB.truePH)

		if debugLog {
			log.Printf("pHboard_driver addr=0x%02X calibration mode: 3-point piecewise using segment %s→%s slope=%.4f mV/pH",
				d.addr, segA.label, segB.label, slope)
			log.Printf("pHboard_driver addr=0x%02X 3-point equation: pH = map(%.2f, %.2f→%.2f mV, %.2f→%.2f pH) = %.4f",
				d.addr, mv, segA.obsMV, segB.obsMV, segA.truePH, segB.truePH, ph)
		}

		return clampPH(ph), slope, "3-point piecewise"
	}
}

func (p *phPin) Value() (float64, error) {
	mv, raw, code, err := p.parent.readObservedMV()
	if err != nil {
		if p.parent.debug {
			log.Printf("pHboard_driver addr=0x%02X read error: %v", p.parent.addr, err)
		}
		return 0, err
	}

	ph, slope, mode := p.parent.calibratedPHFromMV(mv, p.parent.debug)

	if p.parent.debug {
		log.Printf("pHboard_driver addr=0x%02X raw=% X adc=%d observed_mv=%.2f tempC=%.2f mode=%s",
			p.parent.addr, raw, code, mv, p.parent.tempC, mode)
		log.Printf("pHboard_driver addr=0x%02X anchors: pH4=%.2f pH7=%.2f pH10=%.2f slope_used=%.4f",
			p.parent.addr, p.parent.obs4mV, p.parent.obs7mV, p.parent.obs10mV, slope)
	}

	return ph, nil
}

func (p *phPin) Measure() (float64, error) { return p.Value() }

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
			p.parent.obs7mV = obs
			log.Printf("pHboard_driver calibrated Obs7_mV=%.2f", obs)
		case exp == 4:
			p.parent.obs4mV = obs
			log.Printf("pHboard_driver calibrated Obs4_mV=%.2f", obs)
		case exp == 10:
			p.parent.obs10mV = obs
			log.Printf("pHboard_driver calibrated Obs10_mV=%.2f", obs)
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

func (p *phPin) Snapshot() (hal.Snapshot, error) {
	mv, raw, code, err := p.parent.readObservedMV()
	if err != nil {
		return hal.Snapshot{}, err
	}

	ph, slope, mode := p.parent.calibratedPHFromMV(mv, false)

	s25 := p.parent.idealSlope25C(false)
	sT, enabled, reason := p.parent.slopeAtTemp(s25)
	if enabled && reason == "" {
		reason = "Nernst slope scaled by absolute temperature (used for 0/1-point ideal model)"
	}

	notes := []string{}
	if p.parent.doTempComp {
		if p.parent.tempUpdatedAt.IsZero() {
			notes = append(notes, "Temp compensation enabled but temperature has never been injected; 0/1-point results may be off.")
		} else if age := time.Since(p.parent.tempUpdatedAt); age > 2*time.Minute {
			notes = append(notes, fmt.Sprintf("Temperature is stale (age=%v); check temp sensor injection.", age))
		}
	} else {
		notes = append(notes, "Temp compensation disabled (explicit by configuration).")
	}

	meta := map[string]any{
		"channel": p.ch,

		"calibration_observed_key": "observed_mv",
		"raw_signal_key":           "observed_mv",
		"primary_signal_key":       "value",
		"secondary_signal_keys":    []string{"slope_used", "tempC", "obs7_mV", "obs4_mV", "obs10_mV", "adc_code"},

		"display_roles": map[string]any{
			"primary":  "Primary (pH)",
			"observed": "Observed (electrode mV)",
		},
		"display_names": map[string]any{
			"value":       "pH (calibrated)",
			"observed_mv": "Electrode (mV)",
			"slope_used":  "Slope used (mV/pH)",
			"tempC":       "Temperature (°C)",
			"obs7_mV":     "Anchor: pH7 (mV)",
			"obs4_mV":     "Anchor: pH4 (mV)",
			"obs10_mV":    "Anchor: pH10 (mV)",
			"adc_code":    "ADC code",
			"raw_hex":     "Raw bytes (hex)",
			"mode":        "Calibration mode",
		},
		"display_help": map[string]any{
			"observed_mv": "Raw physical electrode millivolts from the I2C ADC module.",
			"slope_used":  "Slope used by the active calibration mode. For 0/1-point this comes from the ideal or override slope; for 2/3-point it comes from anchor mapping.",
			"obs7_mV":     "Measured electrode mV in pH 7 buffer. Set to -1 to disable.",
			"obs4_mV":     "Measured electrode mV in pH 4 buffer. Set to -1 to disable.",
			"obs10_mV":    "Measured electrode mV in pH 10 buffer. Set to -1 to disable.",
			"mode":        "0-point ideal, 1-point offset, 2-point linear, or 3-point piecewise calibration.",
		},
		"signal_decimals": map[string]any{
			"value":       3,
			"observed_mv": 2,
			"slope_used":  4,
			"tempC":       2,
			"obs7_mV":     2,
			"obs4_mV":     2,
			"obs10_mV":    2,
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
		"calibration_mode": mode,
	}

	return hal.Snapshot{
		Value: ph,
		Unit:  "pH",
		Signals: map[string]hal.Signal{
			"observed_mv": {Now: mv, Unit: "mV"},
			"slope_used":  {Now: slope, Unit: "mV/pH"},
			"tempC":       {Now: p.parent.tempC, Unit: "C"},
			"obs7_mV":     {Now: p.parent.obs7mV, Unit: "mV"},
			"obs4_mV":     {Now: p.parent.obs4mV, Unit: "mV"},
			"obs10_mV":    {Now: p.parent.obs10mV, Unit: "mV"},
			"adc_code":    {Now: float64(code), Unit: ""},
			"raw_hex":     {Now: 0, Unit: fmt.Sprintf("% X", raw)},
		},
		Meta: meta,
		Notes: append(notes,
			"Use -1 to disable any calibration anchor.",
			"0 enabled points: ideal model. 1 enabled point: offset on ideal model. 2 enabled points: direct linear anchor mapping. 3 enabled points: piecewise linear anchor mapping.",
			"Driver includes min-gap + cache + retry to avoid I2C timing failures during calibration UI.",
			"If you run pH + ORP drivers at the same I2C address, a global per-address lock prevents read collisions.",
		),
	}, nil
}

func (d *phDriver) Name() string           { return driverName }
func (d *phDriver) Close() error           { return nil }
func (d *phDriver) Metadata() hal.Metadata { return d.meta }

func (d *phDriver) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if n != 0 {
		return nil, fmt.Errorf("%s supports only channel 0 (pH). Asked:%d", driverName, n)
	}
	return d.pins[0], nil
}

func (d *phDriver) AnalogInputPins() []hal.AnalogInputPin {
	return []hal.AnalogInputPin{d.pins[0]}
}

func (d *phDriver) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		return []hal.Pin{d.pins[0]}, nil
	default:
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
}