package orp_board

import (
	"encoding/binary"
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
	driverName = "Orp_Driver I2C ORP (ADC→mV)"

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

type orpDriver struct {
	addr byte
	bus  i2c.Bus
	meta hal.Metadata

	vrefV         float64
	calibrationMV float64

	debug bool
	pins  []*orpPin

	mu sync.Mutex

	lastXferAt   time.Time
	lastSampleAt time.Time
	lastMV       float64
	lastRaw      []byte
	lastCode     int32
}

type orpPin struct {
	parent *orpDriver
	ch     int
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

func (d *orpDriver) enforceMinGap(minGap time.Duration) {
	if d.lastXferAt.IsZero() {
		return
	}
	elapsed := time.Since(d.lastXferAt)
	if elapsed < minGap {
		time.Sleep(minGap - elapsed)
	}
}

func (d *orpDriver) writeCmd(payload ...byte) error {
	d.lastXferAt = time.Now()
	return d.bus.WriteBytes(d.addr, payload)
}

func (d *orpDriver) initADC() error {
	lock := lockForAddr(d.addr)
	lock.Lock()
	defer lock.Unlock()

	d.mu.Lock()
	defer d.mu.Unlock()

	if d.debug {
		log.Printf("orp_board_driver addr=0x%02X init: reset, config=0x%02X, start continuous conversion",
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

func (d *orpDriver) readObservedMV() (mv float64, raw []byte, adcCode int32, err error) {
	lock := lockForAddr(d.addr)
	lock.Lock()
	defer lock.Unlock()

	d.mu.Lock()
	defer d.mu.Unlock()

	if !d.lastSampleAt.IsZero() && time.Since(d.lastSampleAt) < cacheMaxAge {
		if d.debug {
			log.Printf("orp_board_driver addr=0x%02X cache hit age=%v mv=%.2f",
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
				log.Printf("orp_board_driver addr=0x%02X read attempt=%d error=%v", d.addr, attempt, e)
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
				log.Printf("orp_board_driver addr=0x%02X read attempt=%d error=%v payload=% X", d.addr, attempt, lastErr, payload)
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
				log.Printf("orp_board_driver addr=0x%02X read attempt=%d error=%v payload=% X", d.addr, attempt, lastErr, payload)
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

func (p *orpPin) Value() (float64, error) {
	observedMV, raw, code, err := p.parent.readObservedMV()
	if err != nil {
		if p.parent.debug {
			log.Printf("orp_board_driver addr=0x%02X read error: %v", p.parent.addr, err)
		}
		return 0, err
	}

	correctedMV := observedMV
	offsetMV := 0.0

	if p.parent.calibrationMV != 0 {
		// calibrationMV = observed value when in 256mV solution
		offsetMV = 256.0 - p.parent.calibrationMV
		correctedMV = observedMV + offsetMV
	}

	if p.parent.debug {
		log.Printf("orp_board_driver addr=0x%02X raw=% X adc=%d", p.parent.addr, raw, code)

		if p.parent.calibrationMV != 0 {
			log.Printf(
				"orp_board_driver addr=0x%02X equation: observed_mV = (%d / 32768.0) * %.3f * 1000 = %.2f ; corrected_mV = %.2f + (256.00 - %.2f) = %.2f",
				p.parent.addr,
				code,
				p.parent.vrefV,
				observedMV,
				observedMV,
				p.parent.calibrationMV,
				correctedMV,
			)
		} else {
			log.Printf(
				"orp_board_driver addr=0x%02X equation: mV = (%d / 32768.0) * %.3f * 1000 = %.2f",
				p.parent.addr,
				code,
				p.parent.vrefV,
				observedMV,
			)
		}

		log.Printf(
			"orp_board_driver addr=0x%02X calibration: observed_at_256=%.2f offset=%.2f corrected=%.2f",
			p.parent.addr,
			p.parent.calibrationMV,
			offsetMV,
			correctedMV,
		)
	}

	return correctedMV, nil
}

func (p *orpPin) Measure() (float64, error) { return p.Value() }

func (p *orpPin) Calibrate(ms []hal.Measurement) error {
	return nil
}

func (p *orpPin) Name() string           { return driverName + " (ORP)" }
func (p *orpPin) Number() int            { return p.ch }
func (p *orpPin) Close() error           { return nil }
func (p *orpPin) Metadata() hal.Metadata { return p.parent.meta }

func (p *orpPin) Snapshot() (hal.Snapshot, error) {
	observedMV, raw, code, err := p.parent.readObservedMV()
	if err != nil {
		return hal.Snapshot{}, err
	}

	correctedMV := observedMV
	offsetMV := 0.0

	if p.parent.calibrationMV != 0 {
		offsetMV = 256.0 - p.parent.calibrationMV
		correctedMV = observedMV + offsetMV
	}

	meta := map[string]any{
		"channel": p.ch,

		"calibration_observed_key": "observed_mv",
		"raw_signal_key":           "observed_mv",
		"primary_signal_key":       "value",
		"secondary_signal_keys":    []string{"observed_mv", "offset_mv", "calibration_mv", "adc_code"},

		"display_roles": map[string]any{
			"primary":  "Primary (ORP mV)",
			"observed": "Observed (electrode mV)",
		},
		"display_names": map[string]any{
			"value":          "ORP (corrected mV)",
			"observed_mv":    "Observed (mV)",
			"offset_mv":      "Calibration offset (mV)",
			"calibration_mv": "Observed mV at 256mV solution",
			"adc_code":       "ADC code",
			"raw_hex":        "Raw bytes (hex)",
		},
		"display_help": map[string]any{
			"observed_mv":    "Raw electrode millivolts from the I2C ADC module before calibration correction.",
			"offset_mv":      "Offset applied so the stored observed reading in 256 mV solution maps to 256 mV.",
			"calibration_mv": "Measured ORP mV when probe was placed in a 256 mV calibration solution.",
		},
		"signal_decimals": map[string]any{
			"value":          2,
			"observed_mv":    2,
			"offset_mv":      2,
			"calibration_mv": 2,
			"adc_code":       0,
		},
	}

	notes := []string{
		"ORP driver reports electrode millivolts directly.",
		"Driver includes min-gap + cache + retry to avoid I2C timing failures.",
		"If you run pH + ORP drivers at the same I2C address, a global per-address lock prevents read collisions.",
	}

	if p.parent.calibrationMV == 0 {
		notes = append(notes, "Calibration correction disabled because Calibration_mV is 0.")
	} else {
		notes = append(notes, fmt.Sprintf("Calibration enabled: 256mV reference using observed %.2f mV, offset %.2f mV.", p.parent.calibrationMV, offsetMV))
	}

	return hal.Snapshot{
		Value: correctedMV,
		Unit:  "mV",
		Signals: map[string]hal.Signal{
			"observed_mv":    {Now: observedMV, Unit: "mV"},
			"offset_mv":      {Now: offsetMV, Unit: "mV"},
			"calibration_mv": {Now: p.parent.calibrationMV, Unit: "mV"},
			"adc_code":       {Now: float64(code), Unit: ""},
			"raw_hex":        {Now: 0, Unit: fmt.Sprintf("% X", raw)},
		},
		Meta:  meta,
		Notes: notes,
	}, nil
}

func (d *orpDriver) Name() string           { return driverName }
func (d *orpDriver) Close() error           { return nil }
func (d *orpDriver) Metadata() hal.Metadata { return d.meta }

func (d *orpDriver) AnalogInputPin(n int) (hal.AnalogInputPin, error) {
	if n != 0 {
		return nil, fmt.Errorf("%s supports only channel 0 (ORP). Asked:%d", driverName, n)
	}
	return d.pins[0], nil
}

func (d *orpDriver) AnalogInputPins() []hal.AnalogInputPin {
	return []hal.AnalogInputPin{d.pins[0]}
}

func (d *orpDriver) Pins(cap hal.Capability) ([]hal.Pin, error) {
	switch cap {
	case hal.AnalogInput:
		return []hal.Pin{d.pins[0]}, nil
	default:
		return nil, fmt.Errorf("unsupported capability: %s", cap.String())
	}
}