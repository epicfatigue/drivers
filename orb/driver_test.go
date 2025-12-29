package orb

import (
	"math"
	"testing"

	"github.com/reef-pi/hal"
	"github.com/reef-pi/rpi/i2c"
)

var params = map[string]interface{}{
	"Address":       0x24,
	"ADCMax":        16383,
	"Vref":          3.3,
	"ORPRefVoltage": 1.8491,
	"ORPRefmV":      256.0,
}

func setBusBytesForVoltage(bus *i2c.MockBus, v float64, adcMax int, vref float64) {
	adc := int(math.Round((v / vref) * float64(adcMax)))
	if adc < 0 {
		adc = 0
	}
	if adc > adcMax {
		adc = adcMax
	}
	raw := uint16(adc << 2) // stored left-shifted by 2 bits
	bus.Bytes[0] = byte(raw >> 8)
	bus.Bytes[1] = byte(raw & 0xFF)
}

func TestOrbDriver(t *testing.T) {
	bus := i2c.MockBus()
	bus.Bytes = make([]byte, 2)

	f := Factory()

	// invalid config should fail
	if _, err := f.NewDriver(nil, bus); err == nil {
		t.Error("creation should fail when config is invalid")
	}

	driver, err := f.NewDriver(params, bus)
	if err != nil {
		t.Fatal(err)
	}

	if driver.Metadata().Name != "orb" {
		t.Error("unexpected driver name")
	}
	if !driver.Metadata().HasCapability(hal.AnalogInput) {
		t.Error("AnalogInput capability should exist")
	}
	if driver.Metadata().HasCapability(hal.DigitalInput) {
		t.Error("DigitalInput capability should not exist")
	}

	d := driver.(hal.AnalogInputDriver)

	if len(d.AnalogInputPins()) != 1 {
		t.Error("expected only one channel")
	}
	if _, err := d.AnalogInputPin(1); err == nil {
		t.Error("expected error for invalid channel")
	}

	ch, err := d.AnalogInputPin(0)
	if err != nil {
		t.Fatal(err)
	}
	if ch.Name() != "0" {
		t.Error("unexpected channel name")
	}

	// Set bytes to represent ~1.8491V
	setBusBytesForVoltage(bus, 1.8491, 16383, 3.3)

	v, err := ch.Value() // voltage
	if err != nil {
		t.Fatal(err)
	}
	if v < 1.848 || v > 1.851 {
		t.Errorf("unexpected voltage: %f", v)
	}

	// Measure should be ~256.0mV when voltage matches reference voltage
	orp, err := ch.Measure()
	if err != nil {
		t.Fatal(err)
	}
	if orp < 255.9 || orp > 256.1 {
		t.Errorf("unexpected ORP: %f", orp)
	}

	if err := d.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestFloatAddress(t *testing.T) {
	bus := i2c.MockBus()
	bus.Bytes = make([]byte, 2)

	floatAddress := float64(36) // 0x24
	floatParams := map[string]interface{}{
		"Address":       floatAddress,
		"ADCMax":        16383,
		"Vref":          3.3,
		"ORPRefVoltage": 1.8491,
		"ORPRefmV":      256.0,
	}

	f := Factory()
	if _, err := f.NewDriver(floatParams, bus); err != nil {
		t.Error("orb should convert float address to int before casting to byte")
	}
}

func TestCalibrateUpdatesReference(t *testing.T) {
	bus := i2c.MockBus()
	bus.Bytes = make([]byte, 2)

	f := Factory()
	driver, err := f.NewDriver(params, bus)
	if err != nil {
		t.Fatal(err)
	}

	d := driver.(hal.AnalogInputDriver)
	ch, err := d.AnalogInputPin(0)
	if err != nil {
		t.Fatal(err)
	}

	// Set a new calibration reference:
	// Expected = known ORP (mV), Observed = measured voltage (V)
	if err := ch.Calibrate([]hal.Measurement{
		{Expected: 400, Observed: 2.0000},
	}); err != nil {
		t.Fatal(err)
	}

	// Encode bytes for 2.0V *consistently*
	setBusBytesForVoltage(bus, 2.0, 16383, 3.3)

	orp, err := ch.Measure()
	if err != nil {
		t.Fatal(err)
	}
	if orp < 399.9 || orp > 400.1 {
		t.Errorf("unexpected ORP after calibrate: %f", orp)
	}
}
