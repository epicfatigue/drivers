package orb

import (
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

	// For ~1.8491V:
	// adc ≈ round(1.8491/3.3 * 16383) = 9180
	// raw = adc << 2 = 36720 = 0x8F70
	bus.Bytes[0] = 0x8F
	bus.Bytes[1] = 0x70

	v, err := ch.Value() // voltage
	if err != nil {
		t.Fatal(err)
	}
	if v < 1.84 || v > 1.86 {
		t.Errorf("unexpected voltage: %f", v)
	}

	// Measure should be ~256.0mV when voltage matches reference voltage
	orp, err := ch.Measure()
	if err != nil {
		t.Fatal(err)
	}
	if orp < 255.0 || orp > 257.0 {
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

	// IMPORTANT:
	// Your previous bytes (0x9B00) decode to ~1.998V, which yields ~398mV.
	// Use a value that decodes to ~2.0V:
	// adc ≈ round(2.0/3.3*16383)=9930, raw=9930<<2=0x9B28
	bus.Bytes[0] = 0x9B
	bus.Bytes[1] = 0x28

	orp, err := ch.Measure()
	if err != nil {
		t.Fatal(err)
	}
	// Allow a little tolerance due to ADC quantization.
	if orp < 399.0 || orp > 401.5 {
		t.Errorf("unexpected ORP after calibrate: %f", orp)
	}
}
