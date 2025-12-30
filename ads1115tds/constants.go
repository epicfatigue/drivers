package ads1115tds

// ADS1115 register pointers
const (
	regConversion = 0x00
	regConfig     = 0x01
)

// Config register bitfields (ADS1115)
const (
	// OS bit (start single conversion)
	configOsSingle = 0x8000

	// MUX single-ended (AINx vs GND)
	configMuxSingle0 = 0x4000 // 100 << 12
	configMuxSingle1 = 0x5000 // 101 << 12
	configMuxSingle2 = 0x6000 // 110 << 12
	configMuxSingle3 = 0x7000 // 111 << 12

	// PGA (gain) bits
	configGainTwoThirds = 0x0000 // +/-6.144V
	configGainOne       = 0x0200 // +/-4.096V
	configGainTwo       = 0x0400 // +/-2.048V
	configGainFour      = 0x0600 // +/-1.024V
	configGainEight     = 0x0800 // +/-0.512V
	configGainSixteen   = 0x0A00 // +/-0.256V

	// Mode
	configModeSingle = 0x0100 // single-shot

	// Data rate (ADS1115 max is 860 SPS)
	configDataRate860 = 0x00E0 // 111 << 5

	// Comparator (disabled)
	configComparatorModeTraditional       = 0x0000
	configComparitorNonLatching           = 0x0000
	configComparitorPolarityActiveLow     = 0x0000
	configComparitorQueueNone             = 0x0003 // disable comparator
)
