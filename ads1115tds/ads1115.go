package ads1115tds

// ADS1115 register pointers
const (
	regConversion = 0x00
	regConfig     = 0x01
)

// Config bit fields (ADS1115)
const (
	// Operational status / single-shot start
	configOsSingle = 0x8000

	// Mux (single-ended AINx vs GND)
	configMuxSingle0 = 0x4000
	configMuxSingle1 = 0x5000
	configMuxSingle2 = 0x6000
	configMuxSingle3 = 0x7000

	// PGA gain
	configGainTwoThirds = 0x0000 // +/-6.144V
	configGainOne       = 0x0200 // +/-4.096V
	configGainTwo       = 0x0400 // +/-2.048V
	configGainFour      = 0x0600 // +/-1.024V
	configGainEight     = 0x0800 // +/-0.512V
	configGainSixteen   = 0x0A00 // +/-0.256V

	// Mode
	configModeContinuous = 0x0000
	configModeSingle     = 0x0100

	// Data rate (SPS)
	configDataRate8    = 0x0000
	configDataRate16   = 0x0020
	configDataRate32   = 0x0040
	configDataRate64   = 0x0060
	configDataRate128  = 0x0080
	configDataRate250  = 0x00A0
	configDataRate475  = 0x00C0
	configDataRate860  = 0x00E0
	configDataRate1600 = configDataRate860 // keep name from earlier code; ADS1115 max is 860

	// Comparator (we disable it)
	configComparatorModeTraditional    = 0x0000
	configComparitorNonLatching        = 0x0000
	configComparitorPolarityActiveLow  = 0x0000
	configComparitorQueueNone          = 0x0003
)

func muxForChannel(ch int) (uint16, bool) {
	switch ch {
	case 0:
		return configMuxSingle0, true
	case 1:
		return configMuxSingle1, true
	case 2:
		return configMuxSingle2, true
	case 3:
		return configMuxSingle3, true
	default:
		return 0, false
	}
}

func fsVoltsForGain(gain uint16) (float64, bool) {
	switch gain {
	case configGainTwoThirds:
		return 6.144, true
	case configGainOne:
		return 4.096, true
	case configGainTwo:
		return 2.048, true
	case configGainFour:
		return 1.024, true
	case configGainEight:
		return 0.512, true
	case configGainSixteen:
		return 0.256, true
	default:
		return 0, false
	}
}
