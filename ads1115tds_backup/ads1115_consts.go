package ads1115tds

// ADS1115 registers
const (
	regConversion = 0x00
	regConfig     = 0x01
)

// OS / Mode
const (
	configOsSingle   uint16 = 0x8000
	configModeSingle uint16 = 0x0100
)

// Mux (single-ended AINx vs GND)
const (
	configMuxSingle0 uint16 = 0x4000 // AIN0
	configMuxSingle1 uint16 = 0x5000 // AIN1
	configMuxSingle2 uint16 = 0x6000 // AIN2
	configMuxSingle3 uint16 = 0x7000 // AIN3
)

// PGA gain (full-scale range)
const (
	configGainTwoThirds uint16 = 0x0000 // +/- 6.144V
	configGainOne       uint16 = 0x0200 // +/- 4.096V
	configGainTwo       uint16 = 0x0400 // +/- 2.048V
	configGainFour      uint16 = 0x0600 // +/- 1.024V
	configGainEight     uint16 = 0x0800 // +/- 0.512V
	configGainSixteen   uint16 = 0x0A00 // +/- 0.256V
)

// Data rate (SPS)
const (
	configDataRate860 uint16 = 0x00E0 // 860 SPS (max on ADS1115)
)

// Comparator: disabled
const (
	configComparatorModeTraditional   uint16 = 0x0000
	configComparitorNonLatching       uint16 = 0x0000
	configComparitorPolarityActiveLow uint16 = 0x0000
	configComparitorQueueNone         uint16 = 0x0003
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
