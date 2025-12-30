package ads1115tds

// ADS1115 register pointers
const (
	regConversion = 0x00
	regConfig     = 0x01
)

// ADS1115 config bits
const (
	// Operational status / single-shot start
	configOsSingle  = 0x8000
	configOsBusy    = 0x0000
	configOsNotBusy = 0x8000

	// Input mux (single-ended)
	configMuxSingle0 = 0x4000
	configMuxSingle1 = 0x5000
	configMuxSingle2 = 0x6000
	configMuxSingle3 = 0x7000

	// PGA gain
	configGainTwoThirds = 0x0000 // +/- 6.144V
	configGainOne       = 0x0200 // +/- 4.096V
	configGainTwo       = 0x0400 // +/- 2.048V
	configGainFour      = 0x0600 // +/- 1.024V
	configGainEight     = 0x0800 // +/- 0.512V
	configGainSixteen   = 0x0A00 // +/- 0.256V

	// Mode
	configModeContinuous = 0x0000
	configModeSingle     = 0x0100

	// Data rate (ADS1115)
	configDataRate8    = 0x0000
	configDataRate16   = 0x0020
	configDataRate32   = 0x0040
	configDataRate64   = 0x0060
	configDataRate128  = 0x0080
	configDataRate250  = 0x00A0
	configDataRate475  = 0x00C0
	configDataRate860  = 0x00E0

	// Comparator (we disable it)
	configComparatorModeTraditional = 0x0000
	configComparatorModeWindow      = 0x0010

	configComparatorPolarityActiveLow  = 0x0000
	configComparatorPolarityActiveHigh = 0x0008

	configComparatorNonLatching = 0x0000
	configComparatorLatching    = 0x0004

	configComparatorQueue1    = 0x0000
	configComparatorQueue2    = 0x0001
	configComparatorQueue4    = 0x0002
	configComparatorQueueNone = 0x0003
)

var channelMux = [4]uint16{
	configMuxSingle0,
	configMuxSingle1,
	configMuxSingle2,
	configMuxSingle3,
}
