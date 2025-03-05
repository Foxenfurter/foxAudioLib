// Package: github.com/Foxenfurter/foxAudioLib/foxConvolver
// filename foxConvolver.go
// Package is designed to manage convolution of Filter Impulse Responses
// includes any logic neeeded to prepare the filter for convolution, such as Resampling and Normalization
package foxConvolver

import (
	"math"
	"math/cmplx"

	gofft "github.com/argusdusty/gofft"
	"scientificgo.org/fft"
)

const packageName = "foxConvolver"

// Convolver represents a convolver structure
type Convolver struct {
	FilterImpulse     []float64 // Impulse response
	impulseLength     int       // Length of the impulse response
	signalBlockLength int       // Length of signal block
	overlapLength     int       // Length of overlap
	paddedLength      int       // Length of padded impulse response
	outputLength      int
	impulseFFT        []complex128
	overlapTail       []float64 // Overlap tail
	Buffer            []float64 // Buffer to manage difference between streamed signal size and target size for convolution
	//streaming         bool
	DebugFunc   func(string)
	WarningFunc func(string)
}

func (myConvolver *Convolver) GetPaddedLength() int {
	return myConvolver.paddedLength

}

// Init initializes the convolver structure
// supply FIR filter impulse and overlap factor as a value between 0.1 and 0.9
func NewConvolver(impulse []float64) Convolver {
	newConvolver := Convolver{
		FilterImpulse: impulse,
	}

	return newConvolver
}

// Allows the impulse to be changed, but forces the length to be updated and nominally wrong so that
// the convolver process will recalulcate PaddedLength and FilterImpulseFFT values
func (myConvolver *Convolver) AmendFilterImpulse(impulse []float64) {
	myConvolver.FilterImpulse = impulse

}

// convert an arry of float64 to complex128 using input as the real numbers
func convertFloat64ToComplex128(data []float64) []complex128 {
	converted := make([]complex128, len(data))
	for i := range data {
		converted[i] = complex(data[i], 0)
	}
	return converted
}

// convert an arry of complex128 back to float64 extracting real numbers
func convertComplex128ToFloat64(data []complex128) []float64 {
	converted := make([]float64, len(data))
	for i, r := range data {
		converted[i] = real(r)
	}
	return converted
}

// return the next power of 2 that is greater than the input
func NextPowerOf2(x int) int {
	return int(math.Pow(2, math.Ceil(math.Log2(float64(x)))))
}

func DustyConvolver(signalBlock []float64, impulse []float64) []float64 {

	cmplxSignal := gofft.Float64ToComplex128Array(signalBlock)
	cmplximpulse := gofft.Float64ToComplex128Array(impulse)
	output, err := gofft.Convolve(cmplxSignal, cmplximpulse)
	if err != nil {
		println("Error with dusty")
	}
	return gofft.Complex128ToFloat64Array(output)
}

func (myConvolver *Convolver) DustyConvolver(signalBlock []float64) []float64 {
	if myConvolver.FilterImpulse == nil || len(myConvolver.FilterImpulse) == 0 {
		return signalBlock
	}
	//println("Filter Length: ", len(myConvolver.FilterImpulse), " FilterImpulse length: ", myConvolver.impulseLength)
	// Check to see if any inputs to the calculation have changed and recalulate if necessary
	if len(myConvolver.FilterImpulse) != myConvolver.impulseLength || len(signalBlock) != myConvolver.signalBlockLength {

		println("Using Dusty...")
		myConvolver.signalBlockLength = len(signalBlock)           // L
		myConvolver.impulseLength = len(myConvolver.FilterImpulse) // M

		myConvolver.overlapLength = myConvolver.impulseLength    // M-1
		myConvolver.outputLength = myConvolver.signalBlockLength //equal to L
		// we have pre-computed the signal block length
		myConvolver.paddedLength = (myConvolver.signalBlockLength + myConvolver.overlapLength) - 1 // L+M-1 = N -- not technically necessary for next power of 2
		myConvolver.impulseFFT = gofft.Float64ToComplex128Array(myConvolver.FilterImpulse)
		myConvolver.overlapTail = make([]float64, myConvolver.overlapLength)

	}

	cmplxSignal := append(myConvolver.overlapTail, (signalBlock)...)
	// now create next tail - it needs to be done pre-convolution
	start := len(cmplxSignal) - myConvolver.overlapLength
	end := start + myConvolver.overlapLength

	myConvolver.overlapTail = cmplxSignal[start:end]

	output, err := gofft.Convolve(gofft.Float64ToComplex128Array(cmplxSignal), myConvolver.impulseFFT)

	if err != nil {
		println("Error with dusty")
	}
	start = myConvolver.overlapLength
	end = start + myConvolver.signalBlockLength

	return gofft.Complex128ToFloat64Array(output[start:end])
}

//Convolver using Overlap and save method. Allows for signal to be sent in blocks e.g. streaming or via channels

func (myConvolver *Convolver) ConvolveOverlapSave(signalBlock []float64) []float64 {
	if myConvolver.FilterImpulse == nil || len(myConvolver.FilterImpulse) == 0 {
		return signalBlock
	}
	//println("Filter Length: ", len(myConvolver.FilterImpulse), " FilterImpulse length: ", myConvolver.impulseLength)
	// Check to see if any inputs to the calculation have changed and recalulate if necessary
	if len(myConvolver.FilterImpulse) != myConvolver.impulseLength || len(signalBlock) != myConvolver.signalBlockLength {

		myConvolver.signalBlockLength = len(signalBlock) // L
		myConvolver.InitForStreaming()

	}

	overlappedSignal := append(myConvolver.overlapTail, (signalBlock)...)
	// make sure that the signal is the same length as the padded filter.
	if myConvolver.paddedLength > len(overlappedSignal) {
		signalPadding := make([]float64, myConvolver.paddedLength-len(overlappedSignal))
		overlappedSignal = append(overlappedSignal, signalPadding...)
	}
	// now create next tail - it needs to be done pre-convolution - ignore any padding
	start := myConvolver.overlapLength + myConvolver.signalBlockLength - myConvolver.overlapLength
	end := start + myConvolver.overlapLength

	myConvolver.overlapTail = overlappedSignal[start:end]
	if len(overlappedSignal) != len(myConvolver.impulseFFT) {
		println("signal length: ", len(overlappedSignal), " Impulse: ", len(myConvolver.impulseFFT))
		return signalBlock
	}
	//now do the convolution

	output := myConvolver.convolve(convertFloat64ToComplex128(overlappedSignal))
	start = myConvolver.overlapLength
	end = start + myConvolver.signalBlockLength

	//return output[start:end]
	return output[start:end]
}

// minamalistic convolver computes fft and of signal and pre-computed fft for filter
// returns the complex value
// func (myConvolver *Convolver) convolve(signal, filter []complex128) []complex128 {
func (myConvolver *Convolver) convolve(signal []complex128) []float64 {

	signal = fft.Fft(signal, false)
	for i := 0; i < len(signal); i++ {
		signal[i] *= myConvolver.impulseFFT[i]

	}
	return convertComplex128ToFloat64(fft.Fft(signal, true))
}

// Initialise Convolver for Streaming convolution - prebuilds the Impulse FFT and sets up the overlap
// calculates the target size for the signal
func (myConvolver *Convolver) InitForStreaming() {
	// Check to see if any inputs to the calculation have changed and recalulate if necessary
	myConvolver.impulseLength = len(myConvolver.FilterImpulse) // M
	myConvolver.overlapLength = myConvolver.impulseLength      // M-1
	myConvolver.outputLength = myConvolver.signalBlockLength   //equal to L
	// we have pre-computed the signal block length
	myConvolver.paddedLength = NextPowerOf2(4 * myConvolver.impulseLength)
	//myConvolver.paddedLength = (myConvolver.signalBlockLength + myConvolver.overlapLength) // L+M-1 = N -- not technically necessary for next power of 2
	//
	paddedFilterImpulse := make([]complex128, myConvolver.paddedLength)
	// copy impulse into the padded impulse
	for i := 0; i < myConvolver.impulseLength; i++ {
		paddedFilterImpulse[i] = complex(myConvolver.FilterImpulse[i], 0)
	}
	// Fill remainder of padded impulse array with zeros
	for i := myConvolver.impulseLength; i < len(paddedFilterImpulse); i++ {
		paddedFilterImpulse[i] = complex(0, 0)
	}
	myConvolver.impulseFFT = fft.Fft(paddedFilterImpulse, false)
	myConvolver.overlapTail = make([]float64, myConvolver.overlapLength)

}

// Initialise Convolver for normal FFT type convolution & prebuilds the ImpulseFFT
func (myConvolver *Convolver) initNormal(ImpulseLength int, SignalBlockLength int) {
	// Check to see if any inputs to the calculation have changed and recalulate if necessary
	if ImpulseLength == 0 {
		//Houston we have a problem
		return
	}
	myConvolver.impulseLength = ImpulseLength         // M
	myConvolver.signalBlockLength = SignalBlockLength //L
	// multiplying by 4 is arbitrary - seems a reasonable compromise between performance and latency
	myConvolver.paddedLength = NextPowerOf2(max(ImpulseLength, SignalBlockLength)) //N
	myConvolver.outputLength = myConvolver.signalBlockLength                       //equal to L
	// we have pre-computed the signal block length
	//println("paddedLength: ", myConvolver.paddedLength, " FilterImpulse length: ", myConvolver.impulseLength, " signal: ", myConvolver.signalBlockLength)
	paddedFilterImpulse := make([]complex128, myConvolver.paddedLength)
	// copy impulse into the padded impulse
	for i := 0; i < myConvolver.impulseLength; i++ {
		paddedFilterImpulse[i] = complex(myConvolver.FilterImpulse[i], 0)
	}
	// Fill remainder of padded impulse array with zeros
	for i := myConvolver.impulseLength; i < len(paddedFilterImpulse); i++ {
		paddedFilterImpulse[i] = complex(0, 0)
	}
	myConvolver.impulseFFT = fft.Fft(paddedFilterImpulse, false)

}

// Convolver optimized for go channels. Reads an Input Channel and Sends to an output Channel
// Processing size is optimised for FFT performance hence is derived from the impulse length
// Must use overlap and save mechanism for convolving to avoid dropouts and other glitches.
func (myConvolver *Convolver) ConvolveChannel(inputSignalChannel, outputSignalChannel chan []float64) {
	myConvolver.InitForStreaming()
	totalProcessed := 0
	targetSignalLength := myConvolver.GetPaddedLength() - len(myConvolver.FilterImpulse)

	println("targetSignalLength", targetSignalLength)
	for inputBlock := range inputSignalChannel {
		totalProcessed += len(inputBlock)
		if len(myConvolver.FilterImpulse) == 0 {
			// Nothing to convolve with so just hand on the input
			outputSignalChannel <- inputBlock

		} else {
			// The convolver has a buffer for accumulating and managing the signal
			// adding to it here...
			myConvolver.Buffer = append(myConvolver.Buffer, inputBlock...)
			for {
				if len(myConvolver.Buffer) >= targetSignalLength {
					// handling OLS properly ~ 0.6s

					outputSignalChannel <- myConvolver.ConvolveOverlapSave(myConvolver.Buffer[:targetSignalLength])
					// reposition the buffer

					myConvolver.Buffer = myConvolver.Buffer[targetSignalLength:]

				} else {
					//means that we need to get more data.
					break
				}
			}
		}
	}
	if len(myConvolver.Buffer) > 0 {
		// flush the remaining data in the buffer
		outputSignalChannel <- myConvolver.ConvolveOverlapSave(myConvolver.Buffer)
		myConvolver.Buffer = make([]float64, 0)
	}

	println("Remaining Data in Buffer: ", len(myConvolver.Buffer), " Samples convolved: ", totalProcessed)
	println("Convolver Closing Channel: ")
	close(outputSignalChannel)

}

// Simple convolver multiplication calculation for baseline testing, setup to use pre fft Impulse
func (myConvolver *Convolver) ConvolveSlow(signalBlock []float64) []float64 {
	if myConvolver.FilterImpulse == nil || len(myConvolver.FilterImpulse) == 0 {
		return signalBlock
	}
	// Calculate output length based on convolution formula
	outputLength := len(myConvolver.FilterImpulse) + len(signalBlock) - 1

	// Initialize result slice with zeros
	result := make([]float64, outputLength)

	// Perform convolution using nested loops
	for i := 0; i < len(signalBlock); i++ {
		for j := 0; j < len(myConvolver.FilterImpulse); j++ {
			result[i+j] += myConvolver.FilterImpulse[j] * signalBlock[i]
		}
	}

	return result[:len(signalBlock)]
}

// Simple convolver multiplication calculation for baseline testing
func ConvolveSlow(impulse1 []float64, impulse2 []float64) []float64 {
	// Calculate output length based on convolution formula
	outputLength := len(impulse1) + len(impulse2) - 1

	// Initialize result slice with zeros
	result := make([]float64, outputLength)

	// Perform convolution using nested loops
	for i := 0; i < len(impulse1); i++ {
		for j := 0; j < len(impulse2); j++ {
			result[i+j] += impulse1[i] * impulse2[j]
		}
	}

	return result
}

// use convolver structure with precomputed Impulse values and then convolve signal
func (myConvolver *Convolver) ConvolveFFT(signal []float64) []float64 {
	if myConvolver.FilterImpulse == nil || len(myConvolver.FilterImpulse) == 0 {
		return signal
	}
	signalLength := len(signal)

	myConvolver.initNormal(len(myConvolver.FilterImpulse), signalLength)

	paddedImpulse := make([]complex128, myConvolver.paddedLength)
	paddedSignal := make([]complex128, myConvolver.paddedLength)

	for i := 0; i < myConvolver.impulseLength; i++ {

		paddedImpulse[i] = complex(myConvolver.FilterImpulse[i], 0)
	}

	for i := 0; i < signalLength; i++ {

		paddedSignal[i] = complex(signal[i], 0)
	}

	myConvolver.impulseFFT = fft.Fft(paddedImpulse, false)
	//using a temporary variable for readability
	output := myConvolver.convolve(paddedSignal)
	//now truncate output back to original signal length
	return output[:signalLength]

}

// Function to handle debug calls, allowing for different logging implementations
func (myConvolver *Convolver) debug(message string) {

	if myConvolver.DebugFunc != nil {
		myConvolver.DebugFunc(message)
	} else { // if no external debug function available just print the message
		println(message)
	}

}

func (myConvolver *Convolver) warning(message string) {
	if myConvolver.WarningFunc != nil {
		myConvolver.WarningFunc(message)
	}
}

// simple Cooley-Tukey based FFT for baseline benchmark
func TukeyFFT(X []complex128) []complex128 {
	N := len(X)
	if N <= 1 {
		return X
	}

	// Divide
	even := make([]complex128, N/2)
	odd := make([]complex128, N/2)
	for i := 0; i < N/2; i++ {
		even[i] = X[i*2]
		odd[i] = X[i*2+1]
	}

	// Conquer
	T := TukeyFFT(even)
	U := TukeyFFT(odd)
	x := make([]complex128, N)
	// Combine
	for i := 0; i < N/2; i++ {
		t := cmplx.Rect(1, -2*math.Pi*float64(i)/float64(N)) * U[i]
		x[i] = T[i] + t
		x[i+N/2] = T[i] - t
	}

	return x
}

// simple Cooley-Tukey based inverse FFT for baseline benchmark
func InverseTukeyFFT(X []complex128) []complex128 {
	N := len(X)
	x := make([]complex128, N)

	// Take the conjugate of the input sequence
	for i := 0; i < N; i++ {
		x[i] = cmplx.Conj(X[i])
	}

	// Compute the FFT of the conjugated sequence
	x = TukeyFFT(x)

	// Take the conjugate of the result and normalize
	for i := 0; i < N; i++ {
		//original
		x[i] = cmplx.Conj(x[i]) / complex(float64(N), 0)

	}

	return x
}

func CalculateFFT(input []complex128) []complex128 {
	n := len(input)
	if n == 1 {
		return []complex128{input[0]}
	}

	// Calculate the twiddle factor
	exp := cmplx.Rect(1, -2*math.Pi/float64(n))
	wn := complex(1, 0)

	// Divide the input into two halves
	a0 := CalculateFFT(input[:n/2])
	a1 := CalculateFFT(input[n/2:])

	// Combine the results using FFT formula
	result := make([]complex128, n)
	for i := range result {
		if i >= n/2 {
			wn *= exp
		}
		result[i] = a0[i%(n/2)] + wn*a1[i%(n/2)]
	}

	return result
}

func CalculateInverseFFT(input []complex128) []complex128 {
	n := len(input)
	if n == 1 {
		return []complex128{input[0]}
	}

	// Calculate the twiddle factor
	exp := cmplx.Rect(1, 2*math.Pi/float64(n))
	wn := complex(1, 0)

	// Divide the input into two halves
	a0 := CalculateInverseFFT(input[:n/2])
	a1 := CalculateInverseFFT(input[n/2:])

	// Combine the results using IFFT formula
	result := make([]complex128, n)
	for i := range result {
		if i >= n/2 {
			wn *= exp
		}
		result[i] = a0[i%(n/2)] + wn*a1[i%(n/2)]
	}

	return result
}
