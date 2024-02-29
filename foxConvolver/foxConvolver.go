// Package: github.com/Foxenfurter/foxAudioLib/foxConvolver
// filename foxConvolver.go
// Package is designed to manage convolution of Filter Impulse Responses
// includes any logic neeeded to prepare the filter for convolution, such as Resampling and Normalization
package foxConvolver

import (
	"bytes"
	"fmt"
	"math"
	"math/cmplx"
	"os/exec"
	"strconv"

	"github.com/Foxenfurter/foxAudioLib/foxPEQ"

	"scientificgo.org/fft"
)

const packageName = "foxConvolver"

// Convolver represents a convolver structure
type Convolver struct {
	Impulse         []float64    // Impulse response
	ImpulseLength   int          // Length of the impulse response
	WindowedImpulse []float64    // Windowed impulse response
	OverlapFactor   float64      // Overlap factor
	OverlapLength   int          // Length of overlap
	HopSize         int          // Hop size
	OverlapTail     []complex128 // Overlap tail
	DebugFunc       func(string)
	WarningFunc     func(string)
}

// Init initializes the convolver structure
// supply FIR filter impulse and overlap factor as a value between 0.1 and 0.9
func NewConvolver(impulse []float64, overlapFactor float64) Convolver {
	newConvolver := Convolver{
		Impulse:       impulse,
		ImpulseLength: len(impulse),
	}

	// Apply Hamming window to impulse
	newConvolver.WindowedImpulse = make([]float64, newConvolver.ImpulseLength)
	hammingWindow := hammingWindow(newConvolver.ImpulseLength)
	for i := 0; i < newConvolver.ImpulseLength; i++ {
		newConvolver.WindowedImpulse[i] = impulse[i] * hammingWindow[i]
	}

	// Calculate overlap length, hop size, and overlap tail
	newConvolver.OverlapFactor = math.Max(0.1, math.Min(overlapFactor, 0.9))
	newConvolver.OverlapLength = int(float64(newConvolver.ImpulseLength) * newConvolver.OverlapFactor)
	newConvolver.HopSize = newConvolver.ImpulseLength - newConvolver.OverlapLength + 1
	newConvolver.OverlapTail = make([]complex128, newConvolver.OverlapLength-1)
	return newConvolver
}

func ReadnResampleFirFile(filePath string, targetSampleRate int) (*bytes.Reader, error) {
	mySampleRate := strconv.Itoa(targetSampleRate)
	//myCmd := "sox " + filePath + " --norm=-0.2  -r " + mySampleRate + " -t wav -"
	myCmd := "sox " + filePath + " -r " + mySampleRate + " -t  wav -"
	println(myCmd)
	//cmd := exec.Command(myCmd)
	//cmd := exec.Command("sox", filePath, "-r", mySampleRate, "-t", "wav", "-n", "-b", "16", "-q", "-")
	cmd := exec.Command("sox", filePath, "-r", mySampleRate, "-t", "wav", "-q", "-")
	// Create a bytes.Buffer to capture the output
	var out bytes.Buffer
	cmd.Stdout = &out

	// Run the command
	err := cmd.Run()
	if err != nil {
		println("Error running sox command, err: ", err)
		return nil, err
	}

	// The output is now in the 'out' buffer.
	// You can convert it to a bytes.Reader with bytes.NewReader()
	myReader := bytes.NewReader(out.Bytes())
	return myReader, nil

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

// const FilterLength = 4000
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

func hammingWindow(length int) []float64 {
	window := make([]float64, length)
	alpha := 0.54
	beta := 0.46
	for i := 0; i < length; i++ {
		window[i] = alpha - beta*math.Cos(2*math.Pi*float64(i)/float64(length-1))
	}
	return window
}

func convertFloat64ToComplex128(data []float64) []complex128 {
	converted := make([]complex128, len(data))
	for i := range data {
		converted[i] = complex(data[i], 0)
	}
	return converted
}

func ConvolveImpulseOverlapSave(impulse, signalBlock []float64, savedOverlap []complex128) ([]float64, []complex128) {
	impulseLength := len(impulse)
	blockLength := len(signalBlock)
	overlapSize := impulseLength - 1

	// Pad arrays to the nearest power of 2
	paddedLength := int(math.Pow(2, math.Ceil(math.Log2(float64(impulseLength+blockLength-overlapSize)))))
	paddedImpulse := make([]complex128, paddedLength)
	paddedSignal := make([]complex128, paddedLength)

	// Fill padded impulse array
	for i := 0; i < impulseLength; i++ {
		paddedImpulse[i] = complex(impulse[i], 0)
	}

	// Fill padded signal array with saved overlap and new signal block (zero padding if necessary)
	copy(paddedSignal, savedOverlap)
	copy(paddedSignal[overlapSize:], convertFloat64ToComplex128(signalBlock))
	if blockLength < overlapSize {
		for i := blockLength; i < overlapSize; i++ {
			paddedSignal[i+overlapSize] = complex(0, 0) // Zero padding
		}
	}

	// Apply FFT to both padded input arrays
	paddedImpulse = fft.Fft(paddedImpulse, false)
	paddedSignal = fft.Fft(paddedSignal, false)

	// Multiply the transformed arrays element-wise
	multipliedResult := make([]complex128, paddedLength)
	for i := 0; i < paddedLength; i++ {
		multipliedResult[i] = paddedImpulse[i] * paddedSignal[i]
	}

	// Apply inverse FFT to the multiplied result
	multipliedResult = fft.Fft(multipliedResult, true)

	// Extract the real part and discard the overlap
	result := make([]float64, blockLength)
	for i := 0; i < blockLength; i++ {
		result[i] = real(multipliedResult[i+overlapSize])
	}

	// Extract the new overlap for the next iteration
	copy(savedOverlap, multipliedResult[:overlapSize])

	return result, savedOverlap
}

// Simple convolver multiplication calculation for baseline testing
func ConvolveImpulsesSlow(impulse1 []float64, impulse2 []float64) []float64 {
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

	return result[:]
}

// use convolver directly without precomputed Impulse values and then convolve signal
// the impulse is the filter that you want to apply and the signal is what you want to apply it to - and output
// the original signal length will be kept
func ConvolveImpulsesFFT(impulse, signal []float64) []float64 {
	impulseLength := len(impulse)
	signalLength := len(signal)
	outputLength := impulseLength + signalLength - 1
	result := make([]float64, outputLength)

	// Pad input arrays to the nearest power of 2
	paddedLength := int(math.Pow(2, math.Ceil(math.Log2(float64(impulseLength+signalLength-1)))))
	paddedImpulse := make([]complex128, paddedLength)
	paddedSignal := make([]complex128, paddedLength)

	// Map impulse and signal over the length of the padded array with applied window
	for i := 0; i < impulseLength; i++ {
		//	paddedImpulse[i] = complex(impulse[i]*hammingWindowImpulse[i], 0)
		paddedImpulse[i] = complex(impulse[i], 0)
	}

	for i := 0; i < signalLength; i++ {
		paddedSignal[i] = complex(signal[i], 0)
	}

	// Apply FFT to both padded input arrays
	//println("Using Tukey FFT")
	//paddedImpulse = TukeyFFT(paddedImpulse)
	//paddedSignal = TukeyFFT(paddedSignal)

	paddedImpulse = fft.Fft(paddedImpulse, false)
	paddedSignal = fft.Fft(paddedSignal, false)

	// Multiply the transformed arrays element-wise
	multipliedResult := make([]complex128, paddedLength)
	for i := 0; i < paddedLength; i++ {
		multipliedResult[i] = paddedImpulse[i] * paddedSignal[i]
	}

	// Apply inverse FFT to the multiplied result
	multipliedResult = fft.Fft(multipliedResult, true)
	//multipliedResult = InverseTukeyFFT(multipliedResult)

	// Extract the real part and assign it to the result array
	for i := 0; i < outputLength; i++ {
		result[i] = real(multipliedResult[i])
	}

	return result[:signalLength]
}

// use convolver structure with precomputed Impulse values and then convolve signal
func (myConvolver *Convolver) ConvolveImpulsesFFT(signal []float64) []float64 {

	signalLength := len(signal)
	outputLength := myConvolver.ImpulseLength + signalLength - 1
	result := make([]float64, outputLength)

	// Pad input arrays to the nearest power of 2
	paddedLength := int(math.Pow(2, math.Ceil(math.Log2(float64(outputLength)))))
	paddedImpulse := make([]complex128, paddedLength)
	paddedSignal := make([]complex128, paddedLength)

	for i := 0; i < myConvolver.ImpulseLength; i++ {

		paddedImpulse[i] = complex(myConvolver.Impulse[i], 0)
	}

	for i := 0; i < signalLength; i++ {

		paddedSignal[i] = complex(signal[i], 0)
	}

	// Apply FFT to both padded input arrays
	paddedImpulse = fft.Fft(paddedImpulse, false)
	paddedSignal = fft.Fft(paddedSignal, false)

	// Multiply the transformed arrays element-wise
	multipliedResult := make([]complex128, paddedLength)
	for i := 0; i < paddedLength; i++ {
		multipliedResult[i] = paddedImpulse[i] * paddedSignal[i]
	}

	// Apply inverse FFT to the multiplied result
	multipliedResult = fft.Fft(multipliedResult, true)

	// Extract the real part and assign it to the result array
	for i := 0; i < outputLength; i++ {
		result[i] = real(multipliedResult[i])
	}
	// truncate the result back to the original signal length
	return result[:signalLength]
}

// Calculate the target gain level based on the sampling frequency
// parameters: Sample Rate (int) returns the target gain level as float64
func TargetGain(fromSampleRate, toSampleRate int) float64 {
	// Let's normalize EQ Impulse
	NormaliseRatio := float64(fromSampleRate) / float64(toSampleRate)
	var targetLevel float64

	// 0.05 too high, 0.025 too low
	if toSampleRate == 192000 || toSampleRate == 176000 {
		// I have tweaked the formula for highest sample rate

		targetLevel = 0.96 * (NormaliseRatio + (0.01 * 1 / NormaliseRatio))
	} else {
		targetLevel = 0.96 * (NormaliseRatio + (0.035 * 1 / NormaliseRatio))
	}

	return targetLevel
}

// Calculates the Maximum Gain Value - used for Normalization functions
func CalculateMaxGain(audioData []float64) float64 {
	maxGain := 0.0
	if audioData == nil {
		return maxGain
	}
	for _, sample := range audioData {
		sampleAbs := math.Abs(sample)
		if sampleAbs > maxGain {
			maxGain = sampleAbs
		}
	}
	return maxGain
}

// Normalises and audio signal to the target level and calculate  a max level.
// Max Level is therefore only for the current channel
func NormalizeAudioImpulseTrue(audioImpulse []float64, targetLevel float64) []float64 {
	if len(audioImpulse) == 0 {
		return audioImpulse
	}

	// Calculate max absolute value
	maxGain := math.Abs(audioImpulse[0])
	for _, sample := range audioImpulse {
		absSample := math.Abs(sample)
		if absSample > maxGain {
			maxGain = absSample
		}
	}

	// True normalization (scale to unit amplitude)
	normalizedImpulse := make([]float64, len(audioImpulse))
	for i := range audioImpulse {
		normalizedImpulse[i] = audioImpulse[i] / maxGain
	}

	// Apply target gain to achieve desired level
	targetGain := targetLevel / maxGain
	for i := range normalizedImpulse {
		normalizedImpulse[i] *= targetGain
	}

	return normalizedImpulse
}

// Normalises and audio signal to the target level using a max and target level.
// Max Level has been pre-calculated across all channels
func normalizeAudioImpulse(audioImpulse []float64, targetLevel float64, max float64) []float64 {
	// Check for divide by zero and no normalization needed
	println("NormalizeAudioImpulse max:", max, " target:", targetLevel)
	if max == 0.0 || max == targetLevel {
		return audioImpulse
	}

	// Calculate the normalization factor
	normalizationFactor := targetLevel / max

	// Normalize the audio impulse
	normalizedImpulse := make([]float64, len(audioImpulse))
	for i := range audioImpulse {
		normalizedImpulse[i] = audioImpulse[i] * normalizationFactor
	}

	return normalizedImpulse
}

// Normalizes Audio Data in supplied samples
func Normalize(inputSamples [][]float64, targetLevel float64) float64 {
	// Find max gain on all channels
	impulseGain := 0.0
	for _, channel := range inputSamples {
		impulseGain = max(CalculateMaxGain(channel), math.Abs(impulseGain))

	}
	if impulseGain == 0 {
		return impulseGain
	}

	// Normalize gain
	for i := range inputSamples {
		inputSamples[i] = normalizeAudioImpulse(inputSamples[i], targetLevel, impulseGain) // Replace with actual implementation
		//inputSamples[i] = NormalizeAudioImpulseTrue(inputSamples[i], targetLevel)

		//inputSamples[i] = normalizeAudio(inputSamples[i], impulseGain)
	}

	return impulseGain
}

// Downsample as this is proving poor from accuracy pov
func downsample(input []float64, fromRate, toRate int) []float64 {
	// Calculate the decimation factor

	decimationFactor := fromRate / toRate

	// Create a slice to hold the downsampled signal
	output := make([]float64, len(input)/decimationFactor)

	// Perform the decimation
	for i := range output {
		output[i] = input[i*decimationFactor]
	}

	return output
}

func downsampleComplex(input []float64, fromRate, toRate int) []float64 {

	K := float64(fromRate) / float64(toRate)
	var output []float64
	println("Downsample Output length:", len(input)/int(K), " input:", len(input), " K: ", K)
	// Downsample

	for n := 1; n <= len(input)/int(K); n++ {
		m := K * float64(n)
		mi := int(math.Floor(m))
		if mi < (len(input)/int(K))-1 {
			d := m - float64(mi)
			output = append(output, input[mi]*(d)+(1-d)*input[mi-1])
		} else {
			println("Mi: ", mi, " n:", n)
			break
		}

	}
	return output

}

// Resample channels using linear interpolation as a basis for benchmarking
func resampleLinear(input []float64, inRate, outRate int) []float64 {
	ratio := float64(inRate) / float64(outRate)
	output := make([]float64, int(float64(len(input))/ratio))
	println("resampling linear")
	for i := range output {
		inIndex := float64(i) * ratio
		inIndexInt := int(inIndex)
		frac := inIndex - float64(inIndexInt)

		if inIndexInt+1 < len(input) {
			output[i] = input[inIndexInt]*(1-frac) + input[inIndexInt+1]*frac
		} else {
			output[i] = input[inIndexInt]
		}
	}

	return output
}

// So far the best solution for internal resampling, although it works well for upsampling and poorly for downsampling
func resampleChannel(inputSamples []float64, fromSampleRate, toSampleRate, quality int) ([]float64, error) {
	// If no resampling required
	if fromSampleRate == toSampleRate {
		return inputSamples, nil
	}

	// Best so far, slight hump where low pass filter is applied.
	var samples []float64

	srcLength := len(inputSamples)
	destLength := int(float64(srcLength) * float64(toSampleRate) / float64(fromSampleRate))
	dx := float64(srcLength) / float64(destLength)
	//ideally we low pass filters at Nyquist of from sampling rate. reality is it doesn't matter much
	if fromSampleRate < 8800 {
		// Apply filter before resampling to avoid a bump
		//num := float64(toSampleRate) / float64(fromSampleRate)
		// Fmax: Nyquist half of destination sampleRate
		// Fmax / sampleRater = 0.5
		// Apply a low pass before resampling - this has been hand-tested
		myLowPassFilter := foxPEQ.NewPEQFilter(fromSampleRate, 15)

		err := myLowPassFilter.CalcBiquadFilter("lowpass", 22000, 0, 3.2, "Q")
		//myFilter, err := foxPEQ.CalcBiquadFilter("lowpass", 18000, fromSampleRate, 0, 0.3, "Q")
		if err != nil {
			return inputSamples, err
		}
		// the cascade filters needs a slice of coefficients
		//myFilters := make(foxPEQ.CoefficientsSlice, 0)
		//myFilters.Add(myFilter)
		myLowPassFilter.GenerateFilterImpulse()
		myConvolver := NewConvolver(myLowPassFilter.Impulse, 0.5)

		inputSamples = myConvolver.ConvolveImpulsesFFT(inputSamples)
	}

	fmaxDivSR := 0.5
	rG := 2 * fmaxDivSR

	// Quality is half the window width
	quality = 10

	wndWidth2 := quality
	wndWidth := quality * 2

	x := 0.0
	var rY, rW, rA, rSnc float64
	var tau, j int
	for i := 0; i < destLength; i++ {
		rY = 0.0
		for tau = -wndWidth2; tau < wndWidth2; tau++ {
			// Input sample index
			j = int(x + float64(tau))

			// Hann Window. Scale and calculate sinc
			rW = 0.5 - 0.5*math.Cos(2*math.Pi*(0.5+(float64(j)-x)/float64(wndWidth)))
			rA = 2 * math.Pi * (float64(j) - x) * fmaxDivSR
			rSnc = 1.0
			if rA != 0 {
				rSnc = math.Sin(rA) / rA
			}

			if j >= 0 && j < srcLength {
				rY += rG * rW * rSnc * inputSamples[j]
			}
		}
		samples = append(samples, rY)
		x += dx
	}

	return samples, nil
}

// resample all input samples from fromSampleRate toSampleRate
func (myConvolver *Convolver) Resample(inputSamples [][]float64, fromSampleRate, toSampleRate, quality int) error {
	const errorPrefix = packageName + ":" + "resampler"
	if quality == 0 {
		quality = 10
	}
	var err error
	if fromSampleRate == toSampleRate {
		myConvolver.warning(fmt.Sprintf(errorPrefix + "writing header.."))
		return nil

	}

	myChannelsLength := len(inputSamples)
	for c := 0; c < myChannelsLength; c++ {
		// Tried various methods of optimising downsampling, but basically downsampling from 192000 to 44100 is just bad.

		inputSamples[c], err = resampleChannel(inputSamples[c], fromSampleRate, toSampleRate, quality)
		if err != nil {
			return err
		}

		/* Various experiments, non-better than original code
		//upsample
		if fromSampleRate < toSampleRate {
			inputSamples[c], err = resampleChannel(inputSamples[c], fromSampleRate, toSampleRate, quality)
			if err != nil {
				return err
			}
		} else {
			inputSamples[c], err = resampleChannel(inputSamples[c], fromSampleRate, toSampleRate, quality)

			//downsample
			// experiment to upsample to a multiple of both 192000 and 44100
			//inputSamples[c], err = resampleChannel(inputSamples[c], 192000, 28224000, quality)
			if err != nil {
				return err
			}
			// and then downsample back to 44100
			//inputSamples[c] = downsample(inputSamples[c], 28224000, 44100)
			//inputSamples[c], err = resampleChannel(inputSamples[c], 28224000, 44100, quality)
			//inputSamples[c] = resampleLinear(inputSamples[c], fromSampleRate, toSampleRate)
		}

		//inputSamples[c] = resampleLinear(inputSamples[c], fromSampleRate, toSampleRate)
		*/
	}

	return nil
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
