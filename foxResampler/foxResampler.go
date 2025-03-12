// Package: github.com/Foxenfurter/foxAudioLib/foxConvolver
// filename foxConvolver.go
// Package is designed to manage convolution of Filter Impulse Responses
// includes any logic neeeded to prepare the filter for convolution, such as Resampling and Normalization
package foxResampler

import (
	"bytes"
	"errors"
	"fmt"
	"math"
	"os/exec"
	"strconv"

	"github.com/Foxenfurter/foxAudioLib/foxConvolver"
	"scientificgo.org/fft"
	//	"github.com/mjibson/go-dsp/fft"
)

type Resampler struct {
	InputSamples   [][]float64
	FromSampleRate int
	ToSampleRate   int
	Quality        int
	//streaming         bool
	DebugOn     bool //enables debugging
	DebugFunc   func(string)
	WarningFunc func(string)
}

const packageName = "foxResampler"

// Init initializes the convolver structure
// supply FIR filter impulse and overlap factor as a value between 0.1 and 0.9
func NewResampler() *Resampler {
	newResampler := Resampler{
		//FilterImpulse: impulse,
		Quality: 0,
	}

	return &newResampler
}

// Function to handle debug calls, allowing for different logging implementations

func (myResampler *Resampler) debug(message string) {
	if myResampler.DebugOn {
		if myResampler.DebugFunc != nil {
			myResampler.DebugFunc(message)
		} else { // if no external debug function available just print the message
			println(message)
		}
	}
}

func (myResampler *Resampler) warning(message string) {
	if myResampler.WarningFunc != nil {
		myResampler.WarningFunc(message)
	}
}

// In case you want to use sox as an input.
func ReadnResampleFirFile(filePath string, targetSampleRate int) (*bytes.Reader, error) {
	functionName := "ReadnResampleFirFile"
	mySampleRate := strconv.Itoa(targetSampleRate)
	cmd := exec.Command("sox", filePath, "-r", mySampleRate, "-t", "wav", "-q", "-")
	// Create a bytes.Buffer to capture the output
	var out bytes.Buffer
	cmd.Stdout = &out

	// Run the command
	err := cmd.Run()
	if err != nil {
		ErrorText := packageName + ":" + functionName + " Error running sox command: " + err.Error()
		return nil, errors.New(ErrorText)
	}

	// The output is now in the 'out' buffer.
	// You can convert it to a bytes.Reader with bytes.NewReader()
	myReader := bytes.NewReader(out.Bytes())
	return myReader, nil

}

func ResamplerWorking96k(inputSamples []float64, fromSampleRate, toSampleRate int) ([]float64, error) {
	if fromSampleRate == toSampleRate {
		return inputSamples, nil
	}

	srcLength := len(inputSamples)
	destLength := int(float64(srcLength) * float64(toSampleRate) / float64(fromSampleRate))
	output := make([]float64, destLength)

	// Downsampling (frequency-domain filtering)
	if toSampleRate < int(float64(fromSampleRate)*1.1) {
		nyquist := float64(toSampleRate) * 0.5
		nyquistBin := int(nyquist / float64(fromSampleRate) * float64(srcLength))

		complexSamples := make([]complex128, srcLength)
		for i := range inputSamples {
			complexSamples[i] = complex(float64(inputSamples[i]), 0)
		}
		fftResult := fft.Fft(complexSamples, false)
		//fftResult := fft.FFT(complexSamples)
		// Zero out high frequencies
		for i := nyquistBin + 1; i < srcLength-nyquistBin; i++ {
			fftResult[i] = 0
		}

		ifftResult := fft.Fft(fftResult, true)
		//ifftResult := fft.IFFT(fftResult)
		for i := range ifftResult {
			inputSamples[i] = real(ifftResult[i])
		}

		srcLength = len(inputSamples)
		destLength = int(float64(srcLength) * float64(toSampleRate) / float64(fromSampleRate))
		output = make([]float64, destLength)
	}

	dx := float64(srcLength) / float64(destLength)

	// Upsampling (linear interpolation) or Downsampling (after filter)
	for i := 0; i < destLength; i++ {
		x := float64(i) * dx
		x0 := int(math.Floor(x))
		x1 := int(math.Ceil(x))

		if x0 < 0 {
			x0 = 0
		}
		if x1 >= srcLength {
			x1 = srcLength - 1
		}

		if x0 == x1 {
			output[i] = inputSamples[x0]
		} else {
			y0 := inputSamples[x0]
			y1 := inputSamples[x1]
			output[i] = y0 + (y1-y0)*(x-float64(x0))
		}
	}

	return output, nil
}

// So far the best solution for internal resampling, although it works well for upsampling and poorly for downsampling
func ResampleChannel(inputSamples []float64, fromSampleRate, toSampleRate, quality int) ([]float64, error) {
	// If no resampling required
	if fromSampleRate == toSampleRate {
		return inputSamples, nil
	}

	// Best so far, slight hump where low pass filter is applied.
	var samples []float64

	srcLength := len(inputSamples)
	destLength := int(float64(srcLength) * float64(toSampleRate) / float64(fromSampleRate))
	dx := float64(srcLength) / float64(destLength)

	const fmaxDivSR = 0.5
	const rG = 2 * fmaxDivSR

	// Quality is half the window width
	//NB a higher quality e.g 100 will result in a flatter filter , at the cost of a lot of performance.

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

	// Theoretically apply filter before resampling to avoid a bump, however seems to work better after resampling.
	//num := float64(toSampleRate) / float64(fromSampleRate)
	// Fmax: Nyquist half of destination sampleRate
	// Fmax / sampleRater = 0.5
	// Apply a low pass before resampling - this has been hand-tested
	var Fp = 21000.0 // End of pass-band - below this frequency everything should pass
	var Fs = 22000.0 // Start of stop-band - above this frequency everything should be blocked
	//var Fn = float64(sampleRate) / 2.0 // Nyquist frequency of original Sample Rate
	var att = 60.0 // Stop-band attenuation in dB - not sure if it makes much difference

	var k = 0       // Number of phases (not used in this example)
	var beta = -1.0 // Value will be estimated
	myLowPassImpulse := designLpf(Fp, Fs, float64(toSampleRate), att, k, beta)
	//myLowPassFilter := foxPEQ.NewPEQFilter(fromSampleRate, 15)
	myTConvolver := foxConvolver.NewConvolver(myLowPassImpulse)
	samples = myTConvolver.ConvolveFFT(samples)

	return samples, nil
}

// resample all input samples from fromSampleRate toSampleRate
func (myResampler *Resampler) Resample() error {
	const errorPrefix = packageName + ":" + "resampler"
	if myResampler.Quality == 0 {
		myResampler.Quality = 10
	}
	var err error
	if myResampler.FromSampleRate == myResampler.ToSampleRate {
		myResampler.debug(fmt.Sprintf(errorPrefix + "writing header.."))
		return nil

	}

	myChannelsLength := len(myResampler.InputSamples)
	for c := 0; c < myChannelsLength; c++ {
		// Tried various methods of optimising downsampling, but basically downsampling from 192000 to 44100 is just bad.
		// Various experiments, non-better than original code
		if myResampler.FromSampleRate < myResampler.ToSampleRate {
			//better for upsampling
			myResampler.debug(fmt.Sprintf(errorPrefix + "upsampling"))
			myResampler.InputSamples[c], err = ResampleChannel(myResampler.InputSamples[c], myResampler.FromSampleRate, myResampler.ToSampleRate, myResampler.Quality)
		} else {
			//better for downsampling?
			//	myResampler.debug(fmt.Sprintf(errorPrefix + "downsampling"))
			//myResampler.InputSamples[c], err = resampleDown(myResampler.InputSamples[c], myResampler.FromSampleRate, myResampler.ToSampleRate)
			//myResampler.InputSamples[c], err = downSample(myResampler.InputSamples[c], myResampler.FromSampleRate, myResampler.ToSampleRate)
			myResampler.InputSamples[c], err = ResamplerWorking96k(myResampler.InputSamples[c], myResampler.FromSampleRate, myResampler.ToSampleRate)
			//myResampler.InputSamples[c], err = resampleDownold(myResampler.InputSamples[c], myResampler.FromSampleRate, myResampler.ToSampleRate)
		}
		if err != nil {
			return err
		}
	}

	return nil
}

// The following code is a low pass filter adapted from The SoX Resampler library `libsoxr'
//https://github.com/chirlu/soxr/blob/master/src/filter.c

const (
	PI = math.Pi
)

// Fp End of pass-band
// Fs Start of stop-band
// Fn Nyquist freq; e.g. 0.5, 1, PI; < 0: dummy run
// att Stop-band attenuation in dB *
// numTaps: number of samples in filter
// k  >0: number of phases; <0: num_taps = 1 (mod -k)
// double beta <0: value will be estimated
// returns an impulse of type []float64
func designLpf(Fp, Fs, sampleRate, att float64, k int, beta float64) []float64 {
	// Normalize Fp and Fs to the Nyquist frequency
	Fn := sampleRate / 2.0
	Fp /= Fn
	Fs /= Fn

	numTaps := int(sampleRate / 8.0) // Number of samples in filter which is roughly sampleRate / ( lowest filter frequency / 2)

	phases := max(k, 1)
	// transition bandwidth
	trBw := 0.5 * (Fs - Fp)
	trBw /= float64(phases)
	Fs /= float64(phases)
	trBw = math.Min(trBw, 0.5*Fs)
	Fc := Fs - trBw
	var rho float64
	if phases == 1 {
		rho = 0.5
	} else if att < 120 {
		rho = 0.63
	} else {
		rho = 0.75
	}

	if Fc-trBw < 0 {
		panic("Invalid Fc and trBw combination")
	}

	kaiserParams(att, Fc, trBw, &beta)

	if Fn < 0 {
		return nil
	}
	return makeLpf(numTaps, Fc, beta, rho, float64(phases))

}

func kaiserParams(att, Fc, trBw float64, beta *float64) {
	if *beta < 0 {
		*beta = kaiserBeta(att, trBw*0.5/Fc)
	}
	if att < 60 {
		// not necessary, but errors without it.
		att = (att - 7.95) / (2.285 * PI * 2)
	} else {
		tempBeta := *beta
		tempBeta = math.Pow(0.0007528358-1.577737e-05*tempBeta, tempBeta) + 0.6248022 + 0.06186902
		*beta = tempBeta
	}

}

func kaiserBeta(att, trBw float64) float64 {
	if att >= 60 {
		coefs := [][]float64{
			{-6.784957e-10, 1.02856e-05, 0.1087556, -0.8988365 + 0.001},
			{-6.897885e-10, 1.027433e-05, 0.10876, -0.8994658 + 0.002},
			{-1.000683e-09, 1.030092e-05, 0.1087677, -0.9007898 + 0.003},
			{-3.654474e-10, 1.040631e-05, 0.1087085, -0.8977766 + 0.006},
			{8.106988e-09, 6.983091e-06, 0.1091387, -0.9172048 + 0.015},
			{9.519571e-09, 7.272678e-06, 0.1090068, -0.9140768 + 0.025},
			{-5.626821e-09, 1.342186e-05, 0.1083999, -0.9065452 + 0.05},
			{-9.965946e-08, 5.073548e-05, 0.1040967, -0.7672778 + 0.085},
			{1.604808e-07, -5.856462e-05, 0.1185998, -1.34824 + 0.1},
			{-1.511964e-07, 6.363034e-05, 0.1064627, -0.9876665 + 0.18},
		}
		realm := math.Log(trBw/0.0005) / math.Log(2.0)
		c0 := coefs[rangeLimit(int(realm), 0, len(coefs)-1)]
		c1 := coefs[rangeLimit(1+int(realm), 0, len(coefs)-1)]
		b0 := ((c0[0]*att+c0[1])*att+c0[2])*att + c0[3]
		b1 := ((c1[0]*att+c1[1])*att+c1[2])*att + c1[3]
		return b0 + (b1-b0)*(realm-float64(int(realm)))
	}
	if att > 50 {
		return 0.1102 * (att - 8.7)
	}
	if att > 20.96 {
		return 0.58417*math.Pow(att-20.96, 0.4) + 0.07886*(att-20.96)
	}
	return 0
}

// builds the low pass filter returns an impulse of type []float64 with length numTaps
func makeLpf(numTaps int, Fc, beta, rho, scale float64) []float64 {
	m := numTaps - 1
	h := make([]float64, numTaps)
	mult := scale / besselI0(beta)
	mult1 := 1.0 / (0.5*float64(m) + rho)

	if Fc < 0 || Fc > 1 {
		panic("Fc should be between 0 and 1")
	}

	for i := 0; i <= m/2; i++ {
		z := float64(i) - 0.5*float64(m)
		x := z * PI
		y := z * mult1
		if x != 0 {
			h[i] = math.Sin(Fc*x) / x
		} else {
			h[i] = Fc
		}
		h[i] *= besselI0(beta*math.Sqrt(1-y*y)) * mult
		if m-i != i {
			h[m-i] = h[i]
		}
	}
	return h
}

func besselI0(x float64) float64 {
	var k int
	var w, t, y float64
	a := []float64{
		8.5246820682016865877e-11, 2.5966600546497407288e-9,
		7.9689994568640180274e-8, 1.9906710409667748239e-6,
		4.0312469446528002532e-5, 6.4499871606224265421e-4,
		0.0079012345761930579108, 0.071111111109207045212,
		0.444444444444724909, 1.7777777777777532045,
		4.0000000000000011182, 3.99999999999999998,
		1.0000000000000000001,
	}

	b := []float64{
		6.7852367144945531383e-8, 4.6266061382821826854e-7,
		6.9703135812354071774e-6, 7.6637663462953234134e-5,
		7.9113515222612691636e-4, 0.0073401204731103808981,
		0.060677114958668837046, 0.43994941411651569622,
		2.7420017097661750609, 14.289661921740860534,
		59.820609640320710779, 188.78998681199150629,
		399.8731367825601118, 427.56411572180478514,
	}

	c := []float64{
		2.5568678676452702768e-15, 3.0393953792305924324e-14,
		6.3343751991094840009e-13, 1.5041298011833009649e-11,
		4.4569436918556541414e-10, 1.746393051427167951e-8,
		1.0059224011079852317e-6, 1.0729838945088577089e-4,
		0.05150322693642527738,
	}

	w = math.Abs(x)
	if w < 8.5 {
		t = w * w * 0.0625
		k = 13 * int(t)
		y = (((((((((((a[k]*t+a[k+1])*t+
			a[k+2])*t+a[k+3])*t+a[k+4])*t+
			a[k+5])*t+a[k+6])*t+a[k+7])*t+
			a[k+8])*t+a[k+9])*t+a[k+10])*t+
			a[k+11])*t + a[k+12]
	} else if w < 12.5 {
		k = int(w)
		t = w - float64(k)
		k = 14 * (k - 8)
		y = ((((((((((((b[k]*t+b[k+1])*t+
			b[k+2])*t+b[k+3])*t+b[k+4])*t+
			b[k+5])*t+b[k+6])*t+b[k+7])*t+
			b[k+8])*t+b[k+9])*t+b[k+10])*t+
			b[k+11])*t+b[k+12])*t + b[k+13]
	} else {
		t = 60 / w
		k = 9 * int(t)
		y = ((((((((c[k]*t+c[k+1])*t+
			c[k+2])*t+c[k+3])*t+c[k+4])*t+
			c[k+5])*t+c[k+6])*t+c[k+7])*t +
			c[k+8]) * math.Sqrt(t) * math.Exp(w)
	}
	return y
}

// Compares the value of an int to a lower and higher value and returns the floor or ceiling value
// or if the value is within the bounds returns the original value
func rangeLimit(value, min, max int) int {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}
