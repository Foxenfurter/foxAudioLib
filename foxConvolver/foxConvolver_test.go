package foxConvolver_test

import (
	"fmt"
	"os"
	"sync"
	"testing"
	"time"

	"github.com/Foxenfurter/foxAudioLib/foxAudioDecoder"
	"github.com/Foxenfurter/foxAudioLib/foxAudioDecoder/foxWavReader"
	"github.com/Foxenfurter/foxAudioLib/foxAudioEncoder" // Import your package
	"github.com/Foxenfurter/foxAudioLib/foxConvolver"
	"github.com/Foxenfurter/foxAudioLib/foxPEQ"
	goDSP "github.com/mjibson/go-dsp/fft"
	Sci "scientificgo.org/fft"
)

func TestFoxFFT(t *testing.T) {
	// Create a list of complex arrays
	inputs := [][]complex128{
		{1 + 1i, 2 + 2i, 3 + 3i, 4 + 4i},
		{1 + 1i, 1 + 1i, 1 + 1i, 1 + 1i},
		{1 + 2i, 2 + 3i, 3 + 4i, 4 + 5i},
	}

	// Expected FFT results
	expected := [][]complex128{
		{10 + 10i, -2 - 2i, -2 - 2i, -2 - 2i},
		{4 + 4i, 0 + 0i, 0 + 0i, 0 + 0i},
		{10 + 14i, -2 - 2i, -2 - 2i, -2 - 2i},
	}
	//fftSize := 4
	//fft := foxConvolver.NewFft64(fftSize)

	for i, original := range inputs {
		println("Testing Fox  FFT")
		fmt.Printf("Input: %v\n", original)
		fmt.Printf("Expected FFT result: %v\n", expected[i])
		// Compute the FFT
		fftResult := foxConvolver.TukeyFFT(original)

		fmt.Printf("Actual FFT result: %v\n", fftResult)
		// Compute the inverse FFT
		ifftResult := foxConvolver.InverseTukeyFFT(fftResult)

		// Compare the original array with the result of the IFFT
		for j, v := range ifftResult {
			// Due to numerical precision issues, we need to round the result
			vRounded := complex(round(real(v)), round(imag(v)))
			if original[j] != vRounded {
				fmt.Printf("Mismatch at index %d: expected %v, got %v\n", j, original[j], vRounded)
			}
		}
		fmt.Println()
	}
}

func TestGoDSPFFT(t *testing.T) {
	// Create a list of complex arrays
	inputs := [][]complex128{
		{1 + 1i, 2 + 2i, 3 + 3i, 4 + 4i},
		{1 + 1i, 1 + 1i, 1 + 1i, 1 + 1i},
		{1 + 2i, 2 + 3i, 3 + 4i, 4 + 5i},
	}

	// Expected FFT results
	expected := [][]complex128{
		{10 + 10i, -2 - 2i, -2 - 2i, -2 - 2i},
		{4 + 4i, 0 + 0i, 0 + 0i, 0 + 0i},
		{10 + 14i, -2 - 2i, -2 - 2i, -2 - 2i},
	}

	for i, original := range inputs {
		println("Testing GoDSP  FFT")
		// Compute the FFT
		//"github.com/mjibson/go-dsp/fft"
		fftResult := goDSP.FFT(original)

		fmt.Printf("Input: %v\n", original)
		fmt.Printf("Expected FFT result: %v\n", expected[i])
		fmt.Printf("Actual FFT result: %v\n", fftResult)

		//"github.com/mjibson/go-dsp/fft"
		ifftResult := goDSP.IFFT(fftResult)

		// Compare the original array with the result of the IFFT
		for j, v := range ifftResult {
			// Due to numerical precision issues, we need to round the result
			vRounded := complex(round(real(v)), round(imag(v)))
			if original[j] != vRounded {
				fmt.Printf("Mismatch at index %d: expected %v, got %v\n", j, original[j], vRounded)
			}
		}
		fmt.Println()
	}
}

func TestScientificFFT(t *testing.T) {
	// Create a list of complex arrays
	inputs := [][]complex128{
		{1 + 1i, 2 + 2i, 3 + 3i, 4 + 4i},
		{1 + 1i, 1 + 1i, 1 + 1i, 1 + 1i},
		{1 + 2i, 2 + 3i, 3 + 4i, 4 + 5i},
	}

	// Expected FFT results
	expected := [][]complex128{
		{10 + 10i, -2 - 2i, -2 - 2i, -2 - 2i},
		{4 + 4i, 0 + 0i, 0 + 0i, 0 + 0i},
		{10 + 14i, -2 - 2i, -2 - 2i, -2 - 2i},
	}

	for i, original := range inputs {
		println("Testing Scientific FFT")
		// Compute the FFT
		//	"scientificgo.org/fft"
		fftResult := Sci.Fft(original, false)

		fmt.Printf("Input: %v\n", original)
		fmt.Printf("Expected FFT result: %v\n", expected[i])
		fmt.Printf("Actual FFT result: %v\n", fftResult)

		// Compute the inverse FFT
		//	"scientificgo.org/fft"
		ifftResult := Sci.Fft(fftResult, true)

		// Compare the original array with the result of the IFFT
		for j, v := range ifftResult {
			// Due to numerical precision issues, we need to round the result
			vRounded := complex(round(real(v)), round(imag(v)))
			if original[j] != vRounded {
				fmt.Printf("Mismatch at index %d: expected %v, got %v\n", j, original[j], vRounded)
			}
		}
		fmt.Println()
	}
}

func round(f float64) float64 {
	if f < 0 {
		return float64(int(f - 0.5))
	}
	return float64(int(f + 0.5))
}

func TestPEQvsSoXImpulse(t *testing.T) {
	println("Testing PEQ vs SoXImpulse")
	sampleRate := 44100
	bitDepth := 24

	dtStartRun := time.Now()

	var err error
	// abit OTT for testing a convolver. But simplistic tests won't cut it.
	// create an filter impulse and then run it vs an imported impules.
	// Export the results
	myPEQFilter := foxPEQ.NewPEQFilter(sampleRate, 15)

	err = myPEQFilter.CalcBiquadFilter("lowshelf", 80, 1.8, 0.41, "Q")
	if err != nil {
		fmt.Printf("Invalid filter definitio, %s \n", err.Error())
	}
	err = myPEQFilter.CalcBiquadFilter("highshelf", 6300, 0.25, 0.41, "Q")
	if err != nil {
		fmt.Printf("Invalid filter definitio, %s \n", err.Error())
	}
	err = myPEQFilter.CalcBiquadFilter("peak", 2400, -1.8, 0.17, "Q")
	if err != nil {
		fmt.Printf("Invalid filter definitio, %s \n", err.Error())
	}
	err = myPEQFilter.CalcBiquadFilter("lowshelf", 35, 0.25, 1.25, "Q")
	if err != nil {
		fmt.Printf("Invalid filter definitio, %s \n", err.Error())
	}
	err = myPEQFilter.CalcBiquadFilter("peak", 1200, 0.25, 1.4, "Q")
	if err != nil {
		fmt.Printf("Invalid filter definitio, %s \n", err.Error())
	}

	fmt.Printf("Number of filters created: %v\n", len(myPEQFilter.FilterCoefficients))
	fmt.Printf("now generate impulses: \n")

	myPEQFilter.GenerateFilterImpulse()
	//TestImpulseResponse = append(TestImpulseResponse, newImpulse)
	fmt.Printf("impulse created - length: %v \n", len(myPEQFilter.Impulse))

	// Now let's load a file
	//Filename: "c:\\temp\\Pencil_1644.wav", // Replace with your test file
	//Filename: "C:\\Users\\jonat\\Music\\Resolution\\Pencil_24192.wav",
	//Filename: "c:\\temp\\Opera_with_Sub_REW_96k.wav",
	//Filename: "c:\\temp\\96000_0069465CE0_Impulses_Cavern4Opera.wav",
	//Filename: "c:\\temp\\CavernLeft.wav", // Replace with your test file
	myFirFile := "c:\\temp\\OperaExperiment-192k.wav"
	//myFirFile := "c:\\temp\\96000_Impulses_Cavern4Iloud.wav"
	//Filename: "c:\\temp\\Loudness_yz.wav", // Replace with your test file
	/* Examples Using Sox as a file resampler*/
	//myFirFile := "c:\\temp\\OperaExperiment-192k.wav"
	myFirReader, err := foxConvolver.ReadnResampleFirFile(myFirFile, sampleRate)
	if err != nil {
		println("Error reading and resampling file: ", err)
		panic(err)
	}
	var mySoXDecoder foxWavReader.WavReader

	mySoXDecoder.Input = myFirReader

	err = mySoXDecoder.DecodeWavHeader()
	if err != nil {
		println("Test: Header Read Error...", err)
		panic(err)
	}

	var WG sync.WaitGroup
	DecodedSamplesChannel := make(chan [][]float64, 80000)
	WG.Add(1)
	go func() {
		defer WG.Done()

		mySoXDecoder.DecodeInput(DecodedSamplesChannel)

	}()

	// Close the doneProcessing channel to signal that decoding is finished
	fmt.Println("Test: Setting up Convolver")
	myConvolver := foxConvolver.NewConvolver(myPEQFilter.Impulse, 0.5)
	var myConvolvedSignal []float64
	//Sox
	//targetLevel := foxConvolver.TargetGain(mySoXDecoder.SampleRate, int(sampleRate))
	targetLevel := foxConvolver.TargetGain(192000, int(sampleRate))

	//Test zeozeozeo resampler
	var copyRawImpulse []float64
	WG.Add(1)
	go func() {
		var ResultCounter int
		defer WG.Done()
		for decodedResult := range DecodedSamplesChannel {
			// Discard decoded results
			// this is where we convolve Dude!

			ResultCounter += len(decodedResult[0])
			fmt.Println("Test: Convolving", ResultCounter)
			//resamples in place
			err = myConvolver.Resample(decodedResult, mySoXDecoder.SampleRate, sampleRate, 0)
			if err != nil {
				println("Error resampling: ", err.Error())
				return
			}

			//_ = foxConvolver.Normalize(decodedResult, targetLevel)

			//myConvolvedSignal = foxConvolver.ConvolveImpulsesSlow(myConvolver.Impulse, decodedResult[0])

			//myConvolvedSignal = foxConvolver.ConvolveImpulsesSimple(decodedResult[0], myConvolver.Impulse)
			//myConvolvedSignal = append(myConvolvedSignal,myConvolver.ConvolveImpulsesFFTOverlap(decodedResult[0])[:]...)
			// Build up the result by appending the samples from the channel
			myConvolvedSignal = append(myConvolvedSignal, myConvolver.ConvolveImpulsesFFT(decodedResult[0])[:]...)
			copyRawImpulse = append(copyRawImpulse, (decodedResult[0])[:]...)

		}
		fmt.Println("Test: Not waiting anymore", ResultCounter)

	}()
	fmt.Println("Test: Are we still waiting")
	// Wait for the asynchronous decoding to complete

	WG.Wait()
	fmt.Println("Test: closing decoder")

	//myDecoder.Close()

	fmt.Println("Setting up Encoder")
	OutputSignal := [][]float64{myConvolvedSignal}
	_ = foxConvolver.Normalize(OutputSignal, targetLevel)
	fmt.Println("calculate data Size")
	dataSize := len(OutputSignal[0]) * len(OutputSignal) * (bitDepth / 8)
	testImpulseFilename := "c:/temp/ConvolvedImpulse2SoX.wav"
	myEncoder := foxAudioEncoder.AudioEncoder{
		Type:        "Wav",
		SampleRate:  sampleRate,
		BitDepth:    bitDepth, // Adjust as needed
		NumChannels: 1,
		Size:        int64(dataSize), // Size for a 5-second file (adjust as needed)
		Filename:    testImpulseFilename,
	}
	fmt.Println("Test: Creating new Encoder...")
	// Create encoder
	err = myEncoder.Initialise()
	if err != nil {
		fmt.Println("Test: panic on New Header...")
		panic(err)
	}

	err = myEncoder.EncodeData(OutputSignal)
	if err != nil {
		fmt.Println("Test: panic on New Encoder...")
		panic(err)
	}

	// now output the loaded impulse pre-convolution
	testImpulseFilename = "c://temp//ConvolverRawImpulseSoX.wav"

	myEncoder.Filename = testImpulseFilename
	OutputSignal = [][]float64{copyRawImpulse}
	dataSize = len(OutputSignal[0]) * len(OutputSignal) * (bitDepth / 8)
	myEncoder.Size = int64(dataSize)
	err = myEncoder.Initialise()
	if err != nil {
		fmt.Println("Test: panic on New Header...")
		panic(err)
	}

	myEncoder.EncodeData(OutputSignal)

	fmt.Printf("Wav  created: %s %v %v \n", testImpulseFilename, sampleRate, bitDepth)

	// Now check the convolver Impulse is OK
	testImpulseFilename = "c://temp//ConvolverPEQImpulseSoX.wav"

	myEncoder.Filename = testImpulseFilename
	OutputSignal = [][]float64{myConvolver.Impulse}
	dataSize = len(OutputSignal[0]) * len(OutputSignal) * (bitDepth / 8)
	myEncoder.Size = int64(dataSize)
	err = myEncoder.Initialise()
	if err != nil {
		fmt.Println("Test: panic on New Header...")
		panic(err)
	}

	myEncoder.EncodeData(OutputSignal)

	fmt.Printf("Wav  created: %s %v %v \n", testImpulseFilename, sampleRate, bitDepth)

	// Now output PEQ used as reference
	testImpulseFilename = "c://temp//PEQImpulseSoX.wav"
	myEncoder.Filename = testImpulseFilename
	OutputSignal = [][]float64{myPEQFilter.Impulse}
	dataSize = len(OutputSignal[0]) * len(OutputSignal) * (bitDepth / 8)
	myEncoder.Size = int64(dataSize)
	err = myEncoder.Initialise()
	if err != nil {
		fmt.Println("Test: panic on New Header...")
		panic(err)
	}

	myEncoder.EncodeData(OutputSignal)

	fmt.Printf("Wav  created: %s %v %v \n", testImpulseFilename, sampleRate, bitDepth)

	fmt.Printf("Time taken: %s\n", time.Since(dtStartRun))

}

func TestPEQConvolveVsImpulse(t *testing.T) {
	println("Testing PEQ Convolve Vs Impulese")
	sampleRate := 192000
	bitDepth := 24

	dtStartRun := time.Now()

	var err error
	// abit OTT for testing a convolver. But simplistic tests won't cut it.
	// create an filter impulse and then run it vs an imported impules.
	// Export the results
	myPEQFilter := foxPEQ.NewPEQFilter(sampleRate, 15)

	err = myPEQFilter.CalcBiquadFilter("lowshelf", 80, 1.8, 0.41, "Q")
	if err != nil {
		fmt.Printf("Invalid filter definitio, %s \n", err.Error())
	}
	err = myPEQFilter.CalcBiquadFilter("highshelf", 6300, 0.25, 0.41, "Q")
	if err != nil {
		fmt.Printf("Invalid filter definitio, %s \n", err.Error())
	}
	err = myPEQFilter.CalcBiquadFilter("peak", 2400, -1.8, 0.17, "Q")
	if err != nil {
		fmt.Printf("Invalid filter definitio, %s \n", err.Error())
	}
	err = myPEQFilter.CalcBiquadFilter("lowshelf", 35, 0.25, 1.25, "Q")
	if err != nil {
		fmt.Printf("Invalid filter definitio, %s \n", err.Error())
	}
	err = myPEQFilter.CalcBiquadFilter("peak", 1200, 0.25, 1.4, "Q")
	if err != nil {
		fmt.Printf("Invalid filter definitio, %s \n", err.Error())
	}

	fmt.Printf("Number of filters created: %v\n", len(myPEQFilter.FilterCoefficients))
	fmt.Printf("now generate impulses: \n")

	myPEQFilter.GenerateFilterImpulse()
	//TestImpulseResponse = append(TestImpulseResponse, newImpulse)
	fmt.Printf("impulse created - length: %v \n", len(myPEQFilter.Impulse))

	/* Examples using internal Resampler*/

	myInternalDecoder := &foxAudioDecoder.AudioDecoder{
		//Filename: "c:\\temp\\Pencil_1644.wav", // Replace with your test file
		//Filename: "C:\\Users\\jonat\\Music\\Resolution\\Pencil_1644.wav",
		Filename: "C:\\Users\\jonat\\Music\\Resolution\\Pencil_24192.wav",
		//Filename: "c:\\temp\\Opera_with_Sub_REW_96k.wav",
		//Filename: "c:\\temp\\96000_0069465CE0_Impulses_Cavern4Opera.wav",
		//Filename: "c:\\temp\\CavernLeft.wav", // Replace with your test file
		//Filename: "c:\\temp\\OperaExperiment-192k.wav",
		//Filename: "c:\\temp\\96000_Impulses_Cavern4Iloud.wav",
		//Filename: "c:\\temp\\Loudness_yz.wav", // Replace with your test file

		Type: "Wav",
	}

	fmt.Println("Test: Decoding input file... ", myInternalDecoder.Filename)
	err = myInternalDecoder.Initialise()
	if err != nil {
		t.Fatalf("Test: Error initializing AudioDecoder: %v", err)
	}

	var WG sync.WaitGroup
	DecodedSamplesChannel := make(chan [][]float64, 10)
	WG.Add(1)
	go func() {
		defer WG.Done()

		myInternalDecoder.DecodeSamples(DecodedSamplesChannel, nil)
	}()

	// Close the doneProcessing channel to signal that decoding is finished
	fmt.Println("Test: Setting up Convolver")
	myConvolver := foxConvolver.NewConvolver(myPEQFilter.Impulse, 0.5)
	myConvolvedSignal := make([]float64, 0)
	//Internal
	targetLevel := foxConvolver.TargetGain(myInternalDecoder.SampleRate, int(sampleRate))

	//Test zeozeozeo resampler
	if err != nil {
		panic(err)
	}
	var copyRawImpulse []float64
	WG.Add(1)
	go func() {
		var ResultCounter int
		var overlaptail []complex128 //[]float64
		var tempConv []float64
		defer WG.Done()
		for decodedResult := range DecodedSamplesChannel {
			// Discard decoded results
			// this is where we convolve Dude!

			ResultCounter += len(decodedResult[0])
			//fmt.Println("Test: Convolving", ResultCounter)
			//resamples in place

			err = myConvolver.Resample(decodedResult, myInternalDecoder.SampleRate, sampleRate, 0)
			if err != nil {
				println("Error resampling: ", err.Error())
				return
			}

			//myConvolvedSignal = foxConvolver.ConvolveImpulsesSlow(myConvolver.Impulse, decodedResult[0])

			//myConvolvedSignal = foxConvolver.ConvolveImpulsesSimple(decodedResult[0], myConvolver.Impulse)

			// Build up the result by appending the samples from the channel
			//myConvolvedSignal = append(myConvolvedSignal, myConvolver.ConvolveImpulsesFFT(decodedResult[0])[:]...)

			//seems to be working
			tempConv, overlaptail = foxConvolver.ConvolveImpulseOverlapSave(myConvolver.Impulse, decodedResult[0], overlaptail)

			myConvolvedSignal = append(myConvolvedSignal, tempConv...)
			copyRawImpulse = append(copyRawImpulse, (decodedResult[0])[:]...)

		}
		// if there is any tail left

		if len(overlaptail) > 0 {
			//myConvolvedSignal = append(myConvolvedSignal, myConvolver.ConvolveImpulsesFFT(overlaptail)[:]...)
			//tempConv, _ = foxConvolver.ConvolveImpulsesOverlapAdd(myConvolver.Impulse, overlaptail, nil)
			//myConvolvedSignal = append(myConvolvedSignal, tempConv...)

		}

		fmt.Println("Test: Not waiting anymore", ResultCounter)

	}()
	fmt.Println("Test: Are we still waiting")
	// Wait for the asynchronous decoding to complete

	WG.Wait()
	fmt.Println("Test: closing decoder")

	//myDecoder.Close()

	fmt.Println("Setting up Encoder: ", targetLevel)
	OutputSignal := [][]float64{myConvolvedSignal}
	//Normalisation here lowers the level of 24 bit and drops a bit from output
	//_ = foxConvolver.Normalize(OutputSignal, targetLevel)
	fmt.Println("calculate data Size")
	dataSize := len(OutputSignal[0]) * len(OutputSignal) * (bitDepth / 8)
	testImpulseFilename := "c:/temp/ConvolvedImpulse2.wav"
	myEncoder := foxAudioEncoder.AudioEncoder{
		Type:        "Wav",
		SampleRate:  sampleRate,
		BitDepth:    bitDepth, // Adjust as needed
		NumChannels: 1,
		Size:        int64(dataSize), // Size for a 5-second file (adjust as needed)
		Filename:    testImpulseFilename,
	}
	fmt.Println("Test: Creating new Encoder...")
	// Create encoder
	err = myEncoder.Initialise()
	if err != nil {
		fmt.Println("Test: panic on New Header...")
		panic(err)
	}

	err = myEncoder.EncodeData(OutputSignal)
	if err != nil {
		fmt.Println("Test: panic on New Encoder...")
		panic(err)
	}

	// now output the loaded impulse pre-convolution
	testImpulseFilename = "c://temp//ConvolverRawImpulse.wav"

	myEncoder.Filename = testImpulseFilename
	OutputSignal = [][]float64{copyRawImpulse}
	dataSize = len(OutputSignal[0]) * len(OutputSignal) * (bitDepth / 8)
	myEncoder.Size = int64(dataSize)
	err = myEncoder.Initialise()
	if err != nil {
		fmt.Println("Test: panic on New Header...")
		panic(err)
	}

	myEncoder.EncodeData(OutputSignal)

	fmt.Printf("Wav  created: %s %v %v \n", testImpulseFilename, sampleRate, bitDepth)

	// Now check the convolver Impulse is OK
	testImpulseFilename = "c://temp//ConvolverPEQImpulse.wav"

	myEncoder.Filename = testImpulseFilename
	OutputSignal = [][]float64{myConvolver.Impulse}
	dataSize = len(OutputSignal[0]) * len(OutputSignal) * (bitDepth / 8)
	myEncoder.Size = int64(dataSize)
	err = myEncoder.Initialise()
	if err != nil {
		fmt.Println("Test: panic on New Header...")
		panic(err)
	}

	myEncoder.EncodeData(OutputSignal)

	fmt.Printf("Wav  created: %s %v %v \n", testImpulseFilename, sampleRate, bitDepth)

	// Now output PEQ used as reference
	testImpulseFilename = "c://temp//PEQImpulse.wav"
	myEncoder.Filename = testImpulseFilename
	OutputSignal = [][]float64{myPEQFilter.Impulse}
	dataSize = len(OutputSignal[0]) * len(OutputSignal) * (bitDepth / 8)
	myEncoder.Size = int64(dataSize)
	err = myEncoder.Initialise()
	if err != nil {
		fmt.Println("Test: panic on New Header...")
		panic(err)
	}

	myEncoder.EncodeData(OutputSignal)

	fmt.Printf("Wav  created: %s %v %v \n", testImpulseFilename, sampleRate, bitDepth)

	fmt.Printf("Time taken: %s\n", time.Since(dtStartRun))
}

func TestMain(m *testing.M) {
	// Run tests
	fmt.Println("Running DSP Test:")
	exitCode := m.Run()

	// Exit with the test result code
	os.Exit(exitCode)
}
