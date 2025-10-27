package main

import (
	"fmt"
	"sync"
	"time"

	"github.com/Foxenfurter/foxAudioLib/foxAudioEncoder"
	"github.com/Foxenfurter/foxAudioLib/foxConvolver"
	"github.com/Foxenfurter/foxAudioLib/foxNormalizer"
	"github.com/Foxenfurter/foxAudioLib/foxPEQ"
	"github.com/Foxenfurter/foxAudioLib/foxResampler"

	"github.com/Foxenfurter/foxAudioLib/foxAudioDecoder"
)

func main() {
	const functionName = "Main"
	fmt.Printf("Testing something foxAudioLib \n")
	startTime := time.Now()

	// Initialize AudioDecoder
	myDecoder := foxAudioDecoder.AudioDecoder{

		Type: "WAV",
	}
	myDecoder.Filename = "" // Replace with your test file
	myDecoder.Filename = "C:\\Users\\jonat\\Music\\Resolution\\Pencil_1644.wav"
	//myDecoder.Filename = "C:\\Users\\jonat\\Music\\Resolution\\Pencil_24192.wav"

	err := myDecoder.Initialise()
	if err != nil {
		fmt.Println(functionName+": Decoding input file... ", myDecoder.Filename)
	}

	// Initialise Audio Encoder
	myEncoder := foxAudioEncoder.AudioEncoder{
		Type:        "Wav",
		SampleRate:  myDecoder.SampleRate,
		BitDepth:    myDecoder.BitDepth, // Adjust as needed
		NumChannels: myDecoder.NumChannels,
		Size:        int64(myDecoder.Size), // Size for a 5-second file (adjust as needed)
		Filename:    "c:\\temp\\output.wav",
	}

	fmt.Println("Test: Output file: ", myEncoder.Filename)
	fmt.Println("Test: Creating new Encoder SampleRate: ", myEncoder.SampleRate, " Channels: ", myEncoder.NumChannels, " BitDepth: ", myEncoder.BitDepth)
	// Create encoder
	err = myEncoder.Initialise()
	if err != nil {
		fmt.Println("Test: panic on New Header...")
		panic(err)
	}
	// Input and output setup and ready to go...

	// Potentially we want to do this on a per channel basis, like this...
	var applyPEQ = true

	myPEQFilters := make([]foxPEQ.PEQFilter, myDecoder.NumChannels)
	for i := 0; i < myDecoder.NumChannels; i++ {
		myPEQ := foxPEQ.NewPEQFilter(myDecoder.SampleRate, 15)

		// This will need amending to read in or apply from config.
		if applyPEQ {
			err = BuildPEQFilters(&myPEQ)
			if err != nil {
				println("Error building PEQ filters - ", err.Error())
				return
			}
		}
		myPEQFilters[i] = myPEQ

	}
	for i := 0; i < len(myPEQFilters); i++ {
		println("Convolver PEQ filter length:", len(myPEQFilters[i].Impulse))
	}
	elapsedTime := int(time.Since(startTime).Milliseconds())

	println("")
	println("============================================================================================")
	println("AudioLib: PEQ Setup Took: ", elapsedTime)
	println("============================================================================================")
	println("")
	//firFile := "c:\\temp\\CavernLeft.wav"
	//firFile := "c:\\temp\\Cavern4IloudNorm.wav"
	firFile := "c:\\temp\\44100_00F12EE86F_Impulses_Cavern4Iloud.wav"
	//firFile := "c:\\temp\\Opera_with_Sub_REW_96k.wav"
	//firFile := ""
	// No Merge needed without a FIR File
	myConvolvers := make([]foxConvolver.Convolver, myDecoder.NumChannels)
	if firFile != "" {
		myImpulseDecoder := &foxAudioDecoder.AudioDecoder{
			Filename: firFile,
			Type:     "Wav",
		}

		err = myImpulseDecoder.Initialise()
		if err != nil {
			println("Test: Error initializing AudioDecoder: %v", err)
		}

		// Load and Merge Impulse File with PEQ Impulse
		myConvolvers = MergePEQandSingleFIRImpulse(&myPEQFilters, myImpulseDecoder, myDecoder.SampleRate)
		// **** INIT DONE - START Processing ***
	} else {
		println("No FIR Filter - mapping PEQ")
		for i := 0; i < len(myConvolvers); i++ {
			myConvolvers[i].FilterImpulse = myPEQFilters[i].Impulse
		}
	}
	incrementalTime := int(time.Since(startTime).Milliseconds()) - elapsedTime
	elapsedTime = int(time.Since(startTime).Milliseconds())

	println("")
	println("============================================================================================")
	println("AudioLib: Setup Took: ", elapsedTime, " FIR Load & Merge: ", incrementalTime)
	println("============================================================================================")
	println("")
	// overall wait group
	var WG sync.WaitGroup

	DecodedSamplesChannel := make(chan [][]float64, 10000)

	fmt.Println("Test: Decoding Data...")
	WG.Add(1)
	go func() {
		defer WG.Done()
		myDecoder.DecodeSamples(DecodedSamplesChannel)

	}()

	// Create go channel for each audio channel
	audioChannels := make([]chan []float64, myDecoder.NumChannels)

	for i := 0; i < myDecoder.NumChannels; i++ {

		audioChannels[i] = make(chan []float64)
	}
	fmt.Println("Splitting Channels...")
	// Split audio data into separate channels
	channelSplitter(DecodedSamplesChannel, audioChannels, myDecoder.NumChannels)

	// Apply convolution (FIR filter)
	convolvedChannels := make([]chan []float64, myDecoder.NumChannels)
	fmt.Println("Convolve Channels...")
	for i := 0; i < myDecoder.NumChannels; i++ {

		convolvedChannels[i] = make(chan []float64)
		go applyConvolution(audioChannels[i], convolvedChannels[i], myConvolvers[i].FilterImpulse)
	}

	mergedChannel := make(chan [][]float64)
	fmt.Println("Merge Channels...")
	go mergeChannels(convolvedChannels, mergedChannel, myDecoder.NumChannels)

	fmt.Println("Test: Encoding Data...")
	WG.Add(1)
	go func() {

		defer WG.Done()
		//myEncoder.AccumulateAndEncodeChannel(DecodedSamplesChannel, ThrottleLoaderChannel, 40000)
		err := myEncoder.EncodeSamplesChannel(mergedChannel)
		if err != nil {
			println("Error encoding samples: ", err)
		}

	}()

	fmt.Println("Test: Waiting...")
	WG.Wait()
	incrementalTime = int(time.Since(startTime).Milliseconds()) - elapsedTime
	elapsedTime = int(time.Since(startTime).Milliseconds())
	println("")
	println("============================================================================================")
	println("AudioLib: total elapsed time: ", elapsedTime, " Signal Processing: ", incrementalTime)
	println("============================================================================================")
	println("")
	//myDecoder.Close()

}

func BuildPEQFilters(myPEQFilter *foxPEQ.PEQFilter) error {

	var err error
	// abit OTT for testing a convolver. But simplistic tests won't cut it.
	// create an filter impulse and then run it vs an imported impules.
	// Export the results

	err = myPEQFilter.CalcBiquadFilter("lowshelf", 80, 1.8, 0.41, "Q")
	if err != nil {
		return err
	}
	err = myPEQFilter.CalcBiquadFilter("highshelf", 6300, 0.25, 0.41, "Q")
	if err != nil {
		return err
	}
	err = myPEQFilter.CalcBiquadFilter("peak", 2400, -1.8, 0.17, "Q")
	if err != nil {
		return err
	}
	err = myPEQFilter.CalcBiquadFilter("lowshelf", 35, 0.25, 1.25, "Q")
	if err != nil {
		return err
	}
	err = myPEQFilter.CalcBiquadFilter("peak", 1200, 0.25, 1.4, "Q")
	if err != nil {
		return err
	}

	myPEQFilter.GenerateFilterImpulse()

	return nil
}

// This needs a lot of work.
// first we don't know whether we actually have a PEQ Filter
// Then we have to handle whether we have a FIR filter
// We need to resample FIR filter to the signal sampling rate.
// If we have a single PEQ filter then we need to copy it.
// If we have a single FIR filter then we need to copy it.
// Convolve PEQ filter & FIR Filter together.
// If PEQFilter.Channels > FIRFilter.Channels keep additional PEQFiler.Channels and only convolve matching.
// If PEQFilter.Channels < FIRFilter.Channels keep additional FIRFilter.Channels and only convolve matching.
// Now we have MergedFilters
// We need to match the number of channels based against the signal
// If Signal.Channels = 1 then only  MergedFilters[0]
// If Signal.Channels > 1 && MergedFilters.Channels == 1 then copy MergedFilters
// IF MergedFilters.Channels > 1 then match MergedFilters.Channels to Signal.Channels - stop at min(MergedFilters.Channels, Signal.Channels)
// This logic should handle a single FIR file either mono or stereo. Multiple FIR files will need to wait.
func MergePEQandSingleFIRImpulse(lclPEQFilters *[]foxPEQ.PEQFilter, myImpulseDecoder *foxAudioDecoder.AudioDecoder, TargetSampleRate int) []foxConvolver.Convolver {
	startTime := time.Now()
	//tmpConvolver := foxConvolver.NewConvolver(myPEQFilter.Impulse)
	targetLevel := foxNormalizer.TargetGain(myImpulseDecoder.SampleRate, TargetSampleRate, 0.89)
	myConvolvers := make([]foxConvolver.Convolver, len(*lclPEQFilters))
	myConvolvedSignal := make([][]float64, len(*lclPEQFilters))
	DecodedImpulseChannel := make(chan [][]float64, 1000)
	var WG sync.WaitGroup
	WG.Add(1)
	go func() {
		defer WG.Done()
		myImpulseDecoder.DecodeSamples(DecodedImpulseChannel)
	}()

	WG.Add(1)
	go func() {

		defer WG.Done()
		myFirImpulse := make([][]float64, myImpulseDecoder.NumChannels)
		//read the channeled samples into a variable and then process that...
		for decodedResult := range DecodedImpulseChannel {
			for i := 0; i < len(decodedResult); i++ {
				myFirImpulse[i] = append(myFirImpulse[i], decodedResult[i]...)
			}
		}

		elapsedTime := int(time.Since(startTime).Milliseconds())

		println("")
		println("============================================================================================")
		println("AudioLib - Merge Reading FIR: ", elapsedTime)
		println("============================================================================================")
		println("")

		// Mono FIR filter & multi-channel Audio - only apply to first 2 channels
		if len(myFirImpulse) == 1 && len(myConvolvers) >= 2 {
			println("Mono FIR filter & multi-channel Audio - exist")
			myConvolvers[0].FilterImpulse = (*lclPEQFilters)[0].Impulse
			myConvolvers[1].FilterImpulse = (*lclPEQFilters)[1].Impulse
			inputSamples, err := foxResampler.ResampleUpsample(myFirImpulse[0], myImpulseDecoder.SampleRate, TargetSampleRate, 10)
			//println("Impulse length:", len((myConvolvers)[0].FilterImpulse), "Signal: ", len(inputSamples))
			if err != nil {
				println("Error resampling")
				return
			}
			myConvolvedSignal[0] = (myConvolvers)[0].ConvolveFFT(inputSamples)
			myConvolvedSignal[1] = (myConvolvers)[1].ConvolveFFT(inputSamples)

		} else {
			//Multi channel FIR filter match to corresponding channels.
			println("Multi channel FIR filter match to corresponding channels")
			for i := 0; i < min(len(myFirImpulse), len(myConvolvers)); i++ {
				myConvolvers[i].FilterImpulse = (*lclPEQFilters)[i].Impulse
				//println("Impulse length:", len((myConvolvers)[i].FilterImpulse), "Signal: ", len(myFirImpulse[i]))
				// Use a channel level resampler
				//resamples in place - quality 30 is pretty good, 10 is fast
				inputSamples, err := foxResampler.ResampleUpsample(myFirImpulse[i], myImpulseDecoder.SampleRate, TargetSampleRate, 10)
				elapsedTime = int(time.Since(startTime).Milliseconds()) - elapsedTime
				println("AudioLib - Resample: ", elapsedTime)

				if err != nil {
					println("Error resampling")
					return
				}
				myConvolvedSignal[i] = (myConvolvers)[i].ConvolveFFT(inputSamples)
				elapsedTime = int(time.Since(startTime).Milliseconds()) - elapsedTime
				println("AudioLib - ConvolveFFT: ", elapsedTime)

			}
		}
		elapsedTime = int(time.Since(startTime).Milliseconds()) - elapsedTime

		println("")
		println("============================================================================================")
		println("AudioLib - Resample & Convolve FIR: ", elapsedTime)
		println("============================================================================================")
		println("")
	}()

	fmt.Println("Setting up Encoder: ", targetLevel)
	//Normalisation here lowers the level of 24 bit
	_ = foxNormalizer.Normalize(myConvolvedSignal, targetLevel)

	// Now map the Convolved Signal back to the Convolver
	for i := 0; i < min(len(myConvolvedSignal), len(myConvolvers)); i++ {

		(myConvolvers)[i].AmendFilterImpulse(myConvolvedSignal[i])
	}

	WG.Wait()
	fmt.Println("Test: closing decoder")

	return myConvolvers
	//myDecoder.Close()

}

// Apply convolution (example: FIR filter)
func applyConvolution(inputCh, outputCh chan []float64, myImpulse []float64) {

	myConvolver := foxConvolver.NewConvolver(myImpulse)
	myConvolver.ConvolveChannel(inputCh, outputCh)

}

// Split audio data into separate channels
func channelSplitter(inputCh chan [][]float64, outputChs []chan []float64, channelCount int) {
	go func() {
		for chunk := range inputCh {
			for i := 0; i < channelCount; i++ {
				channelData := chunk[i]
				outputChs[i] <- channelData
			}
		}

		// Close all output channels after processing
		for _, ch := range outputChs {
			close(ch)
		}
	}()
}

// Merge audio data from all channels
func mergeChannels(inputChannels []chan []float64, outputChannel chan [][]float64, numChannels int) {
	go func() {
		// Iterate through all input channels
		for {
			var mergedChunks [][]float64 // Temporary slice to hold data from each channel

			// Receive data from all channels
			for i := 0; i < numChannels; i++ {
				chunk, ok := <-inputChannels[i]
				if !ok {
					// Channel closed
					inputChannels[i] = nil // Mark as closed
					continue
				}
				mergedChunks = append(mergedChunks, chunk)
			}

			// Break if all channels are closed
			if len(mergedChunks) == 0 {
				break
			}

			// Send merged chunk
			outputChannel <- mergedChunks
		}

		// Close the output channel

		println("channelMerger - closing output")
		close(outputChannel)
	}()
}
