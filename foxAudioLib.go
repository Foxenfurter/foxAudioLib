package main

import (
	"fmt"
	"sync"
	"time"

	"github.com/Foxenfurter/foxAudioLib/foxAudioEncoder"

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
	err := myDecoder.Initialise()
	if err != nil {
		fmt.Println(functionName+": Decoding input file... ", myDecoder.Filename)
	}
	myDecoder.Filename = "" // Replace with your test file
	//myDecoder.Filename = "C:\\Users\\jonat\\Music\\Resolution\\Pencil_3244.wav"
	//myDecoder.Filename = "c:\\temp\\Pencil_1644.wav" // Replace with your test file
	//myDecoder.Filename = "c:\\temp\\impulse.wav" // Replace with your test file
	//myDecoder.Filename = "c:\\temp\\Loudness.wav" // Replace with your test file
	//myDecoder.Filename = "C:\\Users\\jonat\\go\\src\\github.com\\Foxenfurter\\wavTester\\Ocean-Lab.wav"

	// Initialise Audio Encoder
	myEncoder := foxAudioEncoder.AudioEncoder{
		Type:        "Wav",
		SampleRate:  myDecoder.SampleRate,
		BitDepth:    myDecoder.BitDepth, // Adjust as needed
		NumChannels: myDecoder.NumChannels,
		Size:        int64(myDecoder.Size), // Size for a 5-second file (adjust as needed)
		Filename:    "./output.wav",
	}

	fmt.Println("Test: Output file: ", myEncoder.Filename)
	fmt.Println("Test: Creating new Encoder SampleRate: ", myEncoder.SampleRate, " Channels: ", myEncoder.NumChannels, " BitDepth: ", myEncoder.BitDepth)
	// Create encoder
	err = myEncoder.Initialise()
	if err != nil {
		fmt.Println("Test: panic on New Header...")
		panic(err)
	}

	var WG sync.WaitGroup
	DecodedSamplesChannel := make(chan [][]float64, 10000)
	ThrottleLoaderChannel := make(chan time.Duration)
	ThrottleLoaderChannel = nil

	fmt.Println("Test: Decoding Data...")
	WG.Add(1)
	go func() {
		defer WG.Done()
		myDecoder.DecodeSamples(DecodedSamplesChannel, ThrottleLoaderChannel)

	}()

	fmt.Println("Test: Encoding Data...")
	WG.Add(1)
	go func() {

		defer WG.Done()
		//myEncoder.AccumulateAndEncodeChannel(DecodedSamplesChannel, ThrottleLoaderChannel, 40000)
		myEncoder.EncodeSamplesChannel(DecodedSamplesChannel, ThrottleLoaderChannel)

	}()

	fmt.Println("Test: Waiting...")
	WG.Wait()

	if ThrottleLoaderChannel != nil {
		close(ThrottleLoaderChannel)
	}
	elapsedTime := int(time.Since(startTime).Seconds())
	println("Test: elapsed time: ", elapsedTime)
	//myDecoder.Close()

}
