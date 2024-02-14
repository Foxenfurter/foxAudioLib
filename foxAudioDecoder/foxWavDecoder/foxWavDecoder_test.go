package foxWavDecoder_test

import (
	"fmt"
	"os"
	"sync"
	"testing"

	foxWavDecoder "github.com/Foxenfurter/foxAudioLib/foxAudioDecoder/foxWavDecoder"
)

// ... (DecodeAudioData function and helper functions from previous responses) ...

const InputBufferSize = 65000

func TestReadAndDecodeAudio(t *testing.T) {
	//filename := "c:\\temp\\Pencil_1644.wav"
	filename := "C:\\Users\\jonat\\Music\\Resolution\\Pencil_24176.wav"

	fmt.Println("Test: Full running TestReadAndDecodeAudio...", filename)
	//var mainDecoder foxWavDecoder.FoxDecoder
	var decoder foxWavDecoder.FoxWavDecoder

	fmt.Println("Test: running reader sub-routine...")
	inputFile, err := os.Open(filename)
	if err != nil {
		fmt.Println("Test: File Error...", err)
		panic(err)
	}
	defer inputFile.Close()
	decoder.InputFile = inputFile
	err = decoder.DecodeWavHeader()
	if err != nil {
		println("Test: Header Read Error...", err)
	}
	var WG sync.WaitGroup
	DecodedSamplesChannel := make(chan [][]float64, 10000)
	fmt.Println("Test: Decoding Data...")
	WG.Add(1)
	go func() {
		defer WG.Done()
		decoder.DecodeInput(DecodedSamplesChannel)

	}()

	fmt.Println("Test: Encoding Data...")
	WG.Add(1)
	go func() {

		defer WG.Done()
		var ResultCounter int

		for decodedResult := range DecodedSamplesChannel {
			// Discard decoded results
			ResultCounter += len(decodedResult[0])

		}
		fmt.Println("Test: Not waiting anymore", ResultCounter)
	}()

	fmt.Println("Test: Waiting...")
	WG.Wait()

}

func TestMain(m *testing.M) {
	// Run tests
	fmt.Println("Running Decoder  Test:")
	exitCode := m.Run()

	// Exit with the test result code
	os.Exit(exitCode)
}
