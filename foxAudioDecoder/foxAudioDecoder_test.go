package foxAudioDecoder_test

import (
	"fmt"
	"os"
	"sync"
	"testing"
	"time"

	"github.com/Foxenfurter/foxAudioLib/foxAudioDecoder"
)

func TestPCMDecoderFileLoader(t *testing.T) {
	// Initialize PCMDecoder
	decoder := foxAudioDecoder.PCMDecoder{
		//Filename: "c:\\temp\\Pencil_1644.wav", // Replace with your test file
		Filename:         "c:\\temp\\impulse.wav", // Replace with your test file
		DecoderFrameSize: 1000,                    // Adjust as needed
		Buffer:           make([]byte, foxAudioDecoder.InputBufferSize),
		Type:             "Wav",
	}
	fmt.Println("Decoding input file... ", decoder.Filename)
	err := decoder.Initialise()
	if err != nil {
		t.Fatalf("Error initializing PCMDecoder: %v", err)
	}
	var WG sync.WaitGroup
	// Create a channel to signal completion
	//doneProcessing := make(chan struct{})
	WG.Add(1)
	// Start decoding asynchronously
	go func() {
		defer WG.Done()
		//decoder.Start(doneProcessing)
		decoder.Start()
	}()

	WG.Add(1)
	go func() {
		defer WG.Done()
		//decoder.DecodeData(doneProcessing)
		decoder.DecodeData()
	}()

	// Close the doneProcessing channel to signal that decoding is finished

	WG.Add(1)
	go func() {
		var ResultCounter int
		defer WG.Done()
		for decodedResult := range decoder.OutputChan {
			// Discard decoded results
			ResultCounter += len(decodedResult[0])
			//fmt.Println("Test: Received from Results channel\n", len(decodedResult[0]))
		}
		fmt.Println("Test: Not waiting anymore", ResultCounter)
		//<-doneProcessing

	}()
	fmt.Println("Test: Are we still waiting")
	// Wait for the asynchronous decoding to complete

	WG.Wait()
	fmt.Println("Test: closing decoder")
	// Close channels
	//close(doneProcessing)
	decoder.Close()

	fmt.Println("Test: Number of Frame Samples:", decoder.DecoderFrameCount)
	fmt.Println("Test: Done...")
}

// Helper function for timeout
func timeout(seconds int) <-chan struct{} {
	ch := make(chan struct{})
	go func() {
		defer close(ch)
		time.Sleep(time.Duration(seconds) * time.Second)
	}()
	return ch
}

func TestMain(m *testing.M) {
	// Run tests
	fmt.Println("Running Encoder  Test:")
	exitCode := m.Run()

	// Exit with the test result code
	os.Exit(exitCode)
}
