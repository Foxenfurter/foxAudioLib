package foxAudioDecoder_test

import (
	"fmt"
	"os"
	"sync"
	"testing"
	"time"

	"github.com/Foxenfurter/foxAudioLib/foxAudioDecoder"
)

func TestAudioDecoderFileLoader(t *testing.T) {
	// Initialize AudioDecoder
	decoder := foxAudioDecoder.AudioDecoder{
		//Filename: "c:\\temp\\Pencil_1644.wav", // Replace with your test file
		//Filename: "c:\\temp\\impulse.wav", // Replace with your test file
		Filename:         "c:\\temp\\Loudness_yz.wav", // Replace with your test file
		DecoderFrameSize: 1000,                        // Adjust as needed
		Buffer:           make([]byte, foxAudioDecoder.InputBufferSize),
		Type:             "Wav",
	}
	fmt.Println("Test: Decoding input file... ", decoder.Filename)
	err := decoder.Initialise()
	if err != nil {
		t.Fatalf("Test: Error initializing AudioDecoder: %v", err)
	}
	var WG sync.WaitGroup

	WG.Add(1)
	// Start decoding asynchronously
	go func() {
		defer WG.Done()
		decoder.ReadInput()
	}()

	WG.Add(1)
	go func() {
		defer WG.Done()
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

		}
		fmt.Println("Test: Not waiting anymore", ResultCounter)

	}()
	fmt.Println("Test: Are we still waiting")
	// Wait for the asynchronous decoding to complete

	WG.Wait()
	fmt.Println("Test: closing decoder")

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
