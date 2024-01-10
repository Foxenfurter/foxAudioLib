package foxDSP_test

import (
	"fmt"
	"os"
	"testing"
	"time"

	"github.com/Foxenfurter/foxAudioLib/foxDSP"
)

func TestDSP(t *testing.T) {
	sampleRate := 48000
	bitDepth := 24
	testImpulseFilename := "c:/temp/Testfilter75_go.wav"
	dtStartRun := time.Now()
	var Fc int = 20000
	Nyquist := 0.445
	length := 4000
	//NyQ := float64(Fc) / float64(sampleRate)
	NyR := float64(Fc) / float64(sampleRate)

	if NyR > Nyquist {
		// Need to add in Error Logger here.
		Fc = int(float64(sampleRate) * Nyquist)
	}
	coefficients := foxDSP.CalcBiquadFilter("lowpass", 1000, 44100, 0.5, 0.1, "Q")
	if len(coefficients) != 6 {
		t.Errorf("Expected 6 coefficients, got %d", len(coefficients))
	}
	var filters [][]float64
	//filters = append(filters, coefficients)
	// Matches 82.5 - 75 phon
	filters = append(filters, foxDSP.CalcBiquadFilter("lowshelf", 80, sampleRate, 1.8, 0.41, "Q"))
	filters = append(filters, foxDSP.CalcBiquadFilter("highshelf", 6300, sampleRate, 0.25, 0.41, "Q"))
	filters = append(filters, foxDSP.CalcBiquadFilter("peak", 2400, sampleRate, -1.8, 0.17, "Q"))
	filters = append(filters, foxDSP.CalcBiquadFilter("lowshelf", 35, sampleRate, 0.25, 1.25, "Q"))
	filters = append(filters, foxDSP.CalcBiquadFilter("peak", 1200, sampleRate, 0.25, 1.4, "Q"))
	fmt.Printf("Number of filters created: %v\n", filters)
	fmt.Printf("now generate impulses: \n")
	TestImpulseResponse := make([][]float64, 0)

	TestImpulseResponse = append(TestImpulseResponse, foxDSP.CascadeFilters(filters, length))
	//TestImpulseResponse = append(TestImpulseResponse, newImpulse)
	fmt.Printf("impulse created - length: %v \n", len(TestImpulseResponse[0]))

	// Create a StreamManager instance
	streamManager := foxDSP.StreamManager{
		InputImpulse: TestImpulseResponse,
	}

	// Call ExportWavFile to export the WAV file
	err := foxDSP.ExportWavFile(testImpulseFilename, sampleRate, bitDepth, &streamManager)
	if err != nil {
		fmt.Println("Error exporting WAV file:", err)
	} else {
		fmt.Println("WAV file exported successfully!")
	}

	//maxWav := foxDSP.WriteWavFile(testImpulseFilename, TestImpulseResponse, sampleRate, bitDepth)
	//fmt.Printf("Wav  created: %s %v %v %v \n", testImpulseFilename, sampleRate, bitDepth, maxWav)

	fmt.Printf("Time taken: %s\n", time.Since(dtStartRun))
}

func TestMain(m *testing.M) {
	// Run tests
	fmt.Println("Running DSP Test:")
	exitCode := m.Run()

	// Exit with the test result code
	os.Exit(exitCode)
}
