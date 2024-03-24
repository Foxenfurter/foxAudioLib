// Package: github.com/Foxenfurter/foxAudioLib/foxAudioDecoder
// filename foxAudioDecoder.go
// package calls a decoder  based upon the supplied format
// and then processes the returned bytestream
// either to standardd out or to the supplied file name
package foxAudioDecoder

import (
	"errors"
	"fmt"

	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/Foxenfurter/foxAudioLib/foxAudioDecoder/foxWavReader"
)

const packageName = "foxAudioDecoder"

// The input Buffer is used to read in byte date from the external source
const InputBufferSize = 64000 // I don't think this is used anywhere

// The DecoderFrameSize is the number of frame samples to output ian each frame.
// A frame being an arbitrary number of frame samples and a frame sample being  n channels of samples e.g. stereo is 2 single samples.
const DefaultDecoderFrameSize int = 1000 // I don't think this is used anywhere

type AudioDecoder struct {
	SampleRate  int
	BitDepth    int
	NumChannels int
	Size        uint32 // Size of the audio data
	Type        string
	Filename    string // Added for file reading
	WavDecoder  *foxWavReader.WavReader
	FrameSample int
	DebugFunc   func(string) // enables the use of an external debug function supplied at the application level - expect to use foxLog

}

// NewDecoder creates a new decoder with implicit file opening or Stdin setup
func (myDecoder *AudioDecoder) Initialise() error {
	const functionName = "Initialise"
	// We need to be able to read the header in the filestream before we do any processing
	// we are going to peak this and 1000 bytes should cover almost all formats.
	var myFile *os.File
	if myDecoder.Filename == "" {
		myFile = os.Stdin
	} else {
		var err error
		// clean and standardize the filename
		myDecoder.Filename = filepath.ToSlash(filepath.Clean(myDecoder.Filename))
		myFile, err = os.Open(myDecoder.Filename)
		if err != nil {
			return err
		}
	}

	// Initialize decoder based on type
	// Decide which encoder to use
	switch strings.ToUpper(myDecoder.Type) {
	case "WAV":
		myDecoder.WavDecoder = &foxWavReader.WavReader{}
		myDecoder.WavDecoder.Input = myFile
		myDecoder.WavDecoder.DebugFunc = myDecoder.DebugFunc
		//Init the header
		err := myDecoder.WavDecoder.DecodeWavHeader()
		if err != nil {
			return err
		}
		// Read the audio header data into a buffer
		myDecoder.BitDepth = int(myDecoder.WavDecoder.BitDepth)
		myDecoder.SampleRate = int(myDecoder.WavDecoder.SampleRate)
		myDecoder.NumChannels = int(myDecoder.WavDecoder.NumChannels)

		// If the input buffer is bigger then 2000 than piping in through std in seems to cause issues for the upstream app
		// hence setting buffer size to too
		myMultiple := int(1200 / ((myDecoder.BitDepth / 8) * myDecoder.NumChannels))
		myDecoder.FrameSample = (myDecoder.BitDepth / 8) * myDecoder.NumChannels * myMultiple
		myDecoder.Size = myDecoder.WavDecoder.Size

	default:
		errorText := "unsupported encoder type "
		return errors.New(packageName + ":" + functionName + ":" + errorText)
	}

	// It is important that the low level decoder packages support these setter functions

	myDecoder.debug(fmt.Sprintf(packageName+":"+functionName+" SampleRate: [%v] Channels: [%v] BitDepth: [%v] Size [%v] FrameSample [%v]",
		myDecoder.SampleRate, myDecoder.NumChannels, myDecoder.BitDepth, myDecoder.Size, myDecoder.FrameSample))

	return nil
}

// Should call the lower level function and pass in the Channel to be used for transmitting decoded samples
// Throttle Loadre is optional and enables the Decoder to limit the speed of the loader.
// Current version of wav loader does not need throttling
func (myDecoder *AudioDecoder) DecodeSamples(DecodedSamplesChannel chan [][]float64, ThrottleLoaderChannel chan time.Duration) error {
	const functionName = "DecodeSamples"
	switch strings.ToUpper(myDecoder.Type) {
	case "WAV":
		err := myDecoder.WavDecoder.DecodeInput(DecodedSamplesChannel)
		return err
	default:
		errorText := "unsupported encoder type "
		return errors.New(packageName + ":" + functionName + ":" + errorText)
	}

}

func maxValueForBitDepth(depth int) float64 {

	MaxValue := 0.0
	switch depth {
	case 16:
		MaxValue = 32768.0
	case 24:
		MaxValue = 8388608.0
	case 32:
		MaxValue = 2147483647.0
	}
	return MaxValue
}

func (myDecoder *AudioDecoder) Close() {
	const functionName = "Close"
	// channels are closed in the functions that populate them.
	// This function is called if any other cleanup is needed
	myDecoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  All done..."))
}

// Some functions for identifying the most common audio formats from the header in the bytestream under 1000 bytes needed.
func identifyFormat(data []byte) string {
	patterns := map[string][]byte{
		"WAV":  {0x52, 0x49, 0x46, 0x46},
		"MP3":  {0x49, 0x44, 0x33},             // Check for "ID3"
		"FLAC": {0x66, 0x4C, 0x61, 0x43},       // Check for "fLaC"
		"AIFF": {0x6D, 0x73, 0x62, 0x66},       // Check for "msbf"
		"Ogg":  {0x4F, 0x67, 0x67, 0x53},       // Check for "OggS"
		"WMA":  {0x57, 0x4D, 0x41, 0x00},       // Check for "WMA\x00\x00\x00"
		"M4A":  {0x66, 0x74, 0x79, 0x70},       // Check for "ftyp"
		"APE":  {0x41, 0x50, 0x45, 0x76, 0x32}, // Check for "APEv2"
	}

	for format, pattern := range patterns {
		if len(data) >= len(pattern) && compareBytes(data[:len(pattern)], pattern) {
			return format
		}
	}

	return "Unknown"
}

func compareBytes(a, b []byte) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// Function to handle debug calls, allowing for different logging implementations
func (myDecoder *AudioDecoder) debug(message string) {

	if myDecoder.DebugFunc != nil {
		myDecoder.DebugFunc(message)
	} else { // if no external debug function available just print the message
		println(message)
	}

}
