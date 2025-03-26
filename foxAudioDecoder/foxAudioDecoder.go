// Package: github.com/Foxenfurter/foxAudioLib/foxAudioDecoder
// filename foxAudioDecoder.go
// package calls a decoder  based upon the supplied format
// and then processes the returned bytestream
// either to standardd out or to the supplied file name
package foxAudioDecoder

import (
	"bufio"
	"encoding/binary"
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
	BigEndian   bool
	Size        int64 // Size of the audio data
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
	myDecoder.debug("Initialising...")
	var myFile *os.File
	if myDecoder.Filename == "" {
		stat, err := os.Stdin.Stat()
		if err != nil {
			return err // Handle the error
		}

		if (stat.Mode() & os.ModeCharDevice) == 0 {
			myDecoder.debug("Data is being piped to stdin")
			// Proceed with reading from stdin (using your combinedReader logic)
			// ...
			myFile = os.Stdin

		} else {
			return errors.New("no file specified and stdin is not a pipe")
		}

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
		myDecoder.WavDecoder.Input = bufio.NewReaderSize(myFile, 4096)
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
		myDecoder.Size = int64(myDecoder.WavDecoder.Size)
	case "PCM":
		myDecoder.WavDecoder = &foxWavReader.WavReader{}
		myDecoder.WavDecoder.Input = bufio.NewReaderSize(myFile, 4096)
		myDecoder.WavDecoder.DebugFunc = myDecoder.DebugFunc
		//Do not Init the header instead we will have received the necessary header information as arguments

		myDecoder.WavDecoder.BitDepth = int(myDecoder.BitDepth)
		myDecoder.WavDecoder.SampleRate = int(myDecoder.SampleRate)
		myDecoder.WavDecoder.NumChannels = int(myDecoder.NumChannels)
		if myDecoder.BigEndian {
			myDecoder.WavDecoder.ByteOrder = binary.BigEndian
			myDecoder.WavDecoder.LittleEndian = false
		} else {
			myDecoder.WavDecoder.ByteOrder = binary.LittleEndian
			myDecoder.WavDecoder.LittleEndian = true
		}
		// If the input buffer is bigger then 2000 than piping in through std in seems to cause issues for the upstream app
		// hence setting buffer size to too
		myMultiple := int(1200 / ((myDecoder.BitDepth / 8) * myDecoder.NumChannels))
		myDecoder.FrameSample = (myDecoder.BitDepth / 8) * myDecoder.NumChannels * myMultiple
		myDecoder.Size = int64(myDecoder.WavDecoder.Size)

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
	case "PCM":
		err := myDecoder.WavDecoder.DecodePCMInput(DecodedSamplesChannel)
		//err := myDecoder.WavDecoder.DecodeInput(DecodedSamplesChannel)
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
	// Define format checks with offset and magic bytes
	type formatCheck struct {
		offset  int
		pattern []byte
		format  string
	}

	checks := []formatCheck{
		// WAV/RIFF
		{0, []byte{0x52, 0x49, 0x46, 0x46}, "WAV"},
		// MP3 with ID3 tag
		{0, []byte{0x49, 0x44, 0x33}, "MP3"},
		// FLAC
		{0, []byte{0x66, 0x4C, 0x61, 0x43}, "FLAC"},
		// AIFF ("FORM" at 0, then "AIFF" at 8)
		{0, []byte{0x46, 0x4F, 0x52, 0x4D}, "AIFF"},
		// Ogg
		{0, []byte{0x4F, 0x67, 0x67, 0x53}, "Ogg"},
		// WMA/ASF GUID (first 16 bytes)
		{0, []byte{0x30, 0x26, 0xB2, 0x75, 0x8E, 0x66, 0xCF, 0x11, 0xA6, 0xD9, 0x00, 0xAA, 0x00, 0x62, 0xCE, 0x6C}, "WMA"},
		// M4A (MP4) - "ftyp" at offset 4
		{4, []byte{0x66, 0x74, 0x79, 0x70}, "ftyp"},
		// APE - "MAC " at start
		{0, []byte{0x4D, 0x41, 0x43, 0x20}, "APE"},
		// MIDI - "MThd" at 0
		{0, []byte{0x4D, 0x54, 0x68, 0x64}, "MIDI"},
		// AMR - "#!AMR"
		{0, []byte{0x23, 0x21, 0x41, 0x4D, 0x52}, "AMR"},
		// AU - ".snd"
		{0, []byte{0x2E, 0x73, 0x6E, 0x64}, "AU"},
	}

	var format string

	for _, check := range checks {
		end := check.offset + len(check.pattern)
		if end > len(data) {
			continue
		}
		if compareBytes(data[check.offset:end], check.pattern) {
			return check.format // Corrected: return the found format immediately
		}
	}

	if format == "ftyp" {
		if len(data) >= 12 {
			brand := data[8:12]
			if compareBytes(brand, []byte{0x6D, 0x70, 0x34, 0x32}) || compareBytes(brand, []byte{0x6D, 0x34, 0x61, 0x20}) {
				return "M4A"
			} else {
				return "MP4"
			}
		}
	}

	// Fallback for MP3 without ID3 tag (check frame sync)
	if len(data) >= 2 && data[0] == 0xFF && (data[1]&0xE0) == 0xE0 {
		return "MP3"
	}

	return "Unknown"
}

// Example compareBytes function (assumed to be correctly implemented)
func compareBytes(a, b []byte) bool {
	if len(a) != len(b) {
		return false
	}
	for i, v := range a {
		if v != b[i] {
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
