// package calls an encoder based upon the supplied format
// and then processes the returned bytestream
// either to standardd out or to the supplied file name
package foxAudioEncoder

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"

	foxWavEncoder "github.com/Foxenfurter/foxAudioLib/foxAudioEncoder/foxWavEncoder"
)

const packageName = "foxAudioEncoder"

// Encoder definition EncoderDefinition
type AudioEncoder struct {
	SampleRate  int
	BitDepth    int
	NumChannels int
	Size        int64
	Type        string
	Filename    string // Added filename field

	DebugFunc func(string) // enables the use of an external debug function supplied at the application level - expect to use foxLog

	encoder EncoderInterface
}

// each Encoder must have these methods defined
type EncoderInterface interface {
	EncodeHeader() ([]byte, error)
	EncodeData(samples [][]float64) ([]byte, error)
}

// Constructor for Encoder
func (myEncoder *AudioEncoder) Initialise() error {
	const functionName = "Initialise"
	myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  Creating Header..."))
	var err error
	// Remove existing file if it exists and a filename is provided
	if myEncoder.Filename != "" { // Only check for existing file if filename is not blank
		//clean and standardize the file path
		myEncoder.Filename = filepath.ToSlash(filepath.Clean(myEncoder.Filename))

		if _, err := os.Stat(myEncoder.Filename); err == nil {
			err = os.Remove(myEncoder.Filename)
			if err != nil {
				return fmt.Errorf(packageName+":"+functionName+":error removing existing file: %w", err)
			}
		}
	}
	myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  decide which encoder to use..."))

	// Decide which encoder to use
	switch strings.ToUpper(myEncoder.Type) {

	case "WAV":
		myEncoder.encoder = &foxWavEncoder.FoxEncoder{
			SampleRate:  myEncoder.SampleRate,
			BitDepth:    myEncoder.BitDepth,
			NumChannels: myEncoder.NumChannels,
			Size:        myEncoder.Size,
		}
		myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  use wav now write Header.."))
		err = myEncoder.writeHeader() // Write header during initialization
		if err != nil {
			return fmt.Errorf(packageName+":"+functionName+":error writing wav header: %w", err)
		}
	default:
		return errors.New(packageName + ":" + functionName + ":unsupported encoder type")
	}
	myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  Finished Header..."))
	return err
}

// Expose the methods
func (myEncoder *AudioEncoder) EncodeHeader() error {
	return myEncoder.writeHeader()
}

// Call low level encoder to convert [][]float64 samples to bytesream of choice, and then call output writer
func (myEncoder *AudioEncoder) EncodeData(buffer [][]float64) error {
	const functionName = "EncodeData"
	encodedData, err := myEncoder.encoder.EncodeData(buffer)
	if err != nil {
		return errors.New(packageName + ":" + functionName + ": " + err.Error())
	}
	return myEncoder.writeData(encodedData)
}

// Wraps the encoder into a channel based function that takes a stream of inputSamples as [][]float64, throttleInputChannel as duration and a wait group
// the optional throttleInputChannel will send a delay based upont the encoder processing time to be used by the feeding function in order to slow down any processing
func (myEncoder *AudioEncoder) EncodeSamplesChannel(samplesChannel <-chan [][]float64, throttleInputChannel chan<- time.Duration) error {
	const functionName = "EncodeSamplesChannel"

	for samples := range samplesChannel {

		start := time.Now()

		err := myEncoder.EncodeData(samples)
		if err != nil {
			return errors.New(packageName + ":" + functionName + ": " + err.Error())
		}

		elapsedTime := time.Since(start)
		//
		if throttleInputChannel != nil {
			throttleInputChannel <- elapsedTime
		}

	}

	return nil
}

func (myEncoder *AudioEncoder) AccumulateAndEncodeChannel(samplesChannel <-chan [][]float64, throttleInputChannel chan<- time.Duration, n int) error {
	const functionName = "AccumulateAndEncode"
	channelCount := myEncoder.NumChannels
	fmt.Println(packageName+":"+functionName+": Expecting samples: ", n)

	//counter for samples populated
	s := 0
	accumulatedSamples := make([][]float64, channelCount) // Allocate space for 2 channels
	// Assuming samples[0] represents the channel count
	start := time.Now()
	for samples := range samplesChannel {

		for i := range accumulatedSamples {
			accumulatedSamples[i] = make([]float64, n) // Initialize each channel with an empty slice
		}
		//println("Samples in this batch", len(samples), len(samples[0]))
		for si := 0; si < len(samples[0]); si++ {

			for c := 0; c < channelCount; c++ {

				accumulatedSamples[c][s] = samples[c][si]

			}

			if s == n-1 { // Check channel 0 for fullness
				// Encode and write accumulated samples
				fmt.Println("Encode and write accumulated samples...", s)
				// reset the counter
				s = 0
				err := myEncoder.EncodeData(accumulatedSamples)

				if err != nil {
					// Handle error
					return errors.New(packageName + ":" + functionName + ": " + err.Error())
				}

				//

			}
			s++
		}
		if throttleInputChannel != nil {
			elapsedTime := time.Since(start)
			throttleInputChannel <- elapsedTime
			start = time.Now()
		}

		fmt.Println("Encode and accumulated samples number so far: ", s)

	}
	// We gave sime leftover samples
	for c := 0; c < channelCount; c++ {
		accumulatedSamples[c] = accumulatedSamples[c][:s] // Only keep samples populated so far
	}

	// Handle remaining samples on the last iteration
	if len(accumulatedSamples[0]) > 0 { // Check channel 0 for remaining data
		fmt.Println("Encode and write remaining samples...")
		err := myEncoder.EncodeData(accumulatedSamples)
		if err != nil {
			// Handle error
			return errors.New(packageName + ":" + functionName + ": " + err.Error())

		}
		if throttleInputChannel != nil {
			elapsedTime := time.Since(start)
			throttleInputChannel <- elapsedTime

		}

	}
	return nil
}

func (myEncoder *AudioEncoder) AccumulateAndEncode(samples [][]float64, n int) error {
	const functionName = "AccumulateAndEncode"
	channelCount := myEncoder.NumChannels
	fmt.Println(packageName+":"+functionName+": Expecting samples: ", n)
	accumulatedSamples := make([][]float64, channelCount) // Allocate space for 2 channels
	for i := range accumulatedSamples {
		accumulatedSamples[i] = make([]float64, n) // Initialize each channel with an empty slice
	}
	//counter for samples populated
	s := 0
	// Assuming samples[0] represents the channel count

	for _, sample := range samples {
		for c := 0; c < channelCount; c++ {
			// Ensure enough slots in the accumulated sample for this channel
			if len(accumulatedSamples[c]) < n {
				//accumulatedSamples[c] = append(accumulatedSamples[c], make([]float64, n-len(accumulatedSamples[c]))...)

			}
			accumulatedSamples[c][s] = sample[c]
			//increment the sample count
			if c == 0 {
				s++
			}
			fmt.Println("Encode and accumulated samples number so far: ", s)
			// Add the current sample value to the appropriate channel
			//accumulatedSamples[c][len(accumulatedSamples[c])-1] = sample[c]
		}

		if s == n { // Check channel 0 for fullness
			// Encode and write accumulated samples
			fmt.Println("Encode and write accumulated samples...")
			err := myEncoder.EncodeData(accumulatedSamples)
			if err != nil {
				// Handle error

				return errors.New(packageName + ":" + functionName + ": " + err.Error())
			}

			// Reset accumulated samples for the next batch
			//	for c := 0; c < channelCount; c++ {
			//		accumulatedSamples[c] = accumulatedSamples[c][:0] // Clear slice without reallocation
			//	}
		}
	}
	for c := 0; c < channelCount; c++ {
		accumulatedSamples[c] = accumulatedSamples[c][:s] // Only keep samples populated so far
	}

	// Handle remaining samples on the last iteration
	if len(accumulatedSamples[0]) > 0 { // Check channel 0 for remaining data
		fmt.Println("Encode and write remaining samples...")
		err := myEncoder.EncodeData(accumulatedSamples)
		if err != nil {
			// Handle error
			return errors.New(packageName + ":" + functionName + ": " + err.Error())

		}
	}
	return nil
}

// Helper functions for file writing
func (myEncoder *AudioEncoder) writeHeader() error {
	const functionName = "writeHeader"
	myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  call low level Header.."))
	headerBytes, err := myEncoder.encoder.EncodeHeader()
	if err != nil {
		return errors.New(packageName + ":" + functionName + ": " + err.Error())
	}
	myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  writing header.."))
	err = writeOutput(headerBytes, myEncoder.Filename)
	if err != nil {
		return errors.New(packageName + ":" + functionName + ": " + err.Error())
	}
	return nil
}

func (e *AudioEncoder) writeData(data []byte) error {
	return writeOutput(data, e.Filename)
}

// Helper function for writing to a file
func writeOutput(data []byte, filename string) error {
	const functionName = "writeOutput"
	if filename == "" {
		// Write to standard out
		_, err := os.Stdout.Write(data)
		return errors.New(packageName + ":" + functionName + ": " + err.Error())
	} else {
		// Write to file
		file, err := os.OpenFile(filename, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0644)
		if err != nil {
			return errors.New(packageName + ":" + functionName + ": " + err.Error())
		}
		defer file.Close()
		_, err = file.Write(data)
		if err != nil {
			return errors.New(packageName + ":" + functionName + ": " + err.Error())
		}

	}
	return nil
}

// Function to handle debug calls, allowing for different logging implementations
func (myEncoder *AudioEncoder) debug(message string) {

	if myEncoder.DebugFunc != nil {
		myEncoder.DebugFunc(message)
	} else { // if no external debug function available just print the message
		println(message)
	}

}
