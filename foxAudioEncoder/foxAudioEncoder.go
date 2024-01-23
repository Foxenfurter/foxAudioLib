// package calls an encoder based upon the supplied format
// and then processes the returned bytestream
// either to standardd out or to the supplied file name
package foxAudioEncoder

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"

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
	switch myEncoder.Type {
	case "Wav":
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

func (myEncoder *AudioEncoder) EncodeData(buffer [][]float64) error {
	const functionName = "EncodeData"
	encodedData, err := myEncoder.encoder.EncodeData(buffer)
	if err != nil {
		return errors.New(packageName + ":" + functionName + ": " + err.Error())
	}
	return myEncoder.writeData(encodedData)
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
