// package calls an encoder based upon the supplied format
// and then processes the returned bytestream
// either to standardd out or to the supplied file name
package foxAudioEncoder

import (
	"errors"
	"fmt"
	"os"

	foxWavEncoder "github.com/Foxenfurter/foxAudioLib/foxAudioEncoder/foxWavEncoder"
)

// Encoder definition
type EncoderDefinition struct {
	SampleRate  int
	BitDepth    int
	NumChannels int
	Size        int64
	Type        string
	Filename    string // Added filename field
}

// each Encoder must have these methods defined
type EncoderInterface interface {
	EncodeHeader() ([]byte, error)
	EncodeData(samples [][]float64) ([]byte, error)
}

// Encoder struct
type Encoder struct {
	encoder  EncoderInterface
	filename string // Added filename field
}

// Constructor for Encoder
func NewEncoder(definition *EncoderDefinition) (*Encoder, error) {
	encoder := &Encoder{filename: definition.Filename}
	var err error
	// Remove existing file if it exists and a filename is provided
	if encoder.filename != "" { // Only check for existing file if filename is not blank
		if _, err := os.Stat(encoder.filename); err == nil {
			err = os.Remove(encoder.filename)
			if err != nil {
				return nil, fmt.Errorf("error removing existing file: %w", err)
			}
		}
	}
	switch definition.Type {
	case "Wav":
		encoder.encoder = &foxWavEncoder.FoxEncoder{
			SampleRate:  definition.SampleRate,
			BitDepth:    definition.BitDepth,
			NumChannels: definition.NumChannels,
			Size:        definition.Size,
		}
		err = encoder.writeHeader() // Write header during initialization
	default:
		return nil, errors.New("unsupported encoder type")
	}

	return encoder, err
}

// Expose the methods
func (e *Encoder) EncodeHeader() error {
	return e.writeHeader()
}

func (e *Encoder) EncodeData(buffer [][]float64) error {
	encodedData, err := e.encoder.EncodeData(buffer)
	if err != nil {
		return err
	}
	return e.writeData(encodedData)
}

// Helper functions for file writing
func (e *Encoder) writeHeader() error {
	headerBytes, err := e.encoder.EncodeHeader()
	if err != nil {
		return err
	}
	return writeOutput(headerBytes, e.filename)
}

func (e *Encoder) writeData(data []byte) error {
	return writeOutput(data, e.filename)
}

// Helper function for writing to a file
func writeOutput(data []byte, filename string) error {
	if filename == "" {
		// Write to standard out
		_, err := os.Stdout.Write(data)
		return err
	} else {
		// Write to file
		file, err := os.OpenFile(filename, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0644)
		if err != nil {
			return err
		}
		defer file.Close()

		_, err = file.Write(data)
		return err
	}
}
