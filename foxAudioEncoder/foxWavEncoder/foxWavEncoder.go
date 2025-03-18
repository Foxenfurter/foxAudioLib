// Package: github.com/Foxenfurter/foxAudioLib/foxAudioEncoder/foxwavEncoder/foxwavEncoder.go
// pkg for encoding a stream into wav format. The package has been specified to allow a header to be written in the initial phase
// the body to be following this. The package is expected to be called as part of an encoder function. which will run asynchronously

package foxWavEncoder

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"strconv"
)

// Structure holds basic information about the Samples to be encoded
type FoxEncoder struct {
	SampleRate  int
	BitDepth    int
	NumChannels int
	Size        int64
}

const maxValue32BitInt = 2147483647
const minValue32BitInt = -2147483648
const maxValue24BitInt = 8388607
const minValue24BitInt = -8388608
const maxValue16Bit = 32767.0
const minValue16Bit = -32768.0

/*
EncodeHeader generates the WAV file header (RIFF format) as a byte slice
using information from ther FoxEncoder Struct
*/
func (we *FoxEncoder) EncodeHeader() ([]byte, error) {
	var dataSize, fileSize int64

	if we.Size != 0 {
		dataSize = we.Size
		fileSize = we.Size + 36
	} else {

		dataSize = math.MaxUint32 - 36 // Representing an unknown or unlimited size
		fileSize = math.MaxUint32
	}

	// Create a buffer to store the WAV header
	headerBuffer := new(bytes.Buffer)

	// Write WAV file header to the buffer
	headerBuffer.WriteString("RIFF")

	binary.Write(headerBuffer, binary.LittleEndian, int32(fileSize))
	headerBuffer.WriteString("WAVE")

	// Write the format chunk
	headerBuffer.WriteString("fmt ")
	binary.Write(headerBuffer, binary.LittleEndian, int32(16))                                           // Size of the format chunk
	binary.Write(headerBuffer, binary.LittleEndian, int16(1))                                            // Audio format (PCM)
	binary.Write(headerBuffer, binary.LittleEndian, int16(we.NumChannels))                               // Number of channels
	binary.Write(headerBuffer, binary.LittleEndian, int32(we.SampleRate))                                // Sample rate
	binary.Write(headerBuffer, binary.LittleEndian, int32(we.SampleRate*we.NumChannels*(we.BitDepth/8))) // Byte rate
	binary.Write(headerBuffer, binary.LittleEndian, int16(we.NumChannels*(we.BitDepth/8)))               // Block align
	binary.Write(headerBuffer, binary.LittleEndian, int16(we.BitDepth))                                  // Bits per sample

	// Write the data chunk header to the buffer
	headerBuffer.WriteString("data")
	binary.Write(headerBuffer, binary.LittleEndian, int32(dataSize))
	//fmt.Println("foxWavEncoder: DataSize: ", uint32(dataSize))
	return headerBuffer.Bytes(), nil
}

func (we *FoxEncoder) EncodeDataold(buffer [][]float64) ([]byte, error) {
	// Calculate the total number of samples across all channels
	totalSamples := len(buffer[0]) // Assuming all channels have the same number of samples
	numChannels := we.NumChannels
	if len(buffer) != numChannels {
		return nil, errors.New("foxwavencoder: number of channels in the buffer: " + strconv.Itoa(len(buffer)) + " doesn't match the specified number of channels: " + strconv.Itoa(numChannels))
	}

	// Create a buffer to accumulate encoded samples
	encodedBuffer := new(bytes.Buffer)

	for i := 0; i < totalSamples; i++ {
		//fmt.Println("foxWavEncoder: ", i)
		for channel := 0; channel < numChannels; channel++ {
			sample := math.Max(-1.0, math.Min(1.0, buffer[channel][i]))
			var sampleBytes []byte

			switch we.BitDepth {
			case 16:
				sampleBytes = we.convertTo16BitSample(sample)
			case 24:
				sampleBytes = we.convertTo24BitSample(sample)
			case 32:
				sampleBytes = we.convertTo32BitSample(sample)
			}

			encodedBuffer.Write(sampleBytes)
		}
	}
	//fmt.Println("foxWavEncoder: Finished Encoding Buffer.")
	// Return the result directly
	return encodedBuffer.Bytes(), nil
}

func (we *FoxEncoder) convertTo16BitSample(sample float64) []byte {
	//const bitDepth int = 16

	// Scale the sample to the range of 16-bit signed integers
	scaledValue := sample * float64(maxValue16Bit)
	// Round to the nearest integer
	roundedValue := int16(math.Round(scaledValue))

	// Clip the value to the valid range
	clippedValue := int16(math.Max(float64(minValue16Bit), math.Min(float64(maxValue16Bit), float64(roundedValue))))

	// Convert the clipped value to a byte array in little-endian format
	return []byte{
		byte(clippedValue & 0xFF),
		byte((clippedValue >> 8) & 0xFF),
	}
}

// convertTo24BitSample converts a float64 sample to 24-bit PCM.
func (we *FoxEncoder) convertTo24BitSample(sample float64) []byte {

	// Combine scaling, rounding, and clipping in one step with integer math
	intValue := int32(math.Max(math.Min(math.Round(sample*float64(maxValue24BitInt)), float64(maxValue24BitInt)), float64(minValue24BitInt)))

	// Efficient bitwise conversion using shifts and masks
	return []byte{
		byte(intValue & 0xFF),
		byte((intValue >> 8) & 0xFF),
		byte((intValue >> 16) & 0xFF),
	}
}

// convertTo32BitSample converts a float64 sample to 32-bit PCM.
func (we *FoxEncoder) convertTo32BitSample(sample float64) []byte {

	// Combine scaling, rounding, and clipping in one step
	intValue := int32(math.Max(math.Min(math.Round(sample*float64(maxValue32BitInt)), float64(maxValue32BitInt)), float64(minValue32BitInt)))

	// Convert to byte array in little-endian format using bitwise operations
	return []byte{
		byte(intValue & 0xFF),
		byte((intValue >> 8) & 0xFF),
		byte((intValue >> 16) & 0xFF),
		byte((intValue >> 24) & 0xFF),
	}
}

func (we *FoxEncoder) EncodeData(buffer [][]float64) ([]byte, error) {
	totalSamples := len(buffer[0])
	numChannels := we.NumChannels

	// Validate input
	if len(buffer) != numChannels {
		return nil, fmt.Errorf("channel count mismatch")
	}

	bytesPerSample := we.BitDepth / 8
	totalBytes := totalSamples * numChannels * bytesPerSample
	//fixed size buffer
	encoded := make([]byte, totalBytes)

	switch we.BitDepth {
	//encode loops within  switch statement faster than multiple switch statements
	case 16:
		max := maxValue16Bit
		for i := 0; i < totalSamples; i++ {
			for ch := 0; ch < numChannels; ch++ {
				pos := (i*numChannels + ch) * bytesPerSample
				sample := math.Max(-1.0, math.Min(1.0, buffer[ch][i]))
				//faster than rounding
				scaled := int16(sample*max + 0.5)
				binary.LittleEndian.PutUint16(encoded[pos:], uint16(scaled))
			}
		}

	case 24:
		max := float64(maxValue24BitInt)
		for i := 0; i < totalSamples; i++ {
			for ch := 0; ch < numChannels; ch++ {
				pos := (i*numChannels + ch) * bytesPerSample
				sample := math.Max(-1.0, math.Min(1.0, buffer[ch][i]))
				//faster than rounding
				scaled := int32(sample*max + 0.5)
				// Manual little-endian write for 24-bit
				encoded[pos] = byte(scaled)
				encoded[pos+1] = byte(scaled >> 8)
				encoded[pos+2] = byte(scaled >> 16)
			}
		}

	case 32:
		max := float64(maxValue32BitInt)
		for i := 0; i < totalSamples; i++ {
			for ch := 0; ch < numChannels; ch++ {
				pos := (i*numChannels + ch) * bytesPerSample
				sample := math.Max(-1.0, math.Min(1.0, buffer[ch][i]))
				//faster than rounding
				scaled := int32(sample*max + 0.5)
				binary.LittleEndian.PutUint32(encoded[pos:], uint32(scaled))
			}
		}

	default:
		return nil, errors.New("unsupported bit depth")
	}

	return encoded, nil
}
