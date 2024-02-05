// pkg/foxEncode/wavEncoder/wavEncoder.go
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

// createWavHeader generates the WAV file header as a byte slice

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
	headerBuffer := make([]byte, 44) // WAV header size is 44 bytes

	// Write WAV file header to the buffer
	copy(headerBuffer[0:4], []byte("RIFF"))
	binary.LittleEndian.PutUint32(headerBuffer[4:8], uint32(fileSize)) // Placeholder for total file size - 36 bytes for the header
	copy(headerBuffer[8:12], []byte("WAVE"))

	// Write the format chunk to the buffer
	copy(headerBuffer[12:16], []byte("fmt "))
	binary.LittleEndian.PutUint32(headerBuffer[16:20], 16)                                                   // Size of the format chunk
	binary.LittleEndian.PutUint16(headerBuffer[20:22], 1)                                                    // Audio format (PCM)
	binary.LittleEndian.PutUint16(headerBuffer[22:24], uint16(we.NumChannels))                               // Number of channels
	binary.LittleEndian.PutUint32(headerBuffer[24:28], uint32(we.SampleRate))                                // Sample rate
	binary.LittleEndian.PutUint32(headerBuffer[28:32], uint32(we.SampleRate*we.NumChannels*(we.BitDepth/8))) // Byte rate
	binary.LittleEndian.PutUint16(headerBuffer[32:34], uint16(we.NumChannels*(we.BitDepth/8)))               // Block align
	binary.LittleEndian.PutUint16(headerBuffer[34:36], uint16(we.BitDepth))                                  // Bits per sample

	// Write the data chunk header to the buffer
	copy(headerBuffer[36:40], []byte("data"))
	binary.LittleEndian.PutUint32(headerBuffer[40:44], uint32(dataSize)) // Placeholder for data size
	fmt.Println("foxWavEncoder: DataSize.", uint32(dataSize))
	return headerBuffer, nil
}

func (we *FoxEncoder) EncodeData1(buffer [][]float64) ([]byte, error) {
	// Validate target bit depth
	if we.BitDepth != 16 && we.BitDepth != 24 && we.BitDepth != 32 {
		return nil, errors.New("foxWavEncoder: unsupported target bit depth: " + strconv.Itoa(we.BitDepth))
	}
	fmt.Println("foxWavEncoder: Encoding Buffer.")
	// Calculate multiplier for normalizing samples based on target bit depth
	maxValue := math.Pow(2, float64(we.BitDepth-1)) - 1

	// Initialize byte buffer
	var pcmBuffer bytes.Buffer

	// Iterate through channels and samples
	for _, channel := range buffer {
		for _, sample := range channel {
			// Normalize sample to fit the target bit depth
			normalizedSample := int(math.Round(sample * maxValue))

			// Write normalized sample to byte buffer based on target bit depth
			switch we.BitDepth {
			case 16:
				// Use int16 type explicitly to ensure little-endian encoding
				binary.Write(&pcmBuffer, binary.LittleEndian, int16(normalizedSample))
			case 24:
				// Write 3 bytes for 24-bit sample
				pcmBuffer.Write([]byte{
					byte(normalizedSample),
					byte(normalizedSample >> 8),
					byte(normalizedSample >> 16),
				})
			case 32:
				binary.Write(&pcmBuffer, binary.LittleEndian, int32(normalizedSample))
			}
		}
	}
	fmt.Println("foxWavEncoder: return Encoded Buffer.")
	// Return the result directly
	return pcmBuffer.Bytes(), nil
}

func (we *FoxEncoder) EncodeData(buffer [][]float64) ([]byte, error) {
	// Calculate the total number of samples across all channels
	totalSamples := len(buffer[0]) // Assuming all channels have the same number of samples
	numChannels := we.NumChannels
	if len(buffer) != numChannels {
		return nil, errors.New("foxwavencoder: number of channels in the buffer: " + strconv.Itoa(len(buffer)) + " doesn't match the specified number of channels: " + strconv.Itoa(numChannels))
	}

	// Ensure the buffer size is a multiple of the number of channels
	if totalSamples%numChannels != 0 {
		//return nil, errors.New("foxwavencoder: input buffer size is not a multiple of the number of channels")
	}

	// Create a buffer to accumulate encoded samples
	encodedBuffer := new(bytes.Buffer)
	//fmt.Println("foxWavEncoder: Encoding Buffer.", totalSamples, numChannels, we.BitDepth)
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
	const maxValue16Bit = 32767.0
	const minValue16Bit = -32768.0

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
	const bitDepth = 24
	const maxValue24Bit = 8388607.0
	const minValue24Bit = -8388608.0

	// Scale the sample to the range of 24-bit signed integers
	scaledValue := sample * float64(maxValue24Bit)

	// Round to the nearest integer
	roundedValue := int(math.Round(scaledValue))

	// Clip the value to the valid range
	clippedValue := int(math.Max(float64(minValue24Bit), math.Min(float64(maxValue24Bit), float64(roundedValue))))

	// Convert the clipped value to a byte array in little-endian format
	return []byte{
		byte(clippedValue & 0xFF),
		byte((clippedValue >> 8) & 0xFF),
		byte((clippedValue >> 16) & 0xFF),
	}
}

// convertTo32BitSample converts a float64 sample to 32-bit PCM.
func (we *FoxEncoder) convertTo32BitSample(sample float64) []byte {
	const bitDepth = 32
	const maxValue32Bit = 2147483647.0
	const minValue32Bit = -2147483648.0

	// Scale the sample to the range of 32-bit signed integers
	scaledValue := sample * float64(maxValue32Bit)

	// Round to the nearest integer
	roundedValue := int64(math.Round(scaledValue))

	// Clip the value to the valid range
	clippedValue := int64(math.Max(float64(minValue32Bit), math.Min(float64(maxValue32Bit), float64(roundedValue))))

	// Convert the clipped value to a byte array in little-endian format
	return []byte{
		byte(clippedValue & 0xFF),
		byte((clippedValue >> 8) & 0xFF),
		byte((clippedValue >> 16) & 0xFF),
		byte((clippedValue >> 24) & 0xFF),
	}
}
