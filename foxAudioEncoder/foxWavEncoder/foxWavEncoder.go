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
	"math/rand"
	"strconv"
)

// Structure holds basic information about the Samples to be encoded
type FoxEncoder struct {
	SampleRate  int
	BitDepth    int
	NumChannels int
	Size        int64
	prng        *rand.Rand
	ditherready bool
	ditherState []float64 // Per-channel error feedback (for noise shaping)
	ditherScale float64   // Precomputed dither amplitude
	noiseCoeff  float64   // Noise shaping feedback coefficient
	Peak        float64   // Peak value of the signal
}

const maxValue32Bit = 2147483647.0
const minValue32Bit = -2147483648.0
const maxValue24Bit = 8388607.0
const minValue24Bit = -8388608.0
const maxValue16Bit = 32767.0
const minValue16Bit = -32768.0

// Initialize in Dither for Wav encoder setup
func (we *FoxEncoder) InitDither() {
	we.ditherState = make([]float64, we.NumChannels)
	we.ditherScale = 1.0 / math.Pow(2, float64(we.BitDepth-1))
	we.noiseCoeff = 0.5 // Adjust between 0.25-0.7 for different curves
	we.ditherready = true
	we.prng = rand.New(rand.NewSource(12))
	we.Peak = 0.0
}

func (we *FoxEncoder) GetPeak() float64 {
	return we.Peak
}

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
	// Set dithered
	we.InitDither()
	//fmt.Println("foxWavEncoder: DataSize: ", uint32(dataSize))
	return headerBuffer.Bytes(), nil
}

func (we *FoxEncoder) EncodeDataOriginal(buffer [][]float64) ([]byte, error) {
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
	intValue := int32(math.Max(math.Min(math.Round(sample*float64(maxValue24Bit)), float64(maxValue24Bit)), float64(minValue24Bit)))

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
	intValue := int32(math.Max(math.Min(math.Round(sample*float64(maxValue32Bit)), float64(maxValue32Bit)), float64(minValue32Bit)))

	// Convert to byte array in little-endian format using bitwise operations
	return []byte{
		byte(intValue & 0xFF),
		byte((intValue >> 8) & 0xFF),
		byte((intValue >> 16) & 0xFF),
		byte((intValue >> 24) & 0xFF),
	}
}

func (we *FoxEncoder) encode16BitDithered(buffer []float64, encoded []byte) float64 {
	const (
		max            = 32768.0  // 2^15 (maximum positive 16-bit value)
		min            = -32768.0 // Minimum 16-bit value
		bytesPerSample = 2        // 16 bits = 2 bytes
	)
	invMax := 1.0 / max
	currentPeak := 0.0

	for i := 0; i < len(buffer); i++ {
		sample := buffer[i]

		// Track peak in this buffer
		if abs := math.Abs(sample); abs > currentPeak {
			currentPeak = abs
		}

		// Generate noise-shaped dither
		prevError := we.ditherState[0] // Single channel state
		ditherNoise := (we.prng.Float64() + we.prng.Float64() - 1.0) * we.ditherScale
		dithered := sample + ditherNoise - prevError

		// Scale to 16-bit range and clamp
		scaled := math.Round(dithered * max)
		scaled = math.Max(min, math.Min(max-1, scaled))

		// Quantize and compute error
		quantized := scaled * invMax
		error := dithered - quantized
		we.ditherState[0] = error * we.noiseCoeff

		// Write to output buffer (little-endian)
		binary.LittleEndian.PutUint16(encoded[i*bytesPerSample:], uint16(int16(scaled)))
	}

	return currentPeak
}

func (we *FoxEncoder) encode24BitDithered(buffer []float64, encoded []byte) float64 {
	const (
		max            = 8388608.0  // 2^23 (maximum positive 24-bit value)
		min            = -8388608.0 // Minimum 24-bit value
		bytesPerSample = 3          // 24 bits = 3 bytes
	)
	invMax := 1.0 / max
	currentPeak := 0.0

	for i := 0; i < len(buffer); i++ {
		sample := buffer[i]

		// Track peak in this buffer
		if abs := math.Abs(sample); abs > currentPeak {
			currentPeak = abs
		}

		// Generate noise-shaped dither
		prevError := we.ditherState[0]
		ditherNoise := (we.prng.Float64() + we.prng.Float64() - 1.0) * we.ditherScale
		dithered := sample + ditherNoise - prevError

		// Scale to 24-bit range and clamp
		scaled := math.Round(dithered * max)
		scaled = math.Max(min, math.Min(max-1, scaled))

		// Quantize and compute error
		quantized := scaled * invMax
		error := dithered - quantized
		we.ditherState[0] = error * we.noiseCoeff

		// Write 24-bit (little-endian with masking)
		pos := i * bytesPerSample
		scaledInt := int32(scaled)
		encoded[pos] = byte(scaledInt)
		encoded[pos+1] = byte(scaledInt >> 8)
		encoded[pos+2] = byte((scaledInt >> 16) & 0xFF) // Mask upper bits
	}

	return currentPeak
}

func (we *FoxEncoder) encode32BitDithered(buffer []float64, encoded []byte) float64 {
	const (
		max            = 2147483648.0 // 2^31 (maximum positive 32-bit value)
		min            = -2147483648.0
		bytesPerSample = 4 // 32 bits = 4 bytes
	)
	invMax := 1.0 / max
	currentPeak := 0.0

	for i := 0; i < len(buffer); i++ {
		sample := buffer[i]

		// Track peak in this buffer
		if abs := math.Abs(sample); abs > currentPeak {
			currentPeak = abs
		}

		// Generate noise-shaped dither
		prevError := we.ditherState[0]
		ditherNoise := (we.prng.Float64() + we.prng.Float64() - 1.0) * we.ditherScale
		dithered := sample + ditherNoise - prevError

		// Scale to 32-bit range and clamp
		scaled := math.Round(dithered * max)
		scaled = math.Max(min, math.Min(max-1, scaled))

		// Quantize and compute error
		quantized := scaled * invMax
		error := dithered - quantized
		we.ditherState[0] = error * we.noiseCoeff

		// Write 32-bit (little-endian)
		binary.LittleEndian.PutUint32(encoded[i*bytesPerSample:], uint32(int32(scaled)))
	}

	return currentPeak
}

func (we *FoxEncoder) EncodeSingleChannel(buffer []float64) ([]byte, error) {
	totalSamples := len(buffer)
	if !we.ditherready {
		we.InitDither()
	}
	bytesPerSample := we.BitDepth / 8
	encoded := make([]byte, totalSamples*bytesPerSample)
	myPeak := 0.0

	switch we.BitDepth {
	case 16:
		myPeak = we.encode16BitDithered(buffer, encoded)
	case 24:
		myPeak = we.encode24BitDithered(buffer, encoded)
	case 32:
		myPeak = we.encode32BitDithered(buffer, encoded)
	default:
		return nil, errors.New("unsupported bit depth")
	}
	if myPeak > we.Peak {
		we.Peak = myPeak
	}
	return encoded, nil
}

// Encodes WAV and PCM data to 16, 24, or 32 bit with dithering
func (we *FoxEncoder) EncodeData(buffer [][]float64) ([]byte, error) {
	totalSamples := len(buffer[0])
	numChannels := we.NumChannels
	if !we.ditherready {
		we.InitDither()
	}
	// Validate input
	if len(buffer) != numChannels {
		return nil, fmt.Errorf("channel count mismatch")
	}

	bytesPerSample := we.BitDepth / 8
	totalBytes := totalSamples * numChannels * bytesPerSample
	encoded := make([]byte, totalBytes)
	localpeak := we.Peak
	var absval float64
	switch we.BitDepth {
	// only generate dither for 16 bit
	case 16:
		max := maxValue16Bit // 2^15 (not 32767!)
		min := minValue16Bit
		invMax := 1.0 / max

		for i := 0; i < totalSamples; i++ {
			basePos := i * numChannels * bytesPerSample
			for ch := 0; ch < numChannels; ch++ {
				//pos := (i*numChannels + ch) * bytesPerSample
				pos := basePos + ch*bytesPerSample
				//sample := math.Max(-1.0, math.Min(1.0, buffer[ch][i]))
				sample := buffer[ch][i]
				// get max level for reporting
				absval = math.Abs(sample)
				if absval > localpeak {
					localpeak = absval
				}
				// Generate noise-shaped dither
				prevError := we.ditherState[ch]

				ditherNoise := (we.prng.Float64() + we.prng.Float64() - 1.0) * we.ditherScale
				dithered := sample + ditherNoise - prevError

				// Scale and clamp to 16-bit range
				scaled := math.Round(dithered * max)
				if scaled < min {
					scaled = min
				} else if scaled > max-1 {
					scaled = max - 1
				}

				//	scaled = math.Max(min, math.Min(max-1, scaled)) // -32768 to 32767
				quantized := scaled * invMax

				// Update error state
				error := dithered - quantized
				we.ditherState[ch] = error * we.noiseCoeff

				binary.LittleEndian.PutUint16(encoded[pos:], uint16(int16(scaled)))
			}
		}

	case 24:
		// 24 bit
		for i := 0; i < totalSamples; i++ {
			basePos := i * numChannels * bytesPerSample
			for ch := 0; ch < numChannels; ch++ {
				//pos := (i*numChannels + ch) * bytesPerSample
				pos := basePos + ch*bytesPerSample
				//sample := math.Max(-1.0, math.Min(1.0, buffer[ch][i]))
				sample := buffer[ch][i]
				// get max level for reporting
				absval = math.Abs(sample)
				if absval > localpeak {
					localpeak = absval
				}

				sample = math.Max(-1.0, math.Min(1.0, sample))
				//faster than rounding
				scaled := int32(sample*maxValue24Bit + 0.5)
				// Write 24-bit with proper masking
				scaledInt := int32(scaled)
				encoded[pos] = byte(scaledInt)
				encoded[pos+1] = byte(scaledInt >> 8)
				encoded[pos+2] = byte((scaledInt >> 16) & 0xFF) // Mask upper bits
			}
		}
	case 32:
		max := maxValue32Bit // 2^31 (not 2147483647!)
		//min := minValue32Bit
		//invMax := 1.0 / max
		for i := 0; i < totalSamples; i++ {
			basePos := i * numChannels * bytesPerSample
			for ch := 0; ch < numChannels; ch++ {
				//pos := (i*numChannels + ch) * bytesPerSample
				pos := basePos + ch*bytesPerSample
				//sample := math.Max(-1.0, math.Min(1.0, buffer[ch][i]))
				sample := buffer[ch][i]
				// get max level for reporting
				absval = math.Abs(sample)
				if absval > localpeak {
					localpeak = absval
				}

				sample = math.Max(-1.0, math.Min(1.0, sample))
				//faster than rounding
				scaled := int32(sample*max + 0.5)
				binary.LittleEndian.PutUint32(encoded[pos:], uint32(scaled))

			}
		}
	case 64:

		for i := 0; i < totalSamples; i++ {
			basePos := i * numChannels * bytesPerSample
			for ch := 0; ch < numChannels; ch++ {
				//pos := (i*numChannels + ch) * bytesPerSample
				pos := basePos + ch*bytesPerSample
				//sample := math.Max(-1.0, math.Min(1.0, buffer[ch][i]))
				sample := buffer[ch][i]
				// get max level for reporting
				absval = math.Abs(sample)
				if absval > localpeak {
					localpeak = absval
				}

				binary.LittleEndian.PutUint64(encoded[pos:], uint64(sample))

			}
		}

	default:
		return nil, errors.New("unsupported bit depth")
	}
	we.Peak = localpeak
	return encoded, nil
}
