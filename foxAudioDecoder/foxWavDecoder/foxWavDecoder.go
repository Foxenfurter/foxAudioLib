// pkg for encoding a stream into wav format. The package has been specified to allow a header to be written in the initial phase
// the body to be following this. The package is expected to be called as part of an encoder function. which will run asynchronously
package foxWavDecoder

import (
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"strconv"
)

// Structure holds basic information about the Samples to be encoded
// this is a type neutral representation of the WavHeader structure
type FoxDecoder struct {
	SampleRate   int
	BitDepth     int
	NumChannels  int
	Size         int
	ReaderCursor int
}

type wavHeader struct {
	ChunkID       [4]byte
	ChunkSize     uint32
	Format        [4]byte
	Subchunk1ID   [4]byte
	Subchunk1Size uint32
	AudioFormat   uint16
	NumChannels   uint16
	SampleRate    uint32
	ByteRate      uint32
	BlockAlign    uint16
	BitDepth      uint16
	Subchunk2ID   [4]byte
	Size          uint32
}

const packageName = "foxWavDecoder"

//	Getters and Setters
//
// ------------Getters----------------------------------------
// GetSampleRate returns the sample rate of the FoxDecoder.
func (fd *FoxDecoder) GetSampleRate() int {
	return fd.SampleRate
}

// GetBitDepth returns the bit depth of the FoxDecoder.
func (fd *FoxDecoder) GetBitDepth() int {
	return fd.BitDepth
}

// GetNumChannels returns the number of channels in the FoxDecoder.
func (fd *FoxDecoder) GetNumChannels() int {
	return fd.NumChannels
}

// GetSize returns the size of the FoxDecoder.
func (fd *FoxDecoder) GetSize() int {
	return fd.Size
}

// GetReaderCursor returns the reader cursor position of the FoxDecoder.
func (fd *FoxDecoder) GetReaderCursor() int {
	return fd.ReaderCursor
}

// --------------Setters--------------------------------
// SetSampleRate sets the SampleRate field of FoxDecoder.
func (fd *FoxDecoder) SetSampleRate(sampleRate int) {
	fd.SampleRate = sampleRate
}

// SetBitDepth sets the BitDepth field of FoxDecoder.
func (fd *FoxDecoder) SetBitDepth(bitDepth int) {
	fd.BitDepth = bitDepth
}

// SetNumChannels sets the NumChannels field of FoxDecoder.
func (fd *FoxDecoder) SetNumChannels(numChannels int) {
	fd.NumChannels = numChannels
}

// SetSize sets the Size field of FoxDecoder.
func (fd *FoxDecoder) SetSize(size int) {
	fd.Size = size
}

// SetReaderCursor sets the ReaderCursor field of FoxDecoder.
func (fd *FoxDecoder) SetReaderCursor(readerCursor int) {
	fd.ReaderCursor = readerCursor
}

// Decode the Header of a wav file from a byte array and return the header to the user
// FoxDecoder structure is being passed in because it may be required for other header processors
func (wd *FoxDecoder) DecodeHeader(data []byte) error {
	const functionName = "DecodeHeader"

	fmt.Println("foxWavDecoder: Decode Header starting...")

	var header wavHeader

	if len(data) < 44 {
		errorText := "invalid WAV file - length of header supplied: " + strconv.Itoa(len(data))
		return errors.New(packageName + ":" + functionName + ":" + errorText)
	}
	//fmt.Println("foxWavDecoder: Copying chunks...")
	copy(header.ChunkID[:], data[0:4])

	header.ChunkSize = binary.LittleEndian.Uint32(data[4:8])
	copy(header.Format[:], data[8:12])
	copy(header.Subchunk1ID[:], data[12:16])
	header.Subchunk1Size = binary.LittleEndian.Uint32(data[16:20])
	header.AudioFormat = binary.LittleEndian.Uint16(data[20:22])
	header.NumChannels = binary.LittleEndian.Uint16(data[22:24])
	header.SampleRate = binary.LittleEndian.Uint32(data[24:28])
	header.ByteRate = binary.LittleEndian.Uint32(data[28:32])
	header.BlockAlign = binary.LittleEndian.Uint16(data[32:34])
	header.BitDepth = binary.LittleEndian.Uint16(data[34:36])
	copy(header.Subchunk2ID[:], data[36:40])
	header.Size = binary.LittleEndian.Uint32(data[40:44])

	//readerCursor := header.ChunkSize + 44
	offsetToFirstSample := 12 + header.Subchunk1Size + 8 // Initial header, fmt subchunk & data header
	if offsetToFirstSample < 44 {                        // Account for potential padding
		offsetToFirstSample = 44
	}
	// Create and return FoxEncoder struct
	wd.SampleRate = int(header.SampleRate)
	wd.BitDepth = int(header.BitDepth)
	wd.NumChannels = int(header.NumChannels)
	wd.Size = int(header.Size)
	wd.ReaderCursor = int(offsetToFirstSample)
	//fmt.Println("foxWavDecoder: NumChannels: ", wd.NumChannels)
	return nil
	//return nil
}

// Decode the data section of a Wav File into 64 bit floats
func (decoder *FoxDecoder) DecodeData(byteStream []byte, numSamples int) ([][]float64, error) {
	const functionName = "DecodeData"
	// ... (error handling and calculations remain the same) ...
	// Create output buffer with float64 type
	output := make([][]float64, decoder.NumChannels)
	// set size for each channel
	for i := range output {
		output[i] = make([]float64, numSamples)
	}
	//println("foxWavDecoder: Number Samples: ", numSamples, "Number Channels: ", decoder.NumChannels, "Bitdepth:", decoder.BitDepth, "Cursor Position:", decoder.ReaderCursor)
	// Decode samples with float64 conversion
	switch decoder.BitDepth {
	case 16:
		err := decode16bitFloat64(byteStream, output, numSamples, decoder.NumChannels)
		if err != nil {
			return nil, err
		}
	case 24:
		decode24bitFloat64(byteStream, output, numSamples, decoder.NumChannels)
	case 32:
		decode32bitFloat64(byteStream, output, numSamples, decoder.NumChannels)
	default:
		{
			errorText := "unsupported bit depth"
			return nil, errors.New(packageName + ":" + functionName + ":" + errorText)
		}
	}
	//println("foxWavDecoder: Return Decoded data")
	return output, nil
}

// Decode 16-bit samples with float64 conversion
func decode16bitFloat64(byteStream []byte, output [][]float64, numSamples int, numChannels int) error {

	const functionName = "decode16bitFloat64"
	// Check for sufficient 16 bit bytes in the byteStream
	expectedLength := numSamples * numChannels * 2
	if len(byteStream) < expectedLength {
		errorText := "byteStream is too short for the given numSamples and numChannels"
		return errors.New(packageName + ":" + functionName + ":" + errorText)

	}

	for i := 0; i < numSamples; i++ {

		for j := 0; j < numChannels; j++ {
			offset := (i*numChannels + j) * 2
			output[j][i] = float64(int16(binary.LittleEndian.Uint16(byteStream[offset:offset+2]))) / 32768.0
		}
	}
	return nil
}

// Decode 24-bit samples (assuming padding to 32 bits) with float64 conversion
func decode24bitFloat64(byteStream []byte, output [][]float64, numSamples int, numChannels int) error {

	const functionName = "decode24bitFloat64"
	// Calculate the expected length of the byte stream
	expectedLength := numSamples * numChannels * 3 // 24 bits (3 bytes) per sample per channel

	// Check if the byte stream is long enough
	if len(byteStream) < expectedLength {
		errorText := "byteStream is too short for the given numSamples and numChannels"
		return errors.New(packageName + ":" + functionName + ":" + errorText)
	}
	for i := 0; i < numSamples; i++ {
		for j := 0; j < numChannels; j++ {
			offset := i*numChannels + j
			value := binary.LittleEndian.Uint32(byteStream[offset*4 : offset*4+4])
			output[j][i] = float64(value>>8) / 8388608.0
		}
	}
	return nil
}

// Decode 32-bit floating-point samples with float64 conversion
func decode32bitFloat64(byteStream []byte, output [][]float64, numSamples int, numChannels int) error {

	const functionName = "decode32bitFloat64"
	// Calculate the expected length of the byte stream
	expectedLength := numSamples * numChannels * 4 // 32 bits (4 bytes) per sample per channel

	// Check if the byte stream is long enough
	if len(byteStream) < expectedLength {
		errorText := "byteStream is too short for the given numSamples and numChannels"
		return errors.New(packageName + ":" + functionName + ":" + errorText)

	}
	for i := 0; i < numSamples; i++ {
		for j := 0; j < numChannels; j++ {
			offset := (i*numChannels + j) * 4
			output[j][i] = math.Float64frombits(uint64(binary.LittleEndian.Uint32(byteStream[offset : offset+4])))
		}
	}
	return nil
}