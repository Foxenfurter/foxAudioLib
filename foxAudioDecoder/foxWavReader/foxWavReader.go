// Package: github.com/Foxenfurter/foxAudioLib/foxAudioDecoder/foxWavReader
// file foxWavReader.go
// pkg for encoding a stream into wav format. The package has been specified to allow a header to be written in the initial phase
// the body to be following this. The package is expected to be called as part of an encoder function. which will run asynchronously
package foxWavReader

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"sync"
	"time"

	"github.com/google/uuid"
)

// Structure holds basic information about the Samples to be encoded
// this is a type neutral representation of the WavHeader structure
type WavReader struct {
	SampleRate     int
	BitDepth       int
	NumChannels    int
	Size           uint32
	ReaderCursor   int
	LittleEndian   bool
	ByteOrder      binary.ByteOrder
	wordLength     int
	bytesPerSample int
	Input          io.Reader // Changed from *os.File to io.Reader to *bufio.Reader back to io.Reader
	AudioFormat    WaveFormat
	DebugFunc      func(string)
	// additional for PCM
	leftoverBytes []byte
}

// Holds detailed information about the wav Header
type WaveFile struct {
	ok            bool
	pos           int64
	dataOffset    int64
	riff          string
	length        uint32
	wave          string
	format        string
	size          uint32
	audioFormat   WaveFormat
	numChannels   uint16
	sampleRate    uint32
	byteRate      uint32
	blockAlign    uint16
	bitsPerSample uint16
	data          string
	dataSize      uint32
	max           uint32
	channelMask   uint32
	formatEx      WaveFormatEx
	bigEndian     bool
}

const packageName = "FoxWavReader"

//	Getters and Setters
//
// ------------Getters----------------------------------------
// GetSampleRate returns the sample rate of the FoxDecoder.
func (fd *WavReader) GetSampleRate() int {
	return fd.SampleRate
}

// GetBitDepth returns the bit depth of the FoxDecoder.
func (fd *WavReader) GetBitDepth() int {
	return fd.BitDepth
}

// GetNumChannels returns the number of channels in the FoxDecoder.
func (fd *WavReader) GetNumChannels() int {
	return fd.NumChannels
}

// GetSize returns the size of the FoxDecoder.
func (fd *WavReader) GetSize() uint32 {
	return fd.Size
}

// GetReaderCursor returns the reader cursor position of the FoxDecoder.
func (fd *WavReader) GetReaderCursor() int {
	return fd.ReaderCursor
}

// SetEndianess sets the Endianess field of FoxDecoder.
func (fd *WavReader) GetLittleEndian() bool {
	return fd.LittleEndian

}

// --------------Setters--------------------------------
// SetSampleRate sets the SampleRate field of FoxDecoder.
func (fd *WavReader) SetSampleRate(sampleRate int) {
	fd.SampleRate = sampleRate
}

// SetBitDepth sets the BitDepth field of FoxDecoder.
func (fd *WavReader) SetBitDepth(bitDepth int) {
	fd.BitDepth = bitDepth
}

// SetNumChannels sets the NumChannels field of FoxDecoder.
func (fd *WavReader) SetNumChannels(numChannels int) {
	fd.NumChannels = numChannels
}

// SetSize sets the Size field of FoxDecoder.
func (fd *WavReader) SetSize(size uint32) {
	fd.Size = size
}

// SetReaderCursor sets the ReaderCursor field of FoxDecoder.
func (fd *WavReader) SetReaderCursor(readerCursor int) {
	fd.ReaderCursor = readerCursor
}

// SetEndianess sets the Endianess field of FoxDecoder.
func (fd *WavReader) SetLittleEndian(endianNess bool) {
	fd.LittleEndian = endianNess
}

const scale8Bit float64 = 1 / 128.0
const scale16Bit float64 = 1 / 32768.0
const scale24Bit float64 = 1 / 8388608.0
const scale32Bit float64 = 1 / 2147483648.0

var samplebyte byte
var sampleINT8 int8
var sampleINT16 int16
var sampleUINT16 uint16
var sampleINT32 int32
var sampleFLOAT32 float32
var sampleFLOAT64 float64

// Byte Converter
func (FD *WavReader) ConvertBytesToFloat64(myBytes []byte) ([][]float64, error) {
	//functionName:="ConvertBytesToFloat64"
	//Calculate the expected number of samples from the input buffer NB we are talking frame samples here
	//copy channels as a minor optimization
	numChannels := FD.NumChannels
	numSamples := uint32(len(myBytes) / ((FD.BitDepth / 8) * (numChannels)))

	byteReader := bytes.NewReader(myBytes)

	samples := make([][]float64, numChannels)
	for s := range samples {
		samples[s] = make([]float64, numSamples)
	}

	var index int = 0

	//println("Expected number of samples: ", numSamples, len(samples[0]))
	for s := uint32(0); s < numSamples; s++ {
		for c := 0; c < numChannels; c++ {
			switch FD.BitDepth {
			case 8:

				binary.Read(byteReader, binary.LittleEndian, &samplebyte)
				// 8-bit PCM uses unsigned bytes
				samples[c][s] = float64(samplebyte-128) * scale8Bit
			case 16:
				// index pos in array is ( (current sample number * channels) + current channel number) * number of bytes in sample
				index = ((int(s) * numChannels) + c) * 2

				if FD.LittleEndian {
					sampleINT16 = (int16(myBytes[index+1]) << 8) | int16(myBytes[index])
				} else {
					//big endian
					sampleINT16 = (int16(myBytes[index]) << 8) | int16(myBytes[index+1])
				}

				samples[c][s] = float64(sampleINT16) * scale16Bit

			case 18:

				binary.Read(byteReader, FD.ByteOrder, &sampleINT16)
				samples[c][s] = float64(sampleINT16) * scale16Bit
			case 24:
				// index pos in array is ( (current sample number * channels) + current channel number) * number of bytes in sample
				index = ((int(s) * numChannels) + c) * 3
				if FD.LittleEndian {
					sampleINT32 = int32(int8(myBytes[index+2]))<<16 | int32(myBytes[index+1])<<8 | int32(myBytes[index])
				} else {
					// big endian
					//sampleINT32 = int32(myBytes[index])<<16 | int32(myBytes[index+1])<<8 | int32(myBytes[index+2])
					// Big-endian: [MSB][Mid][LSB]
					raw := (int32(myBytes[index]) << 24) |
						(int32(myBytes[index+1]) << 16) |
						(int32(myBytes[index+2]) << 8)
					sampleINT32 = raw >> 8 // Sign-extend
				}
				samples[c][s] = float64(sampleINT32) * scale24Bit
			case 26:
				// kept for reference
				binary.Read(byteReader, binary.LittleEndian, &sampleUINT16)
				binary.Read(byteReader, binary.LittleEndian, &sampleINT8)

				// Combine a and b into a 24-bit signed integer
				sampleINT32 = (int32(sampleINT8) << 16) + int32(sampleUINT16)

				// Calculate the final value
				samples[c][s] = float64(sampleINT32) * scale24Bit

			case 32:
				if FD.AudioFormat == PCM {
					binary.Read(byteReader, binary.LittleEndian, &sampleINT32)
					samples[c][s] = float64(sampleINT32) * scale32Bit
				} else {

					// Read 4 bytes and interpret as an IEEE 754 float32
					binary.Read(byteReader, binary.LittleEndian, &sampleFLOAT32)
					samples[c][s] = float64(sampleFLOAT32)
				}
			case 64:

				if (FD.AudioFormat == IEEE_FLOAT) || (FD.AudioFormat == INTERNAL_DOUBLE) {

					binary.Read(byteReader, binary.LittleEndian, &sampleFLOAT64)

				} else {
					// throw new Exception("64-bit PCM not handled");
					sampleFLOAT64 = 0
				}
				samples[c][s] = sampleFLOAT64
			}

		}
	}
	return samples, nil
}

// Byte Converter
func (FD *WavReader) ConvertBytesToFloat64amended(myBytes []byte) ([][]float64, error) {
	//functionName:="ConvertBytesToFloat64"
	//Calculate the expected number of samples from the input buffer NB we are talking frame samples here
	//copy channels as a minor optimization
	sampleSize := (FD.BitDepth / 8) * FD.NumChannels
	if len(myBytes)%sampleSize != 0 {
		return nil, fmt.Errorf("invalid data length: %d (expected multiple of %d)", len(myBytes), sampleSize)
	}

	numChannels := FD.NumChannels
	numSamples := uint32(len(myBytes) / ((FD.BitDepth / 8) * (numChannels)))

	byteReader := bytes.NewReader(myBytes)

	samples := make([][]float64, numChannels)
	for s := range samples {
		samples[s] = make([]float64, numSamples)
	}

	var index int = 0

	//println("Expected number of samples: ", numSamples, len(samples[0]))
	for s := uint32(0); s < numSamples; s++ {
		for c := 0; c < numChannels; c++ {
			switch FD.BitDepth {
			case 8:

				binary.Read(byteReader, binary.LittleEndian, &samplebyte)
				// 8-bit PCM uses unsigned bytes
				samples[c][s] = float64(samplebyte-128) * scale8Bit
			case 16:
				// index pos in array is ( (current sample number * channels) + current channel number) * number of bytes in sample
				index = ((int(s) * numChannels) + c) * 2

				if FD.LittleEndian {
					sampleINT16 = (int16(myBytes[index+1]) << 8) | int16(myBytes[index])
				} else {
					//big endian
					sampleINT16 = (int16(myBytes[index]) << 8) | int16(myBytes[index+1])
				}

				samples[c][s] = float64(sampleINT16) * scale16Bit

			case 18:

				binary.Read(byteReader, FD.ByteOrder, &sampleINT16)
				samples[c][s] = float64(sampleINT16) * scale16Bit
			case 24:
				// index pos in array is ( (current sample number * channels) + current channel number) * number of bytes in sample
				index = ((int(s) * numChannels) + c) * 3
				if FD.LittleEndian {
					sampleINT32 = int32(int8(myBytes[index+2]))<<16 | int32(myBytes[index+1])<<8 | int32(myBytes[index])
				} else {
					// big endian
					//sampleINT32 = int32(myBytes[index])<<16 | int32(myBytes[index+1])<<8 | int32(myBytes[index+2])
					// Big-endian: [MSB][Mid][LSB]
					raw := (int32(myBytes[index]) << 24) |
						(int32(myBytes[index+1]) << 16) |
						(int32(myBytes[index+2]) << 8)
					sampleINT32 = raw >> 8 // Sign-extend
				}
				samples[c][s] = float64(sampleINT32) * scale24Bit
			case 26:
				// kept for reference
				binary.Read(byteReader, binary.LittleEndian, &sampleUINT16)
				binary.Read(byteReader, binary.LittleEndian, &sampleINT8)

				// Combine a and b into a 24-bit signed integer
				sampleINT32 = (int32(sampleINT8) << 16) + int32(sampleUINT16)

				// Calculate the final value
				samples[c][s] = float64(sampleINT32) * scale24Bit

			case 32:
				if FD.AudioFormat == PCM {
					binary.Read(byteReader, binary.LittleEndian, &sampleINT32)
					samples[c][s] = float64(sampleINT32) * scale32Bit
				} else {

					// Read 4 bytes and interpret as an IEEE 754 float32
					binary.Read(byteReader, binary.LittleEndian, &sampleFLOAT32)
					samples[c][s] = float64(sampleFLOAT32)
				}
			case 64:

				if (FD.AudioFormat == IEEE_FLOAT) || (FD.AudioFormat == INTERNAL_DOUBLE) {

					binary.Read(byteReader, binary.LittleEndian, &sampleFLOAT64)

				} else {
					// throw new Exception("64-bit PCM not handled");
					sampleFLOAT64 = 0
				}
				samples[c][s] = sampleFLOAT64
			}

		}
	}
	return samples, nil
}

// DecodePCMInput reads raw PCM input and sends decoded samples to the channel
// Handles partial samples and EOF properly for headerless PCM streams
// DecodePCMInput reads raw PCM input and sends decoded samples to the channel
// ***** WARNING this function craps out super early****//
func (FD *WavReader) DecodePCMInput(DecodedSamplesChannel chan [][]float64) error {
	//	functionName := "DecodePCMInput"
	//start := time.Now()//
	defer close(DecodedSamplesChannel)

	sampleSize := (FD.BitDepth / 8) * FD.NumChannels
	if sampleSize == 0 {
		return fmt.Errorf("invalid sample size")
	}

	// 1. Verify stream is seekable and get exact size
	var totalInputBytes int64
	if seeker, ok := FD.Input.(io.Seeker); ok {
		size, err := seeker.Seek(0, io.SeekEnd)
		if err == nil {
			_, _ = seeker.Seek(0, io.SeekStart) // Reset position
			totalInputBytes = size
			FD.debug(fmt.Sprintf("Input size: %d bytes (%d samples)",
				size, size/int64(sampleSize)))
		}
	}

	// 2. Use buffered reader for reliable partial reads
	bufReader := bufio.NewReaderSize(FD.Input, 65536)
	var (
		leftover     = make([]byte, 0, sampleSize*2)
		totalSamples int
	)

	for {
		// 3. Read through buffer for partial read protection
		data := leftover
		for len(data) < sampleSize {
			chunk, err := bufReader.Peek(bufReader.Size())
			if len(chunk) > 0 {
				data = append(data, chunk...)
				bufReader.Discard(len(chunk))
			}
			if err != nil {
				if err == io.EOF {
					break
				}
				return fmt.Errorf("read error: %w", err)
			}
		}

		// 4. Process complete samples
		fullSamples := len(data) / sampleSize
		if fullSamples > 0 {
			samples, err := FD.ConvertBytesToFloat64(data[:fullSamples*sampleSize])
			if err != nil {
				return fmt.Errorf("conversion error: %w", err)
			}

			select {
			case DecodedSamplesChannel <- samples:
				totalSamples += len(samples[0])
			case <-time.After(1 * time.Second):
				return fmt.Errorf("channel timeout")
			}
		}

		// 5. Save leftovers using circular buffer
		leftover = data[fullSamples*sampleSize:]
		if len(leftover) >= sampleSize*2 {
			copy(leftover, leftover[len(leftover)-sampleSize*2:])
			leftover = leftover[:sampleSize*2]
		}

		// 6. Verify EOF condition strictly
		if _, err := bufReader.Peek(1); err == io.EOF {
			if len(leftover) > 0 {
				FD.debug(fmt.Sprintf("Final partial sample: %X", leftover))
			}
			break
		}
	}

	// 7. Validate input completeness
	if totalInputBytes > 0 {
		expected := totalInputBytes / int64(sampleSize)
		if int64(totalSamples) != expected {
			return fmt.Errorf("input incomplete: expected %d samples, got %d",
				expected, totalSamples)
		}
	}

	FD.debug(fmt.Sprintf("Completed. Samples: %d", totalSamples))
	return nil
}

// Function should resume reading the input stream and pass the resulting samples to the output Channel

func (FD *WavReader) DecodeInput(DecodedSamplesChannel chan [][]float64) error {
	functionName := "DecodeInput"
	start := time.Now()
	TotalBytes := 0
	TotalSamples := 0                                                            //1 channel
	processingBufferSize := (FD.NumChannels * FD.SampleRate) * (FD.BitDepth / 8) // one second
	processingBufferSize = processingBufferSize / 6

	FD.debug(fmt.Sprintf("Starting decode at position: %d, expected data size: %d", FD.ReaderCursor, FD.Size))
	// Smaller buffer for accumulating data
	// No need to Flush the processing buffer as we are re-using and controlling the read cursor
	// reading data from the input buffer
	readBufferSize := 4096 // Tested vs StdIn via piped process (flac | this process ) and anything too big slows to a crawl.
	//readBufferSize = 8192

	if readBufferSize > processingBufferSize { //We don't want the read buffer to be bigger than the processing buffer
		readBufferSize = processingBufferSize
	}
	readBuffer := make([]byte, readBufferSize)
	processingBuffer := make([]byte, processingBufferSize)
	filledBytes := 0

	rowcounter := 0
	EOF := false
	for {
		// Read into the read buffer
		n, err := FD.Input.Read(readBuffer[:readBufferSize])

		if err != nil {
			if err != io.EOF {
				ErrorText := packageName + ":" + functionName + " Error reading: " + err.Error()
				return errors.New(ErrorText)
			}
			// EOF reached
			FD.debug(fmt.Sprintf(packageName + ":" + functionName + " EOF reached"))
			EOF = true

		}
		FD.ReaderCursor += int(n)
		// Check if enough space in the processing buffer
		if filledBytes+n > processingBufferSize {

			// copy as much as we can
			readCursor := (processingBufferSize - filledBytes)
			copy(processingBuffer[filledBytes:], readBuffer[:readCursor])
			// are these the bytes you are looking for ?
			filledBytes += readCursor

			// convert the bytes to Samples of the correct bitDepth

			mySamples, err := FD.ConvertBytesToFloat64(processingBuffer[:filledBytes])
			if err != nil {

				ErrorText := packageName + ":" + functionName + "Error Decoding Bytes: " + err.Error()
				return errors.New(ErrorText)

			}
			TotalSamples += len(mySamples[0])
			DecodedSamplesChannel <- mySamples

			TotalBytes += filledBytes
			filledBytes = 0

			// Check for remaining data in the read buffer (if not EOF)
			if n > 0 {
				copy(processingBuffer, readBuffer[readCursor:n]) // Move remaining data
				filledBytes += n - readCursor
			}
		} else { //
			copy(processingBuffer[filledBytes:], readBuffer[:n])
			filledBytes += n
		}

		rowcounter++
		if EOF {
			// we may have some data left in the processing buffer!
			//fmt.Println("Bytes read:", n, ": Read Buffer Size:", len(readBuffer), ":fill bytes:", filledBytes)
			if filledBytes > 0 {

				mySamples, err := FD.ConvertBytesToFloat64(processingBuffer[:filledBytes])
				if err != nil {
					ErrorText := packageName + ":" + functionName + "Error Decoding Bytes: " + err.Error()
					return errors.New(ErrorText)

				}
				TotalSamples += len(mySamples[0])
				DecodedSamplesChannel <- mySamples

				TotalBytes += filledBytes
				filledBytes = 0

			}
			break
		}
	}
	// We are done so close the channel
	os.Stdin.Sync()
	close(DecodedSamplesChannel)

	elapsedTime := time.Since(start).Milliseconds()
	FD.debug(fmt.Sprintf(packageName+":"+functionName+"Expected Bytes: %v, Tracked bytes: %v, Total bytes read: %v Total samples read: %v Elapsed time (ms): %v ", FD.Size, FD.ReaderCursor, TotalBytes, TotalSamples, elapsedTime))
	return nil
}

// **************Header as per old Inguz c# library************************

type WaveFormatEx struct {
	GUID [16]byte
}

var (
	waveFormatExPCM                      = NewWaveFormatExString("1f0e8ffb-9db2-4f16-a2ef-47481f5dc9e5")
	waveFormatExIEEEFloat                = NewWaveFormatExString("d478b09e-bcb9-4038-afe1-def76ec94512")
	AMBISONIC_B_FORMAT_PCMPCMguid        = NewWaveFormatExString("e8f32218-3fc1-4361-9f41-abf9b3b72260")
	AMBISONIC_B_FORMAT_IEEE_FLOATPCMguid = NewWaveFormatExString("00000003-0721-11d3-8644-C8C1CA000000")
)

// NewWaveFormatExString creates a new WaveFormatEx instance with the specified GUID string.
func NewWaveFormatExString(guid string) WaveFormatEx {
	var waveFormatEx WaveFormatEx
	bytes, err := uuid.Parse(guid)
	if err != nil || len(bytes) != 16 {
		// Handle error appropriately (e.g., return default or panic)

		println(fmt.Sprintf("Invalid GUID: %s length GUID %v len bytes %v", guid, len(guid), len(bytes)))
	}

	copy(waveFormatEx.GUID[:], bytes[:])
	return waveFormatEx
}

// NewWaveFormatExBytes creates a new WaveFormatEx instance with the specified GUID byte array.
func NewWaveFormatExBytes(guid []byte) WaveFormatEx {
	if len(guid) != 16 {
		// Handle error appropriately (e.g., return default or panic)
		println("Invalid GUID byte array length")
	}

	var waveFormatEx WaveFormatEx
	copy(waveFormatEx.GUID[:], guid)
	return waveFormatEx
}

func (wf *WaveFormatEx) Equals(other *WaveFormatEx) bool {
	return wf.GUID == other.GUID
}

func (wf *WaveFormatEx) GetHashCode() int {
	return hash(wf.GUID[:])
}

func (wf *WaveFormatEx) String() string {
	return fmt.Sprintf("%x", wf.GUID)
}

func hash(bytes []byte) int {
	var result int
	for _, b := range bytes {
		result = (result << 5) - result + int(b)
	}
	return result
}

type WaveFormat uint16

const (
	PCM             WaveFormat = 1
	ADPCM           WaveFormat = 2
	IEEE_FLOAT      WaveFormat = 3
	INTERNAL_DOUBLE WaveFormat = 4
	EXTENSIBLE      WaveFormat = 0xFFFE
)

// Function to map uint16 to WaveFormat
func mapUint16ToWaveFormat(value uint16) WaveFormat {
	switch value {
	case 1:
		return PCM
	case 2:
		return ADPCM
	case 3:
		return IEEE_FLOAT
	case 4:
		return INTERNAL_DOUBLE
	case 0xFFFE:
		return EXTENSIBLE
	default:
		// Handle unknown values, you can return an error or a default value
		return PCM // Change to an appropriate default or handle the error accordingly
	}
}

// Reusable buffer pool for fixed-size reads (4 bytes for 32-bit values, 2 for 16-bit)
var bufferPool4 = sync.Pool{
	New: func() interface{} { return make([]byte, 4) },
}

var bufferPool2 = sync.Pool{
	New: func() interface{} { return make([]byte, 2) },
}

func (w *WaveFile) readBytesFromInput(myReader io.Reader, count int) ([]byte, error) {
	buffer := make([]byte, count)
	if _, err := io.ReadFull(myReader, buffer); err != nil {
		return nil, err
	}
	w.pos += int64(count)
	return buffer, nil
}

func (w *WaveFile) readCharsFromInput(myReader io.Reader, count int) (string, error) {
	buffer := make([]byte, count)
	if _, err := io.ReadFull(myReader, buffer); err != nil {
		return "", err
	}
	w.pos += int64(count)
	return string(buffer), nil
}

func (w *WaveFile) readInt32FromInput(myReader io.Reader) (int32, error) {
	buf := bufferPool4.Get().([]byte)
	defer bufferPool4.Put(buf)

	if _, err := io.ReadFull(myReader, buf); err != nil {
		return 0, err
	}
	w.pos += 4
	return int32(binary.LittleEndian.Uint32(buf)), nil
}

func (w *WaveFile) readUInt32FromInput(myReader io.Reader) (uint32, error) {
	buf := bufferPool4.Get().([]byte)
	defer bufferPool4.Put(buf)

	if _, err := io.ReadFull(myReader, buf); err != nil {
		return 0, err
	}
	w.pos += 4
	return binary.LittleEndian.Uint32(buf), nil
}

func (w *WaveFile) readUInt16FromInput(myReader io.Reader) (uint16, error) {
	buf := bufferPool2.Get().([]byte)
	defer bufferPool2.Put(buf)

	if _, err := io.ReadFull(myReader, buf); err != nil {
		return 0, err
	}
	w.pos += 2
	return binary.LittleEndian.Uint16(buf), nil
}

/* Commented out in order to test possibly improved versions
func (w *WaveFile) readBytesFromInput(myReader io.Reader, count int) ([]byte, error) {
	functionName := "readBytesFromInput"
	buffer := make([]byte, count)

	n, err := myReader.Read(buffer) // Read directly from the file
	if err != nil {
		return nil, err
	}

	if n < count {
		return nil, fmt.Errorf(packageName+functionName+" unexpected short read: got %d bytes, expected %d", n, count)
	}

	w.pos += int64(n) // Advance the file position
	return buffer, nil
}

func (w *WaveFile) readCharsFromInput(myReader io.Reader, count int) (string, error) {
	functionName := "readCharsFromInput"
	buffer := make([]byte, count)

	n, err := myReader.Read(buffer) // Read directly from the file
	if err != nil {
		return "", err
	}

	if n < count {
		return "", fmt.Errorf(packageName+functionName+" unexpected short read: got %d bytes, expected %d", n, count)
	}

	w.pos += int64(n) // Advance the file position
	return string(buffer), nil
}

func (w *WaveFile) readInt32FromInput(myReader io.Reader) (int32, error) {
	functionName := "readInt32FromInput"
	buffer := make([]byte, 4) // Fixed size for int32

	n, err := myReader.Read(buffer) // Read into buffer

	if err != nil {
		return 0, err
	}

	if n < 4 {
		return 0, fmt.Errorf(packageName+functionName+" unexpected short read: got %d bytes, expected 4", n)
	}

	var value int32
	binary.Read(bytes.NewReader(buffer), binary.LittleEndian, &value) // Decode from buffer

	w.pos += int64(n)
	return value, nil
}

func (w *WaveFile) readUInt32FromInput(myReader io.Reader) (uint32, error) {
	functionName := "readUInt32FromInput"
	buffer := make([]byte, 4) // Fixed size for int32

	n, err := myReader.Read(buffer) // Read into buffer

	if err != nil {
		return 0, err
	}

	if n < 4 {
		return 0, fmt.Errorf(packageName+functionName+" unexpected short read: got %d bytes, expected 4", n)
	}

	var value uint32
	binary.Read(bytes.NewReader(buffer), binary.LittleEndian, &value) // Decode from buffer

	w.pos += int64(n)
	return value, nil
}

func (w *WaveFile) readUInt16FromInput(myReader io.Reader) (uint16, error) {
	functionName := "readUInt16FromInput"
	var value uint16
	buffer := make([]byte, 2) // uint16 is 2 bytes

	n, err := myReader.Read(buffer) // Read directly from the file

	if err != nil {
		return 0, err
	}

	if n < 2 {
		return 0, fmt.Errorf(packageName+functionName+" :unexpected short read: got %d bytes, expected 2", n)
	}

	reader := bytes.NewReader(buffer) // Use a reader for binary.Read
	if err := binary.Read(reader, binary.LittleEndian, &value); err != nil {
		return 0, err
	}

	w.pos += int64(n) // Advance the file position

	return value, nil
}
*/
// Read Wav File Header
func (fd *WavReader) DecodeWavHeader() error {
	functionName := "DecodeWavHeader"
	var w WaveFile
	var err error
	w.ok = false
	w.pos = 0
	w.dataOffset = 0
	// New counter

	// Read the first 12 bytes (RIFF header + size + WAVE)
	header := make([]byte, 12)
	if _, err := io.ReadFull(fd.Input, header); err != nil {
		return err
	}
	w.riff = string(header[:4])
	fileLen := binary.LittleEndian.Uint32(header[4:8])
	w.wave = string(header[8:12])
	w.pos = 12
	/*
		hdr, err := w.readCharsFromInput(fd.Input, 4)
		if err != nil {
			return err
		}

		w.riff = hdr
	*/
	if w.riff != "RIFF" && w.riff != "FORM" {
		if len(w.riff) == 0 {
			return errors.New(packageName + functionName + ": file could not be read: no data")
		}
		x := ""
		for j := 0; j < len(w.riff); j++ {
			x += fmt.Sprintf("%X ", w.riff[j])
		}
		return fmt.Errorf(packageName+functionName+": file is not WAV: no 'RIFF' tag found, instead '%s' ", x)
	}

	// File length
	/*fileLen, err := w.readInt32FromInput(fd.Input)

	if err != nil {
		return err
	}*/
	w.length = uint32(fileLen)

	// Read Wave
	/*
		wave, err := w.readCharsFromInput(fd.Input, 4)
		if err != nil {
			return err
		}

		w.wave = wave
	*/
	// end of optimise

	if w.wave != "WAVE" && w.wave != "AIFF" {
		return fmt.Errorf(packageName+functionName+": file is not WAV: no 'WAVE' tag found, instead %s", w.wave)
	}
	if w.wave == "AIFF" {
		// The whole file is big-endian, including lengths in the header
		w.bigEndian = true
		fileLenBytes := make([]byte, 4)
		binary.BigEndian.PutUint32(fileLenBytes, uint32(fileLen))
		w.length = binary.BigEndian.Uint32(fileLenBytes)
	}
	// Read Format
	w.format, err = w.readCharsFromInput(fd.Input, 4)

	if err != nil {
		return err
	}

	for w.format != "fmt " && w.format != "COMM" {
		chunkSize, err := w.readInt32FromInput(fd.Input)
		if err != nil {
			return err
		}
		if w.bigEndian {
			chunkSizeBytes := make([]byte, 4)
			binary.BigEndian.PutUint32(chunkSizeBytes, uint32(chunkSize))
			chunkSize = int32(binary.BigEndian.Uint32(chunkSizeBytes))
		}

		if w.dataOffset+int64(chunkSize) > int64(w.length) {
			return errors.New(packageName + functionName + ": file could not be read: invalid 'fmt' chunk size")
		}

		//_, err = w.readBytesFromInput(fd.Input, int(chunkSize))
		// Skip chunkSize bytes without buffering
		_, err = io.CopyN(io.Discard, fd.Input, int64(chunkSize))
		w.pos += int64(chunkSize) // Update position tracking if needed

		if err != nil {
			return err
		}
		w.format, err = w.readCharsFromInput(fd.Input, 4)
		if err != nil {
			return err
		}
	}

	// Continue with the 'fmt' chunk processing...
	if w.format == "fmt " {
		// WAV file-format chunk
		w.size, err = w.readUInt32FromInput(fd.Input)
		if err != nil {
			return err
		}
		if w.size < 16 {
			return errors.New(packageName + functionName + ": file could not be read: don't know how to read 'fmt' size " + fmt.Sprint(w.size))
		}

		// lookup audio format key
		audioFormatKey, err := w.readUInt16FromInput(fd.Input)
		if err != nil {
			return err
		}
		w.audioFormat = mapUint16ToWaveFormat(audioFormatKey)

		if w.audioFormat == PCM || w.audioFormat == ADPCM || w.audioFormat == IEEE_FLOAT ||
			w.audioFormat == INTERNAL_DOUBLE || w.audioFormat == EXTENSIBLE {
			w.numChannels, err = w.readUInt16FromInput(fd.Input)
			if err != nil {
				return err
			}
			w.sampleRate, err = w.readUInt32FromInput(fd.Input)
			if err != nil {
				return err
			}
			w.byteRate, err = w.readUInt32FromInput(fd.Input)
			if err != nil {
				return err
			}
			w.blockAlign, err = w.readUInt16FromInput(fd.Input)
			if err != nil {
				return err
			}
			w.bitsPerSample, err = w.readUInt16FromInput(fd.Input)
			if err != nil {
				return err
			}
			if w.size > 16 {
				skip := uint32(16)
				if w.audioFormat == EXTENSIBLE {
					_, err = w.readUInt16FromInput(fd.Input)

					if err != nil {
						return err
					}
					_, err = w.readUInt16FromInput(fd.Input)

					if err != nil {
						return err
					}
					w.channelMask, err = w.readUInt32FromInput(fd.Input)
					if err != nil {
						return err
					}

					guidBytes, err := w.readBytesFromInput(fd.Input, 16)
					if err != nil {
						return err
					}
					w.formatEx = NewWaveFormatExBytes(guidBytes)
					//		w.dataOffset += 16
					skip = 40

					if w.formatEx == waveFormatExPCM {
						w.audioFormat = PCM

					} else if w.formatEx == waveFormatExIEEEFloat {
						w.audioFormat = IEEE_FLOAT
					}
				}
				// Read and discard the rest of the 'fmt' structure
				_, err := w.readBytesFromInput(fd.Input, int(w.size-skip))

				if err != nil {
					return err
				}
			}
		} else {
			return errors.New(packageName + functionName + ": file could not be read: don't know how to read audio format " + fmt.Sprint(w.audioFormat))
		}

	} else if w.format == "COMM" {
		// Continue with the 'COMM' chunk processing...
		// AIFF file-format chunk
		w.size, err = w.readUInt32FromInput(fd.Input)
		if err != nil {
			return err
		}
		if w.size < 18 {
			return errors.New(packageName + functionName + ": file could not be read: don't know how to read 'COMM' size " + fmt.Sprint(w.size))
		}

		w.audioFormat = PCM
		w.numChannels, err = w.readUInt16FromInput(fd.Input)
		if err != nil {
			return err
		}
		//numFrames
		_, err := w.readUInt32FromInput(fd.Input)
		if err != nil {
			return err
		}
		w.bitsPerSample, err = w.readUInt16FromInput(fd.Input)
		if err != nil {
			return err
		}

		// SampleRate is 10-byte IEEE_extended format.
		ext, err := w.readBytesFromInput(fd.Input, 10)
		if err != nil {
			return err
		}
		if ext[0] == 64 && ext[1] == 14 && ext[2] == 172 && ext[3] == 68 {
			w.sampleRate = 44100
		} else if ext[0] == 64 && ext[1] == 14 && ext[2] == 187 && ext[3] == 128 {
			w.sampleRate = 48000
		} else if ext[0] == 64 && ext[1] == 15 && ext[2] == 187 && ext[3] == 128 {
			w.sampleRate = 96000
		} else if ext[0] == 64 && ext[1] == 16 && ext[2] == 187 && ext[3] == 128 {
			w.sampleRate = 192000
		} else {
			return errors.New(packageName + functionName + ": file could not be read: don't know how to interpret sample rate")
		}

		w.blockAlign = uint16((w.numChannels * w.bitsPerSample) / 8)
		w.byteRate = uint32(w.blockAlign) * w.sampleRate

		if w.size > 18 {
			// Read and discard the rest of the 'COMM' structure
			_, err := w.readBytesFromInput(fd.Input, int(w.size-18))

			if err != nil {
				return err
			}
		}

	} else {
		return fmt.Errorf(packageName+functionName+": file could not be read: no 'fmt' tag found, instead %s", w.format)
	}

	// Read Data
	w.data, err = w.readCharsFromInput(fd.Input, 4)
	if err != nil {
		return err
	}
	// Seek for data chunk
	for w.data != "data" && w.data != "SSND" {
		miscSize, err := w.readInt32FromInput(fd.Input)
		if err != nil {
			return err
		}
		if w.dataOffset+int64(miscSize) > int64(w.length) {
			return errors.New(packageName + functionName + ": file could not be read: invalid data chunk size")
		}
		if w.bigEndian {
			miscSizeBytes := make([]byte, 4)
			binary.BigEndian.PutUint32(miscSizeBytes, uint32(miscSize))
			miscSize = int32(binary.BigEndian.Uint32(miscSizeBytes))
		}

		// Skip chunkSize bytes without buffering
		_, err = io.CopyN(io.Discard, fd.Input, int64(miscSize))
		w.pos += int64(miscSize) // Update position tracking if needed
		//_, err = w.readBytesFromInput(fd.Input, int(miscSize))

		if err != nil {
			return err
		}
		w.data, err = w.readCharsFromInput(fd.Input, 4)
		if err != nil {
			return err
		}
		//	w.dataOffset += 4
	}
	w.dataSize, err = w.readUInt32FromInput(fd.Input)
	if err != nil {
		return err
	}
	// Convert dataSize to host order if BigEndian
	if w.bigEndian {
		dataSizeBytes := make([]byte, 4)
		binary.BigEndian.PutUint32(dataSizeBytes, w.dataSize)
		w.dataSize = binary.BigEndian.Uint32(dataSizeBytes)
	}

	// See if we can read this
	if w.numChannels > 0 {
		switch w.audioFormat {
		case PCM, EXTENSIBLE:
			switch w.bitsPerSample {
			case 8, 16, 24, 32:
				w.ok = true
			}
		case IEEE_FLOAT:
			switch w.bitsPerSample {
			case 32, 64:
				w.ok = true
			}
		case INTERNAL_DOUBLE:
			switch w.bitsPerSample {
			case 64:
				w.ok = true
			}
		}
	}

	if w.ok {
		w.max = uint32((uint32(w.dataSize) / (uint32(w.bitsPerSample / 8))) / uint32(w.numChannels))
		if w.dataSize == 4294967292 {
			fmt.Println(packageName + functionName + ": Wave file: unknown size from header")
			w.max = math.MaxUint32
		}
		if w.max == 0 {
			fmt.Println(packageName + functionName + ":Wave file: zero size from header")
			w.max = math.MaxUint32
		}
	}

	//Header has now been read...
	//let set common header values
	fd.NumChannels = int(w.numChannels)
	fd.BitDepth = int(w.bitsPerSample)
	fd.SampleRate = int(w.sampleRate)
	if w.bigEndian {
		fd.LittleEndian = false
		fd.ByteOrder = binary.BigEndian
	} else {
		fd.LittleEndian = true
		fd.ByteOrder = binary.LittleEndian
	}
	fd.debug("WavReader DecodeWavHeader: Offset:" + fmt.Sprint(w.dataOffset) + " Pos:" + fmt.Sprint(w.pos))
	fd.ReaderCursor = int(w.pos)
	fd.AudioFormat = w.audioFormat
	//fd.Size = w.length
	fd.Size = w.dataSize
	switch fd.BitDepth {
	case 16:
		fd.wordLength = 2
	case 24:
		fd.wordLength = 3
	case 32:
		fd.wordLength = 4
	case 64:
		fd.wordLength = 8
	default:
		fd.wordLength = 2
	}
	fd.bytesPerSample = fd.wordLength * fd.NumChannels

	return nil

}

// Function to handle debug calls, allowing for different logging implementations
func (myDecoder *WavReader) debug(message string) {

	if myDecoder.DebugFunc != nil {
		myDecoder.DebugFunc(message)
	} else { // if no external debug function available just print the message
		println(message)
	}

}
