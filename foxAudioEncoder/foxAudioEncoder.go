// package calls an encoder based upon the supplied format
// and then processes the returned bytestream
// either to standardd out or to the supplied file name
package foxAudioEncoder

import (
	"bufio"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"

	foxWavEncoder "github.com/Foxenfurter/foxAudioLib/foxAudioEncoder/foxWavEncoder"
	"github.com/Foxenfurter/foxAudioLib/foxLog"
)

const packageName = "foxAudioEncoder"

// Encoder definition EncoderDefinition
type AudioEncoder struct {
	SampleRate  int
	BitDepth    int
	NumChannels int
	Size        int64
	Type        string
	Filename    string        // Added filename field
	file        *os.File      // Holds the open file handle
	writer      *bufio.Writer // Buffered writer for efficient writes

	DebugFunc  func(string) // enables the use of an external debug function supplied at the application level - expect to use foxLog
	DebugOn    bool         //enables debugging
	Encoder    EncoderInterface
	Peak       float64
	NumSamples int64
}

// each Encoder must have these methods defined
type EncoderInterface interface {
	EncodeHeader() ([]byte, error)
	EncodeData(samples [][]float64) ([]byte, error)
	EncodeSingleChannel(buffer []float64) ([]byte, error)
	GetPeak() float64
}

const (
	TargetBytesPerWrite = 8192 // Match socketwrapper's BUFFER_SIZE

)

// Constructor for Encoder
func (myEncoder *AudioEncoder) Initialise() error {

	const functionName = "Initialise"

	//myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  Creating Header..."))
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
	//myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  decide which encoder to use..."))

	// Decide which encoder to use
	switch strings.ToUpper(myEncoder.Type) {

	case "WAV":
		myEncoder.Encoder = &foxWavEncoder.FoxEncoder{
			SampleRate:  myEncoder.SampleRate,
			BitDepth:    myEncoder.BitDepth,
			NumChannels: myEncoder.NumChannels,
			Size:        myEncoder.Size,
		}
		myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  Creating wav header..."))

		err = myEncoder.writeHeader() // Write header during initialization
		if err != nil {
			return fmt.Errorf(packageName+":"+functionName+":error writing wav header: %w", err)
		}
	case "PCM":
		myEncoder.Encoder = &foxWavEncoder.FoxEncoder{
			SampleRate:  myEncoder.SampleRate,
			BitDepth:    myEncoder.BitDepth,
			NumChannels: myEncoder.NumChannels,
			Size:        myEncoder.Size,
		}

		//Create empty header - no header for PCM
		myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  PCM Output..."))

	default:
		return errors.New(packageName + ":" + functionName + ":unsupported encoder type")
	}

	//myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  Finished Header..."))
	myEncoder.debug(fmt.Sprintf(packageName+":"+functionName+" Header SampleRate: [%v] Channels: [%v] BitDepth: [%v] Size [%v] ",
		myEncoder.SampleRate, myEncoder.NumChannels, myEncoder.BitDepth, myEncoder.Size))
	return err
}

// Expose the methods
func (myEncoder *AudioEncoder) EncodeHeader() error {
	return myEncoder.writeHeader()
}

// Call low level encoder to convert [][]float64 samples to bytesream of choice, and then call output writer
func (myEncoder *AudioEncoder) EncodeData(buffer [][]float64) error {
	const functionName = "EncodeData"
	encodedData, err := myEncoder.Encoder.EncodeData(buffer)

	if err != nil {
		return errors.New(packageName + ":" + functionName + ": " + err.Error())
	}
	myEncoder.Peak = myEncoder.Encoder.GetPeak()
	myEncoder.NumSamples += int64(len(buffer[0]))
	return myEncoder.writeData(encodedData)
}

// EncoderStatus struct

type EncoderStatus struct {
	EncodedSamples  int64
	BufferedSamples int64
	StartTime       time.Time
}

func (myEncoder *AudioEncoder) EncodeSingleChannel(buffer []float64) ([]byte, error) {
	return myEncoder.Encoder.EncodeSingleChannel(buffer)
}

// Wraps the encoder into a channel based function that takes a stream of bytes, throttleInputChannel as duration and a wait group
// the optional throttleInputChannel will send the number of samples processed so far to be used by the feeding function in order to slow down any processing
// this function is optimized for speed and will use a buffer to store complete samples before encoding

func (myEncoder *AudioEncoder) WriteBytesChannel(
	BytesChannel <-chan []byte,
	throttleInputChannel chan<- int64,
) error {
	const functionName = "WriteBytesChannel"
	myEncoder.debug(fmt.Sprintf("%s:%s: Starting", packageName, functionName))

	var (
		totalSamples  int64
		bytesPerFrame = myEncoder.BitDepth / 8 * myEncoder.NumChannels
		targetBytes   = TargetBytesPerWrite // 8192 or other frame-aligned value
		buffer        []byte
	)

	// Calculate how many complete samples we can fit in targetBytes
	targetSamples := targetBytes / bytesPerFrame
	targetBytes = targetSamples * bytesPerFrame // Ensure byte alignment

	myEncoder.debug(fmt.Sprintf("%s:%s: Target samples: %d (Target Bytes %d bytes)",
		packageName, functionName, targetSamples, targetBytes))

	for chunk := range BytesChannel {
		if BytesChannel == nil { // Ensuring we don't process unexpected nil chunks
			myEncoder.debug(fmt.Sprintf("%s:%s: Bytes Channel closed, breaking loop", packageName, functionName))
			break
		}
		buffer = append(buffer, chunk...)
		// Process complete frames from buffer
		for len(buffer) >= targetBytes {
			frame := buffer[:targetBytes]

			// Write the complete frame
			if err := myEncoder.writeData(frame); err != nil {
				return fmt.Errorf("%s:%s: write error: %w", packageName, functionName, err)
			}

			totalSamples += int64(targetSamples)

			// Update throttle channel
			if throttleInputChannel != nil {
				throttleInputChannel <- totalSamples
			}
			myEncoder.debug(fmt.Sprintf("%s:%s: inner loop - %d bytes", packageName, functionName, len(buffer)))
			// Remove processed data from buffer
			buffer = buffer[targetBytes:]
		}
		if len(chunk) == 0 && len(buffer) < targetBytes { // Ensuring we don't process unexpected nil chunks
			myEncoder.debug(fmt.Sprintf("%s:%s: No more data but buffer residual too small to process", packageName, functionName))
			break
		}
		//let's print a debug message
		myEncoder.debug(fmt.Sprintf("%s:%s: Still writing %d chunk length %d bytes", packageName, functionName, len(chunk), len(buffer)))
	}

	// Flush remaining data (partial frame)
	if len(buffer) > 0 {
		myEncoder.debug(fmt.Sprintf("%s:%s: Flushing final %d bytes",
			packageName, functionName, len(buffer)))

		if err := myEncoder.writeData(buffer); err != nil {
			return fmt.Errorf("%s:%s: final write error: %w", packageName, functionName, err)
		}
		totalSamples += int64(len(buffer) / bytesPerFrame)
	} else {
		//let's print a debug message
		myEncoder.debug(fmt.Sprintf("%s:%s: Finished writing - No data to flush", packageName, functionName))
	}
	// Final throttle update
	if throttleInputChannel != nil {
		select {
		case throttleInputChannel <- totalSamples:
		default: // Avoid blocking if the channel isn't ready
		}
		close(throttleInputChannel)
	}
	myEncoder.NumSamples = totalSamples
	myEncoder.debug(fmt.Sprintf("%s:%s: Completed. Total samples: %d",
		packageName, functionName, totalSamples))
	return nil
}

// Wraps the encoder into a channel based function that takes a stream of inputSamples as [][]float64, throttleInputChannel as duration and a wait group
// the optional throttleInputChannel will send a delay based upont the encoder processing time to be used by the feeding function in order to slow down any processing
// this function is optimized for speed and will use a buffer to store samples before encoding
func (myEncoder *AudioEncoder) EncodeSamplesChannel(
	samplesChannel <-chan [][]float64,
	throttleInputChannel chan<- int64) error {
	const functionName = "EncodeSamplesChannel"
	var totalSamples int64
	totalSamples = 0
	bytesPerSample := myEncoder.BitDepth / 8 * myEncoder.NumChannels
	targetSamples := TargetBytesPerWrite / bytesPerSample // ~1365
	buffer := make([][]float64, myEncoder.NumChannels)
	loggedStart := false

	myEncoder.debug(fmt.Sprintf(packageName+":"+functionName+" Target Samples: %v", targetSamples))
	for samples := range samplesChannel {
		//start := time.Now()
		for i := range samples {
			buffer[i] = append(buffer[i], samples[i]...)
		}

		// Encode when buffer reaches target size
		for len(buffer[0]) >= targetSamples {
			// Extract batch from buffer
			batch := make([][]float64, myEncoder.NumChannels)
			for i := range buffer {
				batch[i] = buffer[i][:targetSamples]
				buffer[i] = buffer[i][targetSamples:]
			}
			// Encode the batch
			if !loggedStart {
				myEncoder.debug(packageName + ":" + functionName + " Encoding batch ")
				loggedStart = true
			}

			err := myEncoder.EncodeData(batch)
			if err != nil {
				myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + " Error with Encoder " + err.Error()))
				return fmt.Errorf("%s:%s: %w", packageName, functionName, err)
			}
			totalSamples += int64(len(batch[0]))

		}
		// Throttle logic
		//elapsed := time.Since(start)

		if throttleInputChannel != nil {
			throttleInputChannel <- totalSamples
		}
	}

	// Flush remaining samples
	if len(buffer[0]) > 0 {
		err := myEncoder.EncodeData(buffer)
		if err != nil {
			myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + " Error with final flush of Encoder " + err.Error()))
			return fmt.Errorf("%s:%s: %w", packageName, functionName, err)
		}
		totalSamples += int64(len(buffer[0]))

	}
	//probably not needed as should no longer be reading by now...
	if throttleInputChannel != nil {
		throttleInputChannel <- totalSamples
		close(throttleInputChannel)
	}
	myEncoder.debug(fmt.Sprintf(packageName+":"+functionName+" Total samples encoded: %v", totalSamples))
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
	myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + " generate and write Header.."))
	headerBytes, err := myEncoder.Encoder.EncodeHeader()
	if err != nil {
		return errors.New(packageName + ":" + functionName + ": " + err.Error())
	}
	//myEncoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  writing header.."))
	//err = writeOutput(headerBytes, myEncoder.Filename)
	err = myEncoder.writeData(headerBytes)
	if err != nil {
		return errors.New(packageName + ":" + functionName + ": " + err.Error())
	}
	return nil
}

func (e *AudioEncoder) writeData(data []byte) error {
	const functionName = "writeData"
	if e.Filename == "" {
		// Write directly to stdout without buffering
		_, err := os.Stdout.Write(data)
		if err != nil {
			return fmt.Errorf("%s:%s: %w", packageName, functionName, err)
		}
		return nil
	}

	// Initialize file and writer on first use
	if e.file == nil {
		file, err := os.OpenFile(e.Filename, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0644)
		if err != nil {
			return fmt.Errorf("%s:%s: %w", packageName, functionName, err)
		}
		e.file = file
		e.writer = bufio.NewWriter(file)

	}

	// Write data through the buffer
	if _, err := e.writer.Write(data); err != nil {
		return fmt.Errorf("%s:%s: %w", packageName, functionName, err)
	}
	e.writer.Flush()
	return nil
}

func (e *AudioEncoder) Close() error {
	const functionName = "Close"
	var err error

	// Flush buffered data before closing
	if e.writer != nil {
		if flushErr := e.writer.Flush(); flushErr != nil {
			err = fmt.Errorf("%s:%s: flush error: %w", packageName, functionName, flushErr)
		}
	}

	// Close the file handle if open
	if e.file != nil {
		if closeErr := e.file.Close(); closeErr != nil {
			if err != nil {
				err = fmt.Errorf("%v; close error: %w", err, closeErr)
			} else {
				err = fmt.Errorf("%s:%s: close error: %w", packageName, functionName, closeErr)
			}
		}
	}

	return err
}

func WriteWavFile(filename string, samples [][]float64, targetSampleRate int,
	targetBitDepth int, numChannels int, Overwrite bool, logger *foxLog.Logger) {
	defer func() {
		if r := recover(); r != nil {
			logger.Error(fmt.Sprintf("Recovered in EncodeAsync: %v", r))
		}
	}()

	// Check if the file already exists
	if _, err := os.Stat(filename); err == nil {
		if !Overwrite {
			logger.Error(fmt.Sprintf("File already exists: %s", filename))
			return // Exit without encoding
		}
		DeleteFile(filename, logger)
	}

	// Calculate actual sample count-based size
	size := int64(0)
	if len(samples) > 0 && len(samples[0]) > 0 {
		size = int64(len(samples[0])) * int64(numChannels) * int64(targetBitDepth/8)
	}

	encoder := AudioEncoder{
		Type:        "Wav",
		SampleRate:  targetSampleRate,
		BitDepth:    targetBitDepth,
		NumChannels: numChannels,
		Size:        size,
		Filename:    filename,
	}

	if err := encoder.Initialise(); err != nil {
		logger.Error(fmt.Sprintf("Encoder init failed for %s: %v", filename, err))
		return
	}

	if err := encoder.EncodeData(samples); err != nil {
		logger.Error(fmt.Sprintf("Encoding failed for %s: %v", filename, err))
	} else {
		logger.Debug(fmt.Sprintf("Successfully encoded %s", filename))
	}
}

func DeleteFile(filePath string, myLogger *foxLog.Logger) error {
	// Delete the file
	err := os.Remove(filePath)
	if err == nil {
		myLogger.Debug(fmt.Sprintf("File '%s' successfully deleted.", filePath))
	} else if os.IsNotExist(err) {
		myLogger.Debug(fmt.Sprintf("File '%s' does not exist.", filePath))
	} else {
		myLogger.Debug(fmt.Sprintf("Error deleting file '%s': %v\n", filePath, err))
	}
	return err
}

// Function to handle debug calls, allowing for different logging implementations
func (myEncoder *AudioEncoder) debug(message string) {
	if myEncoder.DebugOn {
		if myEncoder.DebugFunc != nil {
			myEncoder.DebugFunc(message)
		} else { // if no external debug function available just print the message
			println(message)
		}
	}
}
