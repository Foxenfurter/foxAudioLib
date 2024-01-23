// package calls an encoder based upon the supplied format
// and then processes the returned bytestream
// either to standardd out or to the supplied file name
package foxAudioDecoder

import (
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"sync"

	foxWavDecoder "github.com/Foxenfurter/foxAudioLib/foxAudioDecoder/foxWavDecoder"
)

const packageName = "foxAudioDecoder"

// The input Buffer is used to read in byte date from the external source
const InputBufferSize = 65000

// The DecoderFrameSize is the number of frame samples to output in each frame.
// A frame being an arbitrary number of frame samples and a frame sample being  n channels of samples e.g. stereo is 2 single samples.
const DefaultDecoderFrameSize int = 1000

type AudioDecoder struct {
	SampleRate        int
	BitDepth          int
	NumChannels       int
	Size              uint32 // Size of the audio data
	Type              string
	Filename          string // Added for file reading
	ActiveDecoder     DecoderInterface
	DecoderFrameSize  int
	DecoderFrameCount int64 // number
	Buffer            []byte
	HeaderSize        int // size of the header

	Reader     io.Reader // Use io.Reader for flexibility
	Mutex      *sync.Mutex
	InputChan  chan []byte
	OutputChan chan [][]float64
	WaitGroup  *sync.WaitGroup

	DebugFunc func(string) // enables the use of an external debug function supplied at the application level - expect to use foxLog

}

// each Encoder must have these methods defined
type DecoderInterface interface {
	//Initialise() error
	DecodeHeader([]byte) error // Now returns a *FoxDecoder
	DecodeData([]byte, int) ([][]float64, error)
	GetBitDepth() int
	GetNumChannels() int
	GetSize() uint32
	GetSampleRate() int
}

// NewDecoder creates a new decoder with implicit file opening or Stdin setup
func (myDecoder *AudioDecoder) Initialise() error {
	const functionName = "Initialise"

	// Open file or set reader for Stdin
	if myDecoder.Filename == "" {
		myDecoder.Reader = os.Stdin
	} else {
		var err error
		// clean and standardize the filename
		myDecoder.Filename = filepath.ToSlash(filepath.Clean(myDecoder.Filename))
		myDecoder.Reader, err = os.Open(myDecoder.Filename)
		if err != nil {
			return err
		}
	}
	// this is the number of frame samples to return in one go
	if myDecoder.DecoderFrameSize == 0 {
		myDecoder.DecoderFrameSize = DefaultDecoderFrameSize
	}

	// Initial read of input into the buffer in order to extract the  header
	n, err := myDecoder.Reader.Read(myDecoder.Buffer)
	if err != nil && err != io.EOF {

		return errors.New(packageName + ":" + functionName + ": " + err.Error())
	}
	if n == 0 {
		errorText := "Zero length input: "
		return errors.New(packageName + ":" + functionName + ": " + errorText)
	}

	// Initialize decoder based on type
	switch myDecoder.Type {
	case "Wav":
		myDecoder.ActiveDecoder = new(foxWavDecoder.FoxDecoder)
		err := myDecoder.DecodeHeader()

		// at his point the buffer should have the header stripped out.
		if err != nil {
			return errors.New(packageName + ":" + functionName + ":" + err.Error())
		}
	default:
		errorText := "unsupported encoder type "
		return errors.New(packageName + ":" + functionName + ":" + errorText)
	}
	// At this point we should have decoded the header and so know the number of channels, bitDepth and SampleRate of the data in the bytestream.
	// now prepare for asynchronous operations
	myDecoder.Mutex = &sync.Mutex{}
	if n > myDecoder.HeaderSize {
		myDecoder.Buffer = myDecoder.Buffer[myDecoder.HeaderSize:n]
	}
	// It is important that the low level decoder packages support these setter functions
	myDecoder.BitDepth = myDecoder.ActiveDecoder.GetBitDepth()
	myDecoder.SampleRate = myDecoder.ActiveDecoder.GetSampleRate()
	myDecoder.NumChannels = myDecoder.ActiveDecoder.GetNumChannels()
	myDecoder.Size = myDecoder.ActiveDecoder.GetSize()
	myDecoder.debug(fmt.Sprintf(packageName+":"+functionName+" SampleRate: [%v] Channels: [%v] BitDepth: [%v] Size [%v] ", myDecoder.SampleRate, myDecoder.NumChannels, myDecoder.BitDepth, myDecoder.Size))
	// Now initialise the Buffers
	myDecoder.InputChan = make(chan []byte, 100)
	myDecoder.OutputChan = make(chan [][]float64, 100)

	myDecoder.WaitGroup = &sync.WaitGroup{}

	return nil
}

// Decode Header use the low level library to decode the header
func (myDecoder *AudioDecoder) DecodeHeader() error {
	const functionName = "DecodeHeader"
	err := myDecoder.ActiveDecoder.DecodeHeader(myDecoder.Buffer)
	if err != nil {
		return errors.New(packageName + ":" + functionName + ": " + err.Error())
	}

	// We need to replace this with getter calls from the lower level function.
	//myDecoder.HeaderSize = 44

	return nil
}

// Start is an asynchronous function that continuously reads PCM samples and fills the buffer.
// func (myDecoder *AudioDecoder) Start(doneProcessing chan struct{}) error {
func (myDecoder *AudioDecoder) ReadInput() error {
	const functionName = "ReadInput"
	//myDecoder.debug(fmt.Sprintf(packageName+":"+functionName+" starting buffer length: %v", len(myDecoder.Buffer)))

	// Increment the WaitGroup counter
	myDecoder.WaitGroup.Add(1)

	go func() error {

		myDecoder.debug(fmt.Sprintf(packageName+":"+functionName+" Send initial input: %v ", len(myDecoder.Buffer)))
		// Send the updated AudioDecoder to the channel
		myDecoder.Mutex.Lock()
		myDecoder.InputChan <- myDecoder.Buffer[:len(myDecoder.Buffer)]
		myDecoder.Mutex.Unlock()

		// We need to handle scenarios where the input file contains meta data at the end of the file as well as reaching the end of file
		// what we are doing is setting the total amount of data in bytes that we expect to read from the input int remainingbytes variable
		// we check whether the total amount read in exceeds the expected remaining bytes input and if it does, we truncate the read buffer
		// we also check for end of file
		// finally we decrement the counter
		// if the file size is not known the Header.Size will either be 0 or max of uint32, which should mean EOF will be reached first.

		// initialize remaining bytes - and don't forget to include the first header read as this includes data too!
		remainingBytes := myDecoder.Size - uint32(len(myDecoder.Buffer))

		for {
			// Read PCM samples into the buffer
			n, err := myDecoder.Reader.Read(myDecoder.Buffer)

			if err != nil && err != io.EOF {
				// Log the error or handle it as needed
				return errors.New(packageName + ":" + functionName + ": " + err.Error())
			}

			// Lock the mutex before updating the buffer
			myDecoder.Mutex.Lock()
			//myDecoder.debug(fmt.Sprintf(packageName+":"+functionName+" Sending next packet bytes remaining [%v] buffer read [%v] ", remainingBytes, uint32(n)))
			// Handle truncation if Header.Size is contained in the current chunk
			if remainingBytes > 0 && remainingBytes < uint32(n) {
				n = int(remainingBytes)
			}

			// Send the updated AudioDecoder to the channel
			myDecoder.InputChan <- myDecoder.Buffer[:n]

			// Decrement the remainingBytes counter
			remainingBytes -= uint32(n)

			// Unlock the mutex after updating the buffer
			myDecoder.Mutex.Unlock()

			if err == io.EOF || remainingBytes == 0 {
				// Log the error or handle it as needed
				//myDecoder.debug(fmt.Sprintf(packageName + ":" + functionName + " Reached End of Stream"))
				close(myDecoder.InputChan)
				break
			}
		}

		myDecoder.WaitGroup.Done()
		myDecoder.debug(fmt.Sprintf(packageName + ":" + functionName + " Finished reading"))
		return nil
	}()

	return nil
}

func (myDecoder *AudioDecoder) decodeAndHandleOutput(data []byte, numSamples int) error {
	const functionName = "DecodeData"

	output, err := myDecoder.ActiveDecoder.DecodeData(data, numSamples)
	if err != nil {

		return errors.New(packageName + ":" + functionName + ":" + err.Error())
	}

	myDecoder.Mutex.Lock()
	myDecoder.OutputChan <- output
	myDecoder.Mutex.Unlock()
	return nil
}

func (myDecoder *AudioDecoder) DecodeData() error {
	const functionName = "DecodeData"
	//myDecoder.debug(fmt.Sprintf(packageName + ":" + functionName + " starting"))
	numSamples := myDecoder.DecoderFrameSize
	totalChunkSize := 0
	var leftoverData []byte
	totalSamples := 0
	// calculate size of frame Sample for use in the decoder
	frameSampleSize := myDecoder.NumChannels * myDecoder.BitDepth / 8
	myDecoder.debug(fmt.Sprintf(packageName+":"+functionName+" Starting Number Channels [%v] Bit Depth [%v] ", myDecoder.NumChannels, myDecoder.BitDepth))
	bufferSize := numSamples * frameSampleSize

	for chunk := range myDecoder.InputChan {
		totalChunkSize += len(chunk)
		myDecoder.Mutex.Lock()
		combinedData := append(leftoverData, chunk...)
		myDecoder.Mutex.Unlock()
		for len(combinedData) >= bufferSize { // Ensure sufficient data for decoding
			totalSamples += numSamples
			err := myDecoder.decodeAndHandleOutput(combinedData, numSamples)
			if err != nil {

				return errors.New(packageName + ":" + functionName + ":" + err.Error())
			}
			combinedData = combinedData[bufferSize:]

		}
		leftoverData = combinedData // Move remaining data to leftoverData

	}

	if len(leftoverData) > 0 {

		myDecoder.debug(fmt.Sprintf(packageName+":"+functionName+" decoding Leftover Audio data: [ %v]", len(leftoverData)/frameSampleSize))
		totalSamples += len(leftoverData) / frameSampleSize
		err := myDecoder.decodeAndHandleOutput(leftoverData, len(leftoverData)/frameSampleSize)
		if err != nil {
			return errors.New(packageName + ":" + functionName + ": " + err.Error())
		}

	}
	myDecoder.debug(fmt.Sprintf(packageName+":"+functionName+" Finished Decoding Audio, Samples processed: [ %v] Chunks Received [ %v ]", totalSamples, totalChunkSize))
	close(myDecoder.OutputChan)
	//myDecoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  DECODE DATA sent Done"))
	return nil
}

func (myDecoder *AudioDecoder) Close() {
	const functionName = "Close"
	// channels are closed in the functions that populate them.
	// This function is called if any other cleanup is needed
	myDecoder.debug(fmt.Sprintf(packageName + ":" + functionName + "  All done..."))
}

// Function to handle debug calls, allowing for different logging implementations
func (myDecoder *AudioDecoder) debug(message string) {

	if myDecoder.DebugFunc != nil {
		myDecoder.DebugFunc(message)
	} else { // if no external debug function available just print the message
		println(message)
	}

}
