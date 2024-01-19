// package calls an encoder based upon the supplied format
// and then processes the returned bytestream
// either to standardd out or to the supplied file name
package foxAudioDecoder

import (
	"errors"
	"io"
	"os"
	"sync"

	foxWavDecoder "github.com/Foxenfurter/foxAudioLib/foxAudioDecoder/foxWavDecoder"
)

const packageName = "foxAudioDecoder"

// The input Buffer is used to read in byte date from the external source
const InputBufferSize = 65000

// The DecoderFrameSize is the number of frame samples to output in each frame.
// A frame being an arbitrary number of frame samples and a frame sample being  n channels of samples e.g. stereo is 2 single samples.
const DefaultDecoderFrameSize int = 1000

type PCMDecoder struct {
	SampleRate        int
	BitDepth          int
	NumChannels       int
	Size              int // Size of the audio data
	Type              string
	Filename          string // Added for file reading
	ActiveDecoder     DecoderInterface
	DecoderFrameSize  int
	DecoderFrameCount int64 // number
	Buffer            []byte
	HeaderSize        int

	Reader     io.Reader // Use io.Reader for flexibility
	Mutex      *sync.Mutex
	InputChan  chan []byte
	OutputChan chan [][]float64
	WaitGroup  *sync.WaitGroup
}

// each Encoder must have these methods defined
type DecoderInterface interface {
	//Initialise() error
	DecodeHeader([]byte) error // Now returns a *FoxDecoder
	DecodeData([]byte, int) ([][]float64, error)
	GetBitDepth() int
	GetNumChannels() int
	GetSize() int
	GetSampleRate() int
}

// NewDecoder creates a new decoder with implicit file opening or Stdin setup
func (myDecoder *PCMDecoder) Initialise() error {
	const functionName = "Initialise"
	// Open file or set reader for Stdin
	if myDecoder.Filename == "" {
		myDecoder.Reader = os.Stdin
	} else {
		var err error
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

		return errors.New(packageName + ":" + functionName + ":" + err.Error())
	}
	if n == 0 {
		errorText := "Zero length input: "
		return errors.New(packageName + ":" + functionName + ":" + errorText)
	}

	// Initialize decoder based on type
	switch myDecoder.Type {
	case "Wav":
		myDecoder.ActiveDecoder = new(foxWavDecoder.FoxDecoder)
		err := myDecoder.DecodeHeader()
		println("Channels", myDecoder.ActiveDecoder.(*foxWavDecoder.FoxDecoder).NumChannels)
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
	// Now initialise the Buffers
	myDecoder.InputChan = make(chan []byte, 100)
	myDecoder.OutputChan = make(chan [][]float64, 100)

	myDecoder.WaitGroup = &sync.WaitGroup{}

	return nil
}

// Decode Header
func (myDecoder *PCMDecoder) DecodeHeader() error {
	const functionName = "DecodeHeader"
	err := myDecoder.ActiveDecoder.DecodeHeader(myDecoder.Buffer)
	if err != nil {
		return errors.New(packageName + ":" + functionName + ":" + err.Error())
	}

	// We need to replace this with getter calls from the lower level function.
	myDecoder.HeaderSize = 44

	return nil
}

// Start is an asynchronous function that continuously reads PCM samples and fills the buffer.
// func (myDecoder *PCMDecoder) Start(doneProcessing chan struct{}) error {
func (myDecoder *PCMDecoder) Start() error {
	const functionName = "Start"
	println(packageName, functionName, " starting buffer length", len(myDecoder.Buffer))

	// Increment the WaitGroup counter
	myDecoder.WaitGroup.Add(1)

	go func() error {

		println(packageName, functionName, " sending initial input: ", len(myDecoder.Buffer))
		// Send the updated PCMDecoder to the channel
		myDecoder.Mutex.Lock()
		myDecoder.InputChan <- myDecoder.Buffer[:len(myDecoder.Buffer)]
		myDecoder.Mutex.Unlock()

		for {
			// Read PCM samples into the buffer
			n, err := myDecoder.Reader.Read(myDecoder.Buffer)
			println(packageName, functionName, " reading input bytes read: ", n)
			if err != nil && err != io.EOF {
				// Log the error or handle it as needed
				return errors.New(packageName + ":" + functionName + ":" + err.Error())
			}
			if err == io.EOF {
				// Log the error or handle it as needed
				println(packageName, functionName, " EOF: ", n)
				if n > 0 {
					println("Test: Sending last bits and bobs to bytestream buffer...", len(myDecoder.Buffer))
					myDecoder.Mutex.Lock()
					myDecoder.InputChan <- myDecoder.Buffer[:n]
					myDecoder.Mutex.Unlock()
				}
				break
			}
			if n == 0 {
				// No more data to read
				println(packageName, functionName, " zero length: ")
				break
			}

			// Lock the mutex before updating the buffer
			println(packageName, functionName, " sending input bytes: ", n)
			myDecoder.Mutex.Lock()

			// Send the updated PCMDecoder to the channel

			myDecoder.InputChan <- myDecoder.Buffer[:n]

			// Unlock the mutex after updating the buffer
			myDecoder.Mutex.Unlock()
		}
		println(packageName, functionName, " Done")
		myDecoder.WaitGroup.Done()
		close(myDecoder.InputChan)
		println(packageName, functionName, " START sent DONE")
		return nil
	}()

	return nil
}

func (myDecoder *PCMDecoder) decodeAndHandleOutput(data []byte, numSamples int) error {
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

func (myDecoder *PCMDecoder) DecodeData() error {
	const functionName = "DecodeData"
	println(packageName, functionName, " starting")
	numSamples := myDecoder.DecoderFrameSize
	totalChunkSize := 0
	var leftoverData []byte
	totalSamples := 0
	// calculate size of frame Sample for use in the decoder
	frameSampleSize := myDecoder.NumChannels * myDecoder.BitDepth / 8
	println(packageName, functionName, myDecoder.NumChannels, myDecoder.BitDepth)
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
	if len(leftoverData) < bufferSize {
		println(packageName, functionName, " combined data < buffer sizee")
	}

	// Process any remaining leftover data
	if len(leftoverData) > 0 {
		println(packageName, functionName, " decoding Leftover Audio data...", len(leftoverData)/frameSampleSize)
		totalSamples += len(leftoverData) / frameSampleSize
		err := myDecoder.decodeAndHandleOutput(leftoverData, len(leftoverData)/frameSampleSize)
		if err != nil {
			return errors.New(packageName + ":" + functionName + ":" + err.Error())
		}

	}

	println(packageName, functionName, " Finished Decoding Audio data, Samples processed: [", totalSamples, "] Chunks Received [", totalChunkSize, "]")

	close(myDecoder.OutputChan)

	println(packageName, functionName, "  DECODE DATA sent Done")
	return nil
}

func (myDecoder *PCMDecoder) Close() {
	const functionName = "Close"
	// channels are closed in the functions that populate them.
	// This function is called if any other cleanup is needed
	println(packageName, functionName, "  All done...")
}
