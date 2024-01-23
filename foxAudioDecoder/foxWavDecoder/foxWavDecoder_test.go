package foxWavDecoder_test

import (
	"fmt"
	"io"
	"os"
	"sync"
	"testing"

	foxWavDecoder "github.com/Foxenfurter/foxAudioLib/foxAudioDecoder/foxWavDecoder"
)

// ... (DecodeAudioData function and helper functions from previous responses) ...

const InputBufferSize = 65000

func TestReadAndDecodeAudio(t *testing.T) {
	//filename := "c:/temp/impulse.wav"
	filename := "c:/temp/impulse.wav"
	byteStreamChan := make(chan []byte)
	decodedSamplesChan := make(chan [][]float64)
	fmt.Println("Test: Full running TestReadAndDecodeAudio...")
	//var mainDecoder foxWavDecoder.FoxDecoder
	var decoder foxWavDecoder.FoxDecoder
	var headerDecodingComplete sync.WaitGroup

	headerDecodingComplete.Add(1) // For header decoding goroutine

	var wg sync.WaitGroup
	wg.Add(1) // For reader goroutine

	// Start reader goroutine
	go func() {
		fmt.Println("Test: running reader sub-routine...")
		f, err := os.Open(filename)
		if err != nil {
			fmt.Println("Test: File Error...", err)
			panic(err)
		}
		defer f.Close()

		fmt.Println("Test: Reading Initial buffer...")
		// Read initial buffer
		buf := make([]byte, InputBufferSize)
		n, err := f.Read(buf)
		if err != nil {
			fmt.Println("Test: Read Error ...")
			panic(err)
		}

		// Read and test header
		myheader := buf[:n]
		fmt.Println("Test: Reading Header ...")
		//decoder := foxWavDecoder.FoxDecoder{}
		err = decoder.DecodeHeader(myheader)
		//mainDecoder = *decoder
		if err != nil {
			fmt.Println("Test: Decode Header Error ...")
			panic(err)
		}
		fmt.Println("WAV header info:", decoder.SampleRate)
		headerDecodingComplete.Done() // Signal header decoding completion

		fmt.Println("Test: Reading first data chunk...")
		fmt.Println("Lets see if the default buffersize is too big: ", InputBufferSize, " and size of n:", n, " Buffer Length:", len(buf))
		// Send remaining bytes as the first data chunk
		byteStreamChan <- buf[decoder.ReaderCursor:n]
		// cursor should now be at the beginning of the bytestream
		decoder.ReaderCursor = 0

		fmt.Println("Test: Make first buffer...")

		// Read and send remaining data chunks
		buf = make([]byte, InputBufferSize)
		for {
			n, err := f.Read(buf)

			if err != nil {
				if err == io.EOF {
					// Send any remaining bytes in the buffer
					if n > 0 {
						fmt.Println("Test: Sending last bits and bobs to bytestream buffer...")
						byteStreamChan <- buf[:n]
					}
					break
				}
				panic(err)
			}
			fmt.Println("Lets see if the default buffersize is too big: ", InputBufferSize, " and size of n:", n, " Buffer Length:", len(buf))
			byteStreamChan <- buf[:n]
		}
		fmt.Println("Test: Closing bytestream...")
		close(byteStreamChan) // Signal end of data

	}()

	headerDecodingComplete.Wait() // Wait for header decoding to complete
	wg.Add(1)                     // Add for decoding goroutine
	go func() {
		defer wg.Done() // Signal processing of decoded data completion
		for range decodedSamplesChan {
			// No statement here means the received data is discarded.

		}

	}()

	go func() {
		defer wg.Done() // Signal decoding completion
		decodeAudio(byteStreamChan, decodedSamplesChan, &decoder)

	}()

	wg.Wait() // Wait for both goroutines to finish

}

func decodeAudio(byteStreamChan chan []byte, decodedSamplesChan chan [][]float64, decoder *foxWavDecoder.FoxDecoder) {
	var leftoverData []byte
	numSamples := 1000
	totalSamples := 0

	fmt.Println("Test: Decoder called")
	bufferSize := numSamples * decoder.NumChannels * decoder.BitDepth / 8
	fmt.Println("Test: Buffer Size: ", bufferSize)

	fmt.Println("Test: Reader Cursor Position: ", decoder.ReaderCursor)
	for chunk := range byteStreamChan {
		combinedData := append(leftoverData, chunk...)
		fmt.Println("Test: length Combined Data: ", len(combinedData), " Buffer Size: ", bufferSize)
		for len(combinedData) >= bufferSize { // Ensure sufficient data for decoding
			totalSamples += numSamples
			//decodeAndHandleOutput(decoder, combinedData, len(combinedData)/bufferSize, decodedSamplesChan)
			decodeAndHandleOutput(decoder, combinedData, numSamples, decodedSamplesChan)

			decoder.ReaderCursor += bufferSize
			fmt.Println("Test: New Cursor Before new read: ", decoder.ReaderCursor, " Combined data length: ", len(combinedData))
			combinedData = combinedData[decoder.ReaderCursor:]
			decoder.ReaderCursor = 0
			// Update ReaderCursor
			fmt.Println("Test: New Cursor Position: ", decoder.ReaderCursor, " Combined data length: ", len(combinedData))
			// moved this to last position of loop so that I can check other values first
		}

		leftoverData = combinedData // Move remaining data to leftoverData
	}

	// Process any remaining leftover data
	if len(leftoverData) > 0 {
		fmt.Println("Test: decoding Leftover Audio data...")
		totalSamples += len(leftoverData) / (decoder.NumChannels * decoder.BitDepth / 8)
		decodeAndHandleOutput(decoder, leftoverData, len(leftoverData)/bufferSize, decodedSamplesChan)
	}

	fmt.Println("Test: Finished Decoding Audio data, Samples processed: ", totalSamples)
	close(decodedSamplesChan) // Signal end of decoded samples
	fmt.Println("Test: All done...")
}

func decodeAndHandleOutput(decoder *foxWavDecoder.FoxDecoder, data []byte, numSamples int, decodedSamplesChan chan [][]float64) {
	output, err := decoder.DecodeData(data, numSamples)
	if err != nil {
		fmt.Println("Test: Error with decoder: ")
		// TODO: Handle error properly (e.g., return error or log it)
		close(decodedSamplesChan)
		return
	}

	fmt.Println("Test: Number of samples to be returned", len(output[0]))

	decodedSamplesChan <- output
}

func testFileOpen(t *testing.T) {

	filename := "c:/temp/Pencil_1644.wav"
	//byteStreamChan := make(chan []byte)
	//decodedSamplesChan := make(chan [][]float64)
	fmt.Println("Test: Running TestFileOpen...")
	decoder := foxWavDecoder.FoxDecoder{}
	f, err := os.Open(filename)
	if err != nil {
		fmt.Println("Test: File Error...")
		panic(err)
	}
	defer f.Close()

	fmt.Println("Test: Reading Initial buffer...")
	// Read initial buffer
	buf := make([]byte, InputBufferSize)

	n, err := f.Read(buf)
	if err != nil {
		fmt.Println("Test: Read Error ...")
		panic(err)
	}
	//fmt.Println("Test: Reading Buffer:", buf)
	// Read and test header
	myheader := buf[:n]
	fmt.Println("Test: Reading Header:")
	err = decoder.DecodeHeader(myheader)

	if err != nil {
		fmt.Println("Test: Decode Header Error ...")
		panic(err)
	}
	fmt.Println("WAV header info:", decoder.SampleRate)

}

func TestMain(m *testing.M) {
	// Run tests
	fmt.Println("Running Decoder  Test:")
	exitCode := m.Run()

	// Exit with the test result code
	os.Exit(exitCode)
}
