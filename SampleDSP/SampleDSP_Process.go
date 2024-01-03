package main

import (
	"encoding/binary"
	"fmt"
	"io"
	"os"
	"sync"

	"scientificgo.org/fft"
)

// Config holds configuration parameters for audio processing.
type Config struct {
	NumChannels   int
	SamplingRate  float64
	BufferSize    int
	OverlapFactor float64
}

func processChannel(samples []float64, resultChan chan []float64) {
	// Apply window function to the samples
	windowedFrame := applyWindow(samples, fft.Hamming)

	// Perform FFT on the windowed frame
	fftResult := fft.FFTReal(windowedFrame)

	// Process the frequency domain data as needed

	// Inverse FFT to reconstruct the frame (optional)
	reconstructedFrame := fft.IFFTReal(fftResult)
	fmt.Printf("Reconstructed Frame: %v\n", reconstructedFrame)

	// Send the reconstructed frame to the result channel
	resultChan <- reconstructedFrame
}

func writePCMStream(resultChan chan []float64, config Config, outFile *os.File) {
	defer close(resultChan)

	// Continuously collate and write PCM samples to the output file
	for reconstructedFrame := range resultChan {
		for i := 0; i < config.BufferSize; i++ {
			for channel := 0; channel < config.NumChannels; channel++ {
				// Write PCM samples to the output file
				sample := int16(reconstructedFrame[i])
				err := binary.Write(outFile, binary.LittleEndian, sample)
				if err != nil {
					fmt.Println("Error writing PCM sample:", err)
					return
				}
			}
		}
	}
}

func applyWindow(frame []float64, windowFunc func(int) float64) []float64 {
	window := make([]float64, len(frame))
	for i := range window {
		window[i] = windowFunc(i) * frame[i]
	}
	return window
}

func loadPCMIntoBuffer(filename string, buffer [][]float64, config Config) (int, error) {
	file, err := os.Open(filename)
	if err != nil {
		return 0, err
	}
	defer file.Close()

	// Read PCM samples from the file into the buffer
	for i := 0; i < config.BufferSize; i++ {
		for channel := 0; channel < config.NumChannels; channel++ {
			var sample int16
			err := binary.Read(file, binary.LittleEndian, &sample)
			if err != nil {
				if err == io.EOF {
					return i, nil // Reached end of file
				}
				return i, err
			}

			buffer[channel][i] = float64(sample)
		}
	}

	return config.BufferSize, nil
}

func main() {
	config := Config{
		NumChannels:   2,       // Assuming stereo audio
		SamplingRate:  44100.0, // Change as needed
		BufferSize:    1024,    // Change as needed
		OverlapFactor: 0.5,     // Change as needed
	}

	// Create a buffer to hold PCM samples for each channel
	buffer := make([][]float64, config.NumChannels)
	for i := 0; i < config.NumChannels; i++ {
		buffer[i] = make([]float64, config.BufferSize)
	}

	// Open the output file for writing PCM samples
	outFile, err := os.Create("output.pcm")
	if err != nil {
		fmt.Println("Error creating output file:", err)
		return
	}
	defer outFile.Close()

	// Create a result channel to receive reconstructed frames from channels
	resultChan := make(chan []float64, 10) // Buffer size can be adjusted

	// Continue reading and processing until the end of the PCM stream
	for {
		// Load PCM samples into the buffer
		numSamples, err := loadPCMIntoBuffer("your_pcm_file.pcm", buffer, config)
		if err != nil {
			fmt.Println("Error loading PCM samples:", err)
			return
		}

		// Create a WaitGroup to synchronize goroutines
		var wg sync.WaitGroup

		// Process each channel in parallel
		for channel := 0; channel < config.NumChannels; channel++ {
			// Increment the WaitGroup counter before starting the goroutine
			wg.Add(1)
			go func(channel int) {
				defer wg.Done() // Decrement the counter when the goroutine completes

				// Process the channel asynchronously for the samples in the buffer
				processChannel(buffer[channel][:numSamples], resultChan)
			}(channel)
		}

		// Start a goroutine to write PCM samples to the output file
		go writePCMStream(resultChan, config, outFile)

		// Wait for all processing to complete using WaitGroup
		wg.Wait()

		// Close the result channel to signal the end of processing for this buffer
		close(resultChan)

		// Sleep or perform other operations as needed

		// Check if reached end of PCM stream
		if numSamples < config.BufferSize {
			break
		}
	}

	fmt.Println("Processing complete.")
}
