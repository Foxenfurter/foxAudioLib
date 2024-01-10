foxAudioLib is a set of golang packages to be used within a console DSP programme. The intention is to keep the functionality limited to what is necessary and to keep the implementation simple. This is very much experimental work to see if it is possible to migrate SqueezeDSP away from c# and the dependency on InguzDSP.dll as this is becoming increasingly unmanageable

I have chosen to use golang because it is a statically typed portable language that was specifically designed to be fast and easy to use. In order to drive the development of libraries I have a rough design of the application.


Rough design.
package main

import (
	"fmt"
	"os"
	"sync"
	"time"
)

// Configuration holds the configuration parameters.
type Configuration struct {
	BypassMode bool // Enable bypass mode
	// Add other configuration parameters as needed
}

var config Configuration

func main() {
	// Parse command line arguments.
	inputFile := os.Args[1] // Add proper command line argument parsing.

	// Read file header to determine audio format and extract relevant information.
	// ...

	// Read DSP configuration.
	// ...

	// Build DSP configuration settings.
	// ...

	// Initialize channels, goroutines, and synchronization mechanisms.
	audioDataChannel := make(chan AudioData, bufferSize)
	timeDelayBuffer := make(chan AudioData, bufferSize)
	convolvedBuffer := make(chan AudioData, bufferSize)
	volumeProcessedBuffer := make(chan AudioData, bufferSize)

	var wg sync.WaitGroup

	// Start reading samples into the audio data channel.
	wg.Add(1)
	go readPCMFile(inputFile, audioDataChannel)

	// Start time delay stage.
	for i := 0; i < numTimeDelayProcessors; i++ {
		wg.Add(1)
		go timeDelayStage(audioDataChannel, timeDelayBuffer, &wg)
	}

	// Start convolution stage.
	for i := 0; i < numConvolutionProcessors; i++ {
		wg.Add(1)
		go convolutionStage(timeDelayBuffer, convolvedBuffer, &wg)
	}

	// Start volume processing stage.
	for i := 0; i < numVolumeProcessors; i++ {
		wg.Add(1)
		go volumeProcessingStage(convolvedBuffer, volumeProcessedBuffer, &wg)
	}

	// Start final output logic.
	go finalOutput(volumeProcessedBuffer)

	// Start config watcher.
	go configWatcher(&wg)

	// Wait for all goroutines to finish.
	wg.Wait()

	// Close channels to signal completion.
	close(audioDataChannel)
	close(timeDelayBuffer)
	close(convolvedBuffer)
	close(volumeProcessedBuffer)
}

// Placeholder for a function that watches for changes in the DSP configuration file.
func configWatcher(wg *sync.WaitGroup) {
	defer wg.Done()

	// Implement the logic to watch for changes in the DSP configuration file.
	// If changes are detected, rebuild the DSP configuration settings.
	for {
		// Placeholder for checking config file changes.
		// Example: use a file watcher library to monitor changes.
		// If changes are detected, rebuild DSP configuration settings and update the pipeline.
		time.Sleep(5 * time.Second) // Adjust the sleep duration based on your requirements.
		fmt.Println("Checking config file changes...")
	}
}

// Placeholder for a function that checks whether the bypass mode is enabled.
func isBypassModeEnabled() bool {
	// Add logic to check whether bypass mode is enabled based on configuration.
	return config.BypassMode
}

// Placeholder for a function that performs time delay processing.
func timeDelayStage(input <-chan AudioData, output chan<- AudioData, wg *sync.WaitGroup) {
	defer wg.Done()

	for audioData := range input {
		if isBypassModeEnabled() {
			// Bypass mode: Pass data directly from input to output.
			output <- audioData
		} else {
			// Regular time delay processing.
			// Add your time delay processing logic here.
			// ...

			// Send processed data to the next stage.
			output <- audioData
		}
	}

	// Close output channel to signal completion.
	close(output)
}

// Placeholder for a function that performs convolution processing.
func convolutionStage(input <-chan AudioData, output chan<- AudioData, wg *sync.WaitGroup) {
	defer wg.Done()

	for audioData := range input {
		if isBypassModeEnabled() {
			// Bypass mode: Pass data directly from input to output.
			output <- audioData
		} else {
			// Regular convolution processing.
			// Add your convolution processing logic here.
			// ...

			// Send processed data to the next stage.
			output <- audioData
		}
	}

	// Close output channel to signal completion.
	close(output)
}

// Placeholder for a function that performs volume processing.
func volumeProcessingStage(input <-chan AudioData, output chan<- AudioData, wg *sync.WaitGroup) {
	defer wg.Done()

	for audioData := range input {
		if isBypassModeEnabled() {
			// Bypass mode: Pass data directly from input to output.
			output <- audioData
		} else {
			// Regular volume processing.
			// Add your volume processing logic here.
			// ...

			// Send processed data to the next stage.
			output <- audioData
		}
	}

	// Close output channel to signal completion.
	close(output)
}

// Placeholder for a function that performs the final output logic.
func finalOutput(input <-chan AudioData) {
	// Implement your final output logic here.
	// This function receives the processed audio data and can be adjusted based on your specific needs.
	for audioData := range input {
		// Add logic for final output processing.
		// ...
	}
}
