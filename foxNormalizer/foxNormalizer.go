// Package: github.com/Foxenfurter/foxAudioLib/foxNormalizer
// filename foxNormalizer.go
// Package is designed to manage normalization of audio samples

package foxNormalizer

import (
	"math"
)

// Calculate the target gain level based on the sampling frequency
// parameters: From Sample Rate, To Sample Rate (int) returns the target gain level as float64
func TargetGainold(fromSampleRate, toSampleRate int, BaseLevel float64) float64 {
	// Let's normalize EQ Impulse
	NormaliseRatio := float64(fromSampleRate) / float64(toSampleRate)
	var targetLevel float64

	// 0.05 too high, 0.025 too low
	if toSampleRate == 192000 || toSampleRate == 176000 {
		// I have tweaked the formula for highest sample rate

		targetLevel = BaseLevel * (NormaliseRatio + (0.01 * 1 / NormaliseRatio))
	} else {
		targetLevel = BaseLevel * (NormaliseRatio + (0.035 * 1 / NormaliseRatio))
	}

	return targetLevel
}

func TargetGainCSharp(myInputSampleRate, myOutputSampleRate int, BaseLevel float64) float64 {
	targetLevel := BaseLevel
	normaliseRatio := float64(myInputSampleRate) / float64(myOutputSampleRate)
	if float64(myInputSampleRate)*1.1 < float64(myOutputSampleRate) {
		// lets normalise EQ Impulse
		targetLevel = targetLevel * (normaliseRatio + (0.2 * normaliseRatio))
	}
	if myInputSampleRate == 192000 || myInputSampleRate == 176000 {
		// I have tweaked the formula for highest sample rate

		targetLevel = targetLevel * (normaliseRatio + (0.01 * 1 / normaliseRatio))
	}
	if (myInputSampleRate == 96000 || myInputSampleRate == 88200) && (myOutputSampleRate == 192000 || myOutputSampleRate == 176400) {
		// I have tweaked the formula for highest sample rate as the low pass filter actually boosts levels following normalisation (average level)

		targetLevel = targetLevel * (normaliseRatio + (0.007 * 1 / normaliseRatio))
	}
	return targetLevel
}

func TargetGain(inputRate, outputRate int, baseLevel float64) float64 {
	// Calculate true resampling ratio
	ratio := float64(outputRate) / float64(inputRate)

	// For downsampling: compensate for filter attenuation
	if outputRate < inputRate {
		// Empirical correction factor (adjust based on your filter)
		// Typically 0.9-0.99 for good low-pass filters
		filterAttenuation := 0.95
		return baseLevel * (1.0 / (ratio * filterAttenuation))
	}

	// For upsampling: maintain original level
	return baseLevel
}

// Calculates the Maximum Gain Value - used for Normalization functions
func calculateMaxGain(audioData []float64) float64 {
	maxGain := 0.0
	if audioData == nil {
		return maxGain
	}
	for _, sample := range audioData {
		sampleAbs := math.Abs(sample)
		if sampleAbs > maxGain {
			maxGain = sampleAbs
		}
	}
	return maxGain
}

// Normalises and audio signal to the target level using a max and target level.
// Max Level has been pre-calculated across all channels
func normalizeAudioChannel(audioImpulse []float64, targetLevel float64, max float64) []float64 {
	// Check for divide by zero and no normalization needed

	if max == 0.0 || max == targetLevel {
		return audioImpulse
	}
	//println("NormalizeAudioImpulse max:", max, " target:", targetLevel)
	// Calculate the normalization factor
	normalizationFactor := targetLevel / max

	// Normalize the audio impulse
	normalizedImpulse := make([]float64, len(audioImpulse))
	for i := range audioImpulse {
		normalizedImpulse[i] = audioImpulse[i] * normalizationFactor
	}

	return normalizedImpulse
}

// Normalizes Audio Data in supplied samples
func Normalize(inputSamples [][]float64, targetLevel float64) float64 {
	// Find max gain on all channels
	impulseGain := 0.0
	for _, channel := range inputSamples {
		impulseGain = math.Max(calculateMaxGain(channel), math.Abs(impulseGain))
	}
	if impulseGain == 0 {
		return impulseGain
	}

	// Normalize gain
	for i := range inputSamples {
		inputSamples[i] = normalizeAudioChannel(inputSamples[i], targetLevel, impulseGain) // Replace with actual implementation

	}

	return impulseGain
}
