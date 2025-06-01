package main

import (
	"bytes"
	"encoding/binary"
	"math"
	"os"
)

func main() {
	const (
		sampleRate = 44100
		duration   = 1.0 // 1 second
		amp        = 0.8 // -2dB to avoid clipping
	)

	numSamples := int(sampleRate * duration)

	// Create WAV header
	header := bytes.Buffer{}
	header.Write([]byte("RIFF"))
	binary.Write(&header, binary.LittleEndian, uint32(36+numSamples*2)) // File size
	header.Write([]byte("WAVEfmt "))
	binary.Write(&header, binary.LittleEndian, uint32(16))           // fmt chunk size
	binary.Write(&header, binary.LittleEndian, uint16(1))            // PCM format
	binary.Write(&header, binary.LittleEndian, uint16(1))            // 1 channel
	binary.Write(&header, binary.LittleEndian, uint32(sampleRate))   // Sample rate
	binary.Write(&header, binary.LittleEndian, uint32(sampleRate*2)) // Byte rate
	binary.Write(&header, binary.LittleEndian, uint16(2))            // Block align
	binary.Write(&header, binary.LittleEndian, uint16(16))           // Bits/sample
	header.Write([]byte("data"))
	binary.Write(&header, binary.LittleEndian, uint32(numSamples*2)) // Data size

	// Create file
	file, _ := os.Create("testimpulse.wav")
	defer file.Close()
	file.Write(header.Bytes())

	// Generate impulse + silence
	maxVal := float64(math.MaxInt16) // Convert to float64 first
	for i := 0; i < numSamples; i++ {
		var sample int16
		if i == 0 {
			// Proper type conversion: float → int16
			sample = int16(amp * maxVal)
		}
		binary.Write(file, binary.LittleEndian, sample)
	}
}
