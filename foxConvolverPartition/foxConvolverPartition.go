package foxConvolverPartition

import (
	"bytes"
	"encoding/gob"
	"fmt"
	"math/bits"
	"math/cmplx"
	"strconv"

	fft "github.com/Foxenfurter/foxAudioLib/foxFFT"
)

var (
	PartitionedChunkSize        = 4096 // New samples per chunk (L)
	PartitionedFFTSize          = 8192 // FFT size = 2 × chunk size (N)
	PartitionedImpulseChunkSize = 4096 // Impulse partition size
	//PartitionedOverlapSize      = 4095 // M - 1 = samples to discard/overlap
	PartitionedOverlapSize = 4096 // M - 1 = samples to discard/overlap

)

const packageName = "foxConvolverPartition"

type PartitionedConvolver struct {
	FilterImpulse     []float64
	impulseLength     int
	signalBlockLength int
	outputLength      int // Target output block size for downstream

	// Partitioned convolution state
	impulsePartitions [][]complex128
	numPartitions     int

	// Ring buffer for audio chunk FFTs
	audioFFTRing [][]complex128
	ringPosition int
	ringSize     int

	ringFilled bool // Indicates if we've filled the ring at least once
	// Overlap-save state (algorithm state, not unprocessed input)
	previousChunkTail []float64 // Last 2047 samples from previous chunk - chunkTailCapacity

	// Input buffering - accumulates until we have 2048 samples to process
	Buffer []float64

	// Output buffering - accumulates processed output until we have outputLength samples to send
	outputBuffer []float64

	overlapTail []float64 // For compatibility

	// Pre-allocated working buffers
	workFFT    []complex128
	workResult []complex128
	MaxGain    float64

	// Debug
	DebugOn     bool
	DebugFunc   func(string)
	WarningFunc func(string)
}

func SetChunkSizeForSampleRate(sampleRate int) {
	switch {
	case sampleRate >= 176400: // 192kHz, 176.4kHz
		PartitionedChunkSize = 8192
	case sampleRate >= 88200: // 96kHz, 88.2kHz
		PartitionedChunkSize = 4096
	default: // 44.1kHz, 48kHz
		PartitionedChunkSize = 4096
	}

	PartitionedFFTSize = PartitionedChunkSize * 2
	PartitionedImpulseChunkSize = PartitionedChunkSize
	PartitionedOverlapSize = PartitionedChunkSize

}

func NewPartitionedConvolver(impulse []float64, sampleRate int) *PartitionedConvolver {
	// Set optimal chunk sizes for this sample rate
	if sampleRate > 0 {
		SetChunkSizeForSampleRate(sampleRate)
	}

	pc := &PartitionedConvolver{

		FilterImpulse: impulse,
		impulseLength: len(impulse),
		//Buffer:            make([]float64, 0),
		//outputBuffer:      make([]float64, 0),
		// Pre-allocate with enough capacity to avoid reallocations
		Buffer:            make([]float64, 0, PartitionedChunkSize*2),
		outputBuffer:      make([]float64, 0, PartitionedChunkSize*4),
		previousChunkTail: make([]float64, PartitionedOverlapSize),
		workFFT:           make([]complex128, PartitionedFFTSize),
		workResult:        make([]complex128, PartitionedFFTSize),
		overlapTail:       make([]float64, 0),
		outputLength:      PartitionedChunkSize, // Default to chunk size
		// intitialises ringbuffer
		ringFilled: false,
	}
	//pc.DebugOn = true // Set to true for debug messages
	pc.debug(packageName + "InitForStreaming - Creating New partition")
	pc.initializePartitions()

	return pc
}

func NextPowerOf2(n int) int {
	if n <= 0 {
		return 1
	}
	return 1 << bits.Len(uint(n-1))
}

func (pc *PartitionedConvolver) initializePartitions() {
	if pc.impulseLength == 0 {
		pc.numPartitions = 0
		pc.ringSize = 1 // Must be at least 1 to avoid divide by zero
		pc.impulsePartitions = make([][]complex128, 0)
		pc.audioFFTRing = make([][]complex128, 1)
		pc.audioFFTRing[0] = make([]complex128, PartitionedFFTSize)
		pc.ringPosition = 0

		if len(pc.previousChunkTail) == 0 {
			pc.previousChunkTail = make([]float64, PartitionedOverlapSize)
		}

		pc.debug("PartitionedConvolver initialized: empty impulse (passthrough)")

		return
	}

	pc.numPartitions = (pc.impulseLength + PartitionedImpulseChunkSize - 1) / PartitionedImpulseChunkSize
	pc.impulsePartitions = make([][]complex128, pc.numPartitions)

	for p := 0; p < pc.numPartitions; p++ {
		partition := make([]complex128, PartitionedFFTSize)

		start := p * PartitionedImpulseChunkSize
		end := start + PartitionedImpulseChunkSize
		if end > pc.impulseLength {
			end = pc.impulseLength
		}

		for i := start; i < end; i++ {
			partition[i-start] = complex(pc.FilterImpulse[i], 0)
		}

		// FFT operates in-place - returns result
		fft.Fft(partition, false)
		pc.impulsePartitions[p] = partition
	}

	// Initialize ring buffer
	pc.ringSize = pc.numPartitions
	pc.audioFFTRing = make([][]complex128, pc.ringSize)
	for i := range pc.audioFFTRing {
		pc.audioFFTRing[i] = make([]complex128, PartitionedFFTSize)
	}
	pc.ringPosition = 0

	pc.debug("PartitionedConvolver initialized: " +
		strconv.Itoa(pc.numPartitions) + " partitions, " +
		"ring size: " + strconv.Itoa(pc.ringSize))

	pc.debug(fmt.Sprintf("Chunk=%d, FFT=%d, ImpChunk=%d, Overlap=%d",
		PartitionedChunkSize, PartitionedFFTSize,
		PartitionedImpulseChunkSize, PartitionedOverlapSize))
	// Initialize overlap tail for algorithm state
	if len(pc.previousChunkTail) == 0 {
		pc.previousChunkTail = make([]float64, PartitionedOverlapSize)
	}
	pc.MaxGain = pc.maxGainFromPartitions()

}

func (pc *PartitionedConvolver) InitForStreaming() {
	if len(pc.FilterImpulse) != pc.impulseLength {
		pc.impulseLength = len(pc.FilterImpulse)
		pc.initializePartitions() // Always re-partition when length changes
	} else if len(pc.impulsePartitions) == 0 && len(pc.FilterImpulse) > 0 {
		pc.initializePartitions()
	}

	if pc.outputLength == 0 {
		pc.outputLength = PartitionedChunkSize
	}

	if pc.DebugOn {
		pc.debug(packageName + ": InitForStreaming - Impulse Length: " +
			strconv.Itoa(pc.impulseLength) + " Output Length: " +
			strconv.Itoa(pc.outputLength) + " Ring Position: " +
			strconv.Itoa(pc.ringPosition) + " Ring Filled: " + strconv.FormatBool(pc.ringFilled))
	}
}

func (pc *PartitionedConvolver) processChunk(chunk []float64, updateTail bool) []float64 {
	if pc.ringSize == 0 {
		return chunk
	}

	L := PartitionedChunkSize
	N := PartitionedFFTSize
	overlapSize := L - 1

	currentSlot := pc.audioFFTRing[pc.ringPosition]
	clear(currentSlot)

	// Copy overlap from previous chunk
	for i := 0; i < overlapSize; i++ {
		currentSlot[i] = complex(pc.previousChunkTail[i], 0)
	}

	// Copy new chunk
	for i := 0; i < L; i++ {
		currentSlot[overlapSize+i] = complex(chunk[i], 0)
	}

	// FFT current block
	fft.FftUnchecked(currentSlot, false)
	pc.audioFFTRing[pc.ringPosition] = currentSlot

	// Save tail ONLY if this is a real chunk (not zero-padded)
	if updateTail {
		copy(pc.previousChunkTail, chunk[L-overlapSize:])
	}

	// Accumulate partitions - WITH COLD START PROTECTION
	clear(pc.workResult)

	var maxPartitions int
	if pc.ringFilled {
		maxPartitions = pc.numPartitions
	} else {
		maxPartitions = pc.ringPosition + 1
		if maxPartitions > pc.numPartitions {
			maxPartitions = pc.numPartitions
		}

	}

	for p := 0; p < maxPartitions; p++ {
		audioIdx := (pc.ringPosition - p + pc.ringSize) % pc.ringSize
		audioFFT := pc.audioFFTRing[audioIdx]
		impulseFFT := pc.impulsePartitions[p]

		i := 0
		for i < N-3 {
			pc.workResult[i] += audioFFT[i] * impulseFFT[i]
			pc.workResult[i+1] += audioFFT[i+1] * impulseFFT[i+1]
			pc.workResult[i+2] += audioFFT[i+2] * impulseFFT[i+2]
			pc.workResult[i+3] += audioFFT[i+3] * impulseFFT[i+3]
			i += 4
		}
		for ; i < N; i++ {
			pc.workResult[i] += audioFFT[i] * impulseFFT[i]
		}
	}

	// IFFT
	fft.FftUnchecked(pc.workResult, true)

	// Extract output: discard first (L-1), keep next L
	output := make([]float64, L)
	for i := 0; i < L; i++ {
		output[i] = real(pc.workResult[overlapSize+i])
	}

	// Advance ring
	pc.ringPosition = (pc.ringPosition + 1) % pc.ringSize
	if !pc.ringFilled && pc.ringPosition == 0 {
		pc.ringFilled = true
	}

	return output
}

// ConvolveChannel - streaming convolution via channels
// Handles BOTH input and output buffering for proper gapless playback
// For GAPLESS: Does NOT flush at end - both buffers are saved for next track
// In ConvolveChannel:
func (pc *PartitionedConvolver) ConvolveChannel(inputSignalChannel, outputSignalChannel chan []float64) {
	functionName := "ConvolveChannel"
	NoConvolverMessage := false
	CanConvolve := len(pc.FilterImpulse) > 0

	if CanConvolve {
		pc.InitForStreaming()

		if pc.DebugOn {
			pc.debug(packageName + ": " + functionName + ": Partitioned convolution ready")
		}
	}

	for inputBlock := range inputSignalChannel {
		if !CanConvolve {
			if !NoConvolverMessage {
				pc.debug(packageName + ": " + functionName + ": Nothing to convolve with")
				NoConvolverMessage = true
			}
			outputSignalChannel <- inputBlock
			continue
		}

		// Accumulate input
		pc.Buffer = append(pc.Buffer, inputBlock...)

		// Process complete 2048-sample chunks
		for len(pc.Buffer) >= PartitionedChunkSize {
			chunk := pc.Buffer[:PartitionedChunkSize]
			processedOutput := pc.processChunk(chunk, true) // Update tail only for real chunks
			pc.outputBuffer = append(pc.outputBuffer, processedOutput...)
			pc.Buffer = pc.Buffer[PartitionedChunkSize:]
		}

		// Send complete output blocks
		for len(pc.outputBuffer) >= pc.outputLength {
			outputSignalChannel <- pc.outputBuffer[:pc.outputLength]
			pc.outputBuffer = pc.outputBuffer[pc.outputLength:]
		}

		if !NoConvolverMessage && pc.DebugOn {
			pc.debug(packageName + ": " + functionName + ": Partitioned convolution active")
			NoConvolverMessage = true
		}
	}

	// END OF TRACK - Process remaining samples

	// 1. Process any remaining input (< 2048) with zero-padding
	if len(pc.Buffer) > 0 {
		remainingSamples := len(pc.Buffer)

		if pc.DebugOn {
			pc.debug(packageName + ": " + functionName +
				": Processing final " + strconv.Itoa(remainingSamples) + " samples with zero-padding")
		}

		// Create zero-padded chunk
		paddedChunk := make([]float64, PartitionedChunkSize)
		copy(paddedChunk, pc.Buffer)
		// Rest is already zeros

		// Process the padded chunk
		finalOutput := pc.processChunk(paddedChunk, false) // Do NOT update tail for zero-padded chunk

		// Only keep the valid output (corresponding to the actual input samples)
		// The rest is convolution tail that will affect the NEXT track
		pc.outputBuffer = append(pc.outputBuffer, finalOutput[:remainingSamples]...)

		// Clear the input buffer - it's been processed
		pc.Buffer = make([]float64, 0)
	}

	// 2. Flush all remaining output
	if len(pc.outputBuffer) > 0 {
		outputSignalChannel <- pc.outputBuffer
		if pc.DebugOn {
			pc.debug(packageName + ": " + functionName +
				": Flushed " + strconv.Itoa(len(pc.outputBuffer)) + " final output samples")
		}
		pc.outputBuffer = make([]float64, 0)
	}

	// 3. Algorithm state is preserved for next track
	// - previousChunkTail: last 2047 samples (including from the zero-padded chunk)
	// - audioFFTRing: ring buffer with FFT history
	// - ringPosition: current position
	// These ensure seamless continuation into next track

	if pc.DebugOn {
		pc.debug(packageName + ": " + functionName + ": Track completed. State preserved for gapless continuation.")
	}
}

// ConvolveOverlapSave processes signal blocks, accumulating input and producing output
// For GAPLESS: Does NOT process samples < PartitionedChunkSize - they remain in Buffer
// Returns variable-length output (0, 2048, 4096, etc.) based on how much input was processed
func (pc *PartitionedConvolver) ConvolveOverlapSave(signalBlock []float64) []float64 {
	if len(pc.FilterImpulse) == 0 {
		return signalBlock
	}

	// Track incoming block size
	if pc.signalBlockLength != len(signalBlock) {
		pc.signalBlockLength = len(signalBlock)
		//pc.outputLength = len(signalBlock)
	}

	// Initialize if needed
	if len(pc.impulsePartitions) == 0 {
		pc.initializePartitions()
	}

	// Accumulate and process in 2048-sample chunks
	pc.Buffer = append(pc.Buffer, signalBlock...)
	output := make([]float64, 0, len(signalBlock))

	// Process complete chunks
	for len(pc.Buffer) >= PartitionedChunkSize {
		chunk := pc.Buffer[:PartitionedChunkSize]
		chunkOutput := pc.processChunk(chunk, true) // Update tail only for real chunks
		output = append(output, chunkOutput...)
		pc.Buffer = pc.Buffer[PartitionedChunkSize:]
	}

	// Remaining samples (< 2048) stay in Buffer for gapless continuation
	return output
}

// ConvolveFFT - Single-shot NON-STREAMING convolution
// This is for complete processing of an entire signal, NOT for gapless playback
// Processes ALL samples including the final partial chunk with zero-padding

func (pc *PartitionedConvolver) ConvolveFFT(signal []float64) []float64 {
	if len(pc.FilterImpulse) == 0 {
		return signal
	}

	fullLength := len(signal) + pc.impulseLength - 1
	pc.Reset()

	// Pad signal to ensure we get the full convolution tail
	paddedSignal := make([]float64, fullLength)
	copy(paddedSignal, signal)

	result := pc.ConvolveOverlapSave(paddedSignal)

	/*	if len(pc.Buffer) > 0 {
			paddedChunk := make([]float64, PartitionedChunkSize)
			copy(paddedChunk, pc.Buffer)
			finalOutput := pc.processChunk(paddedChunk)
			remaining := min(len(pc.Buffer), len(finalOutput))
			result = append(result, finalOutput[:remaining]...)
			pc.Buffer = make([]float64, 0)
		}
	*/
	if len(pc.Buffer) > 0 {
		paddedChunk := make([]float64, PartitionedChunkSize)
		copy(paddedChunk, pc.Buffer)
		finalOutput := pc.processChunk(paddedChunk, false) // Do NOT update tail for zero-padded chunk
		result = append(result, finalOutput...)            // Take ALL output
		pc.Buffer = make([]float64, 0)
	}

	if len(result) > fullLength {
		result = result[:fullLength]
	}
	return result
}

// MaxGainFromFFT - Calculate maximum gain in frequency domain
// Utility function for filter normalization
func MaxGainFromFFT(impulse []float64) float64 {
	n := len(impulse)
	if n == 0 {
		return 0.0
	}

	paddedSize := NextPowerOf2(n)
	// Zero-pad to power of 2

	cdata := make([]complex128, paddedSize)
	for i := range n {
		cdata[i] = complex(impulse[i], 0)
	}
	fft.Fft(cdata, false) // Find maximum magnitude in frequency domain
	maxGain := 0.0
	for _, v := range cdata {
		mag := cmplx.Abs(v)
		if mag > maxGain {
			maxGain = mag
		}
	}
	return maxGain
}

func (pc *PartitionedConvolver) maxGainFromPartitions() float64 {
	maxGain := 0.0
	for _, partition := range pc.impulsePartitions {
		for _, v := range partition {
			if mag := cmplx.Abs(v); mag > maxGain {
				maxGain = mag
			}
		}
	}
	return maxGain
}

// GobEncode - Save complete state for gapless playback
// Saves:
// 1. Algorithm state (previousChunkTail, audioFFTRing, ringPosition)
// 2. Unprocessed input (Buffer) - prepended to next track's input
// 3. Unsent output (outputBuffer) - prepended to next track's output
func (pc *PartitionedConvolver) GobEncode() ([]byte, error) {
	var buf bytes.Buffer
	enc := gob.NewEncoder(&buf)

	// Save the chunk size configuration used for this filter
	if err := enc.Encode(PartitionedChunkSize); err != nil {
		return nil, err
	}
	if err := enc.Encode(PartitionedFFTSize); err != nil {
		return nil, err
	}
	if err := enc.Encode(PartitionedImpulseChunkSize); err != nil {
		return nil, err
	}
	if err := enc.Encode(PartitionedOverlapSize); err != nil {
		return nil, err
	}

	// Static configuration
	if err := enc.Encode(pc.FilterImpulse); err != nil {
		return nil, err
	}
	if err := enc.Encode(pc.impulseLength); err != nil {
		return nil, err
	}
	if err := enc.Encode(pc.outputLength); err != nil {
		return nil, err
	}

	// ADD THESE TWO LINES:
	if err := enc.Encode(pc.numPartitions); err != nil {
		return nil, err
	}
	if err := enc.Encode(pc.ringSize); err != nil {
		return nil, err
	}

	if err := enc.Encode(pc.impulsePartitions); err != nil {
		return nil, err
	}

	if err := enc.Encode(pc.MaxGain); err != nil {
		return nil, err
	}

	// Dynamic algorithm state (CRITICAL for continuity)
	/* Note: we are not saving data relating to the input audio stream as these are handled in the ResidualsCache and passed back to the Convolver on the next track load. This includes:
	if err := enc.Encode(pc.audioFFTRing); err != nil {
		return nil, err
	}
	if err := enc.Encode(pc.ringPosition); err != nil {
		return nil, err
	}
	if err := enc.Encode(pc.previousChunkTail); err != nil {
		return nil, err
	}
	*/
	return buf.Bytes(), nil
}

// GobDecode - Restore state for gapless continuation
// When loading next track:
// 1. Restore algorithm state (continues convolution seamlessly)
// 2. Restore input Buffer (unprocessed samples from previous track)
// 3. Restore output Buffer (unsent samples from previous track)
// 4. New input appends to input Buffer, new output appends to output Buffer
func (pc *PartitionedConvolver) GobDecode(data []byte) error {
	buf := bytes.NewBuffer(data)
	dec := gob.NewDecoder(buf)
	// Restore the chunk size configuration
	var chunkSize, fftSize, impulseChunkSize, overlapSize int
	if err := dec.Decode(&chunkSize); err != nil {
		return err
	}
	if err := dec.Decode(&fftSize); err != nil {
		return err
	}
	if err := dec.Decode(&impulseChunkSize); err != nil {
		return err
	}
	if err := dec.Decode(&overlapSize); err != nil {
		return err
	}

	// Set globals to match the cached filter
	PartitionedChunkSize = chunkSize
	PartitionedFFTSize = fftSize
	PartitionedImpulseChunkSize = impulseChunkSize
	PartitionedOverlapSize = overlapSize

	// Static configuration
	if err := dec.Decode(&pc.FilterImpulse); err != nil {
		return err
	}
	if err := dec.Decode(&pc.impulseLength); err != nil {
		return err
	}
	if err := dec.Decode(&pc.outputLength); err != nil {
		return err
	}

	// ADD THESE TWO LINES:
	if err := dec.Decode(&pc.numPartitions); err != nil {
		return err
	}
	if err := dec.Decode(&pc.ringSize); err != nil {
		return err
	}

	if err := dec.Decode(&pc.impulsePartitions); err != nil {
		return err
	}
	if err := dec.Decode(&pc.MaxGain); err != nil {
		return err
	}

	pc.Buffer = make([]float64, 0)
	pc.outputBuffer = make([]float64, 0)
	pc.audioFFTRing = make([][]complex128, pc.ringSize)
	for i := range pc.audioFFTRing {
		pc.audioFFTRing[i] = make([]complex128, PartitionedFFTSize)
	}
	pc.ringPosition = 0
	pc.ringFilled = false
	pc.previousChunkTail = make([]float64, PartitionedOverlapSize)
	pc.workFFT = make([]complex128, PartitionedFFTSize)
	pc.workResult = make([]complex128, PartitionedFFTSize)
	pc.overlapTail = make([]float64, 0)
	return nil
}

func (pc *PartitionedConvolver) EncodeState() ([]byte, error) {
	var buf bytes.Buffer
	enc := gob.NewEncoder(&buf)

	// Dynamic processing state only
	if err := enc.Encode(pc.audioFFTRing); err != nil {
		return nil, err
	}
	if err := enc.Encode(pc.ringPosition); err != nil {
		return nil, err
	}
	if err := enc.Encode(pc.ringFilled); err != nil {
		return nil, err
	}
	if err := enc.Encode(pc.previousChunkTail); err != nil {
		return nil, err
	}
	pc.debug("saving Ring Position: " + strconv.Itoa(pc.ringPosition) + " Ring Filled: " + strconv.FormatBool(pc.ringFilled))

	return buf.Bytes(), nil
}

func (pc *PartitionedConvolver) DecodeState(data []byte) error {
	buf := bytes.NewBuffer(data)
	dec := gob.NewDecoder(buf)

	if err := dec.Decode(&pc.audioFFTRing); err != nil {
		return err
	}
	if err := dec.Decode(&pc.ringPosition); err != nil {
		return err
	}
	if err := dec.Decode(&pc.ringFilled); err != nil {
		return err
	}
	if err := dec.Decode(&pc.previousChunkTail); err != nil {
		return err
	}

	// Clear transient buffers
	pc.Buffer = make([]float64, 0)
	pc.outputBuffer = make([]float64, 0)

	return nil
}

// Reset - Clear all state (for fresh start, NOT for gapless)
func (pc *PartitionedConvolver) Reset() {
	// Clear ring buffer (algorithm state)
	for i := range pc.audioFFTRing {
		for j := range pc.audioFFTRing[i] {
			pc.audioFFTRing[i][j] = 0
		}
	}
	pc.ringPosition = 0

	// Clear all buffers
	pc.Buffer = make([]float64, 0)
	pc.outputBuffer = make([]float64, 0)
	pc.previousChunkTail = make([]float64, PartitionedOverlapSize)

	if pc.DebugOn {
		pc.debug("PartitionedConvolver reset")
	}
}

func (pc *PartitionedConvolver) AmendFilterImpulse(impulse []float64) {
	pc.FilterImpulse = impulse
	pc.impulseLength = len(impulse)
	pc.initializePartitions()
	pc.Reset()

	if pc.DebugOn {
		pc.debug("FilterImpulse amended - " + strconv.Itoa(len(impulse)) + " samples")
	}
}

// Compatibility methods matching original convolver interface

func (pc *PartitionedConvolver) GetPaddedLength() int {
	return PartitionedFFTSize
}

// GetTail - Returns the algorithm state (overlap tail)
// This is the 2047 samples used in overlap-save algorithm
func (pc *PartitionedConvolver) GetTail() []float64 {
	tail := make([]float64, len(pc.previousChunkTail))
	copy(tail, pc.previousChunkTail)
	return tail
}

// SetTail - Sets the algorithm state (overlap tail)
func (pc *PartitionedConvolver) SetTail(tail []float64) {
	if len(tail) == PartitionedOverlapSize {
		copy(pc.previousChunkTail, tail)
	}
}

// GetInputBuffer - Returns unprocessed input samples (for debugging/inspection)
// This is what gets saved for gapless continuation
func (pc *PartitionedConvolver) GetInputBuffer() []float64 {
	buffer := make([]float64, len(pc.Buffer))
	copy(buffer, pc.Buffer)
	return buffer
}

// GetOutputBuffer - Returns unsent output samples (for debugging/inspection)
// This is also saved for gapless continuation
func (pc *PartitionedConvolver) GetOutputBuffer() []float64 {
	buffer := make([]float64, len(pc.outputBuffer))
	copy(buffer, pc.outputBuffer)
	return buffer
}

func (pc *PartitionedConvolver) SetSignalBlockLength(signalBlockLength int) {
	pc.signalBlockLength = signalBlockLength
	// Set output length to match signal block length for compatibility
	pc.outputLength = signalBlockLength
}

// Debug and warning helpers

func (pc *PartitionedConvolver) debug(message string) {
	if pc.DebugOn {
		if pc.DebugFunc != nil {
			pc.DebugFunc(message)
		} else {
			println(message)
		}
	}
}

func (pc *PartitionedConvolver) warning(message string) {
	if pc.WarningFunc != nil {
		pc.WarningFunc(message)
	} else {
		println("WARNING: " + message)
	}
}
