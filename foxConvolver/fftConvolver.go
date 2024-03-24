// Package: github.com/Foxenfurter/foxAudioLib/foxConvolver
// filename foxConvolver.go
// Package is designed to manage convolution of Filter Impulse Responses
// includes any logic neeeded to prepare the filter for convolution, such as Resampling and Normalization
package foxConvolver

import (
	"math"

	gofft "github.com/argusdusty/gofft"
	"scientificgo.org/fft"
)

// FFTConvolver struct represents the FFT convolver.
type FFTConvolver struct {
	blockSize      int
	segSize        int
	segCount       int
	fftComplexSize int
	segments       [][]complex128
	segmentsIR     [][]complex128

	//segments       [][]float64
	//segmentsIR     [][]float64
	//fftBuffer []float64
	fftBuffer []complex128
	//fft             FFT // Placeholder for the FFT library
	preMultiplied   []complex128
	conv            []complex128
	overlap         []float64
	current         int
	inputBuffer     []float64
	inputBufferFill int
}

// NewFFTConvolver creates a new instance of FFTConvolver.
func NewFFTConvolver() *FFTConvolver {
	return &FFTConvolver{}
}

// Reset resets the FFTConvolver to its initial state.
func (convolver *FFTConvolver) Reset() {
	for i := 0; i < convolver.segCount; i++ {
		//convolver.segments[i] = nil
		//convolver.segmentsIR[i] = nil
	}
	convolver.blockSize = 0
	convolver.segSize = 0
	convolver.segCount = 0
	convolver.fftComplexSize = 0
	convolver.segments = nil
	convolver.segmentsIR = nil
	convolver.fftBuffer = nil
	//convolver.preMultiplied = nil
	//convolver.conv = nil
	convolver.overlap = nil
	convolver.current = 0
	convolver.inputBuffer = nil
	convolver.inputBufferFill = 0
}

// Init initializes the FFTConvolver with the given parameters.
// param blockSize Block size internally used by the convolver (partition size)
// param ir The impulse response
// param irLen Length of the impulse response
// return true: Success - false: Failed
func (convolver *FFTConvolver) Init(blockSize int, ir []float64, irLen int) bool {
	convolver.Reset()

	if blockSize == 0 {
		return false
	}

	// Ignore zeros at the end of the impulse response because they only waste computation time
	for irLen > 0 && math.Abs(ir[irLen-1]) < 0.000001 {
		irLen--
	}

	if irLen == 0 {
		return true
	}

	convolver.blockSize = NextPowerOf2(blockSize)
	convolver.segSize = 2 * convolver.blockSize
	convolver.segCount = int(math.Ceil(float64(irLen) / float64(convolver.blockSize)))
	convolver.fftComplexSize = convolver.segSize

	// FFT
	//convolver.fft.Init(convolver.segSize)
	convolver.fftBuffer = make([]complex128, convolver.segSize)

	// Prepare segments
	/*convolver.segments = make([][]float64, convolver.segCount)
	for i := 0; i < convolver.segCount; i++ {
		convolver.segments[i] = make([]float64, convolver.fftComplexSize)
	}
	*/
	convolver.segments = make([][]complex128, convolver.segCount)
	for i := 0; i < convolver.segCount; i++ {
		convolver.segments[i] = make([]complex128, convolver.fftComplexSize)
	}
	// Prepare IR
	//convolver.segmentsIR = make([][]float64, convolver.segCount)
	convolver.segmentsIR = make([][]complex128, convolver.segCount)
	for i := 0; i < convolver.segCount; i++ {
		//segment := make([]float64, convolver.fftComplexSize)
		segment := make([]complex128, convolver.fftComplexSize)
		remaining := irLen - (i * convolver.blockSize)
		sizeCopy := convolver.blockSize
		if remaining < sizeCopy {
			sizeCopy = remaining
		}

		CopyAndPad(segment, gofft.Float64ToComplex128Array(ir[i*convolver.blockSize:]), sizeCopy)
		// float to complex conversion
		// fft complex
		// complex to float conversion - inefficient but let's see if we can make it work
		//tmpcmplx := gofft.Float64ToComplex128Array(convolver.fftBuffer)
		segment = fft.Fft(convolver.fftBuffer, false)
		//	tmpcmplx = fft.Fft(tmpcmplx, false)
		//segment = gofft.Complex128ToFloat64Array(tmpcmplx)
		//	convolver.fft.FFT(convolver.fftBuffer, segment)

		convolver.segmentsIR[i] = segment
	}

	// Prepare convolution buffers
	convolver.preMultiplied = make([]complex128, convolver.fftComplexSize)
	convolver.conv = make([]complex128, convolver.fftComplexSize)
	convolver.overlap = make([]float64, convolver.blockSize)

	// Prepare input buffer
	convolver.inputBuffer = make([]float64, convolver.blockSize)
	convolver.inputBufferFill = 0

	// Reset current position
	convolver.current = 0

	println("blocksize: ", convolver.blockSize, "segSize: ", convolver.segSize, "segCount: ", convolver.segCount, "fftComplexSize: ", convolver.fftComplexSize)
	println("segments: ", len(convolver.segments), "segementsIR: ", len(convolver.segmentsIR), "overlap:", len(convolver.overlap))
	println("inputBuffer : ", len(convolver.inputBuffer), "inputBufferFill : ", convolver.inputBufferFill, "current: ", convolver.current)

	return true
}

// Process processes the input data and produces the output.
// Convolves the the given input samples and immediately outputs the result
// param input The input samples
// param output The convolution result
// param len Number of input/output samples
func (convolver *FFTConvolver) Process(input []float64) []float64 {
	len := len(input)
	output := make([]float64, len)
	if convolver.segCount == 0 {
		for i := 0; i < len; i++ {
			output[i] = 0
		}
		return input
	}

	processed := 0
	//println("Running Process")
	for processed < len {
		inputBufferWasEmpty := convolver.inputBufferFill == 0
		processing := len - processed
		if processing > convolver.blockSize-convolver.inputBufferFill {
			processing = convolver.blockSize - convolver.inputBufferFill
		}
		inputBufferPos := convolver.inputBufferFill
		copy(convolver.inputBuffer[inputBufferPos:], input[processed:processed+processing])

		// Forward FFT
		CopyAndPad(convolver.fftBuffer, gofft.Float64ToComplex128Array(convolver.inputBuffer), convolver.blockSize)
		convolver.segments[convolver.current] = fft.Fft(convolver.fftBuffer, false)

		//tmpcmplx := gofft.Float64ToComplex128Array(convolver.fftBuffer)
		//tmpcmplx = fft.Fft(tmpcmplx, false)
		//convolver.segments[convolver.current] = gofft.Complex128ToFloat64Array(tmpcmplx)
		//convolver.segments[convolver.current] = gofft.Complex128ToFloat64Array(fft.Fft(tmpcmplx, false))
		//convolver.fft.FFT(convolver.fftBuffer, convolver.segments[convolver.current])

		// Complex multiplication
		if inputBufferWasEmpty {
			convolver.preMultiplied = make([]complex128, convolver.fftComplexSize)
			SetZero(convolver.preMultiplied)
			for i := 1; i < convolver.segCount; i++ {
				indexIr := i
				indexAudio := (convolver.current + i) % convolver.segCount
				NewComplexMultiplyAccumulate(convolver.preMultiplied, convolver.segmentsIR[indexIr], convolver.segments[indexAudio])
			}
		}
		copy(convolver.conv, convolver.preMultiplied)

		NewComplexMultiplyAccumulate(convolver.conv, convolver.segments[convolver.current], convolver.segmentsIR[0])
		//**** This is just to confirm that convolver.segments[convolver.current] holds data. It does!
		//convolver.conv = convolver.segments[convolver.current]
		// Backward FFT
		convolver.fftBuffer = fft.Fft(convolver.conv, true)
		//I had this the wrong way around and was getting sound but not convolved...
		//convolver.conv = fft.Fft(convolver.fftBuffer, true)

		//tmpcmplx = gofft.Float64ToComplex128Array(convolver.fftBuffer)
		//convolver.conv = fft.Fft(tmpcmplx, true)

		//convolver.fft.IFFT(convolver.fftBuffer, convolver.conv)

		// Add overlap
		Sum(output[processed:], gofft.Complex128ToFloat64Array(convolver.fftBuffer[inputBufferPos:]), convolver.overlap[inputBufferPos:], processing)

		// Input buffer full => Next block
		convolver.inputBufferFill += processing
		if convolver.inputBufferFill == convolver.blockSize {
			// Input buffer is empty again now
			for i := 0; i < convolver.blockSize; i++ {
				convolver.inputBuffer[i] = 0
			}
			convolver.inputBufferFill = 0

			// Save the overlap
			copy(convolver.overlap, gofft.Complex128ToFloat64Array(convolver.fftBuffer[convolver.blockSize:]))

			// Update current segment
			convolver.current--
			if convolver.current < 0 {
				convolver.current = convolver.segCount - 1
			}
		}

		processed += processing
	}
	return output
}

// CopyAndPad copies the source array into the destination buffer and pads
// the destination buffer with zeros if necessary.
func CopyAndPad(dest, src []complex128, srcSize int) {
	destSize := len(dest)
	if destSize < srcSize {
		srcSize = destSize
	}
	copy(dest[:srcSize], src[:srcSize])
	for i := srcSize; i < destSize; i++ {
		dest[i] = complex(0, 0)
	}
}

func SetZero(dest []complex128) {
	destSize := len(dest)
	for i := 0; i < destSize; i++ {
		dest[i] = complex(0, 0)
	}
}

func NewComplexMultiplyAccumulate(result []complex128, a, b []complex128) {
	if len(result) != len(a) || len(result) != len(b) {
		panic("result, a, and b arrays must have the same length")
	}
	/*	for i := 0; i < len(result); i++ {
			result[i] = result[i] + a[i]*b[i]
		}
	*/

	end4 := len(result) / 4 * 4
	for i := 0; i < end4; i += 4 {
		result[i+0] += complex(real(a[i+0])*real(b[i+0])-imag(a[i+0])*imag(b[i+0]), real(a[i+0])*imag(b[i+0])+imag(a[i+0])*real(b[i+0]))
		result[i+1] += complex(real(a[i+1])*real(b[i+1])-imag(a[i+1])*imag(b[i+1]), real(a[i+1])*imag(b[i+1])+imag(a[i+1])*real(b[i+1]))
		result[i+2] += complex(real(a[i+2])*real(b[i+2])-imag(a[i+2])*imag(b[i+2]), real(a[i+2])*imag(b[i+2])+imag(a[i+2])*real(b[i+2]))
		result[i+3] += complex(real(a[i+3])*real(b[i+3])-imag(a[i+3])*imag(b[i+3]), real(a[i+3])*imag(b[i+3])+imag(a[i+3])*real(b[i+3]))
	}
	for i := end4; i < len(result); i++ {
		result[i] += complex(real(a[i])*real(b[i])-imag(a[i])*imag(b[i]), real(a[i])*imag(b[i])+imag(a[i])*real(b[i]))
	}

}

// ComplexMultiplyAccumulate multiplies the elements of complex arrays a and b and
// adds the result to the complex array result.
func OldComplexMultiplyAccumulate(result, a, b SplitComplexOld) {
	if len(result.re) != len(a.re) || len(result.re) != len(b.re) {
		panic("result, a, and b arrays must have the same length")
	}

	oldcomplexMultiplyAccumulate(result.re, result.im, a.re, a.im, b.re, b.im, len(result.re))
}

func oldcomplexMultiplyAccumulate(re, im, reA, imA, reB, imB []float64, len int) {
	end4 := len / 4 * 4
	for i := 0; i < end4; i += 4 {
		re[i+0] += reA[i+0]*reB[i+0] - imA[i+0]*imB[i+0]
		re[i+1] += reA[i+1]*reB[i+1] - imA[i+1]*imB[i+1]
		re[i+2] += reA[i+2]*reB[i+2] - imA[i+2]*imB[i+2]
		re[i+3] += reA[i+3]*reB[i+3] - imA[i+3]*imB[i+3]
		im[i+0] += reA[i+0]*imB[i+0] + imA[i+0]*reB[i+0]
		im[i+1] += reA[i+1]*imB[i+1] + imA[i+1]*reB[i+1]
		im[i+2] += reA[i+2]*imB[i+2] + imA[i+2]*reB[i+2]
		im[i+3] += reA[i+3]*imB[i+3] + imA[i+3]*reB[i+3]
	}
	for i := end4; i < len; i++ {
		re[i] += reA[i]*reB[i] - imA[i]*imB[i]
		im[i] += reA[i]*imB[i] + imA[i]*reB[i]
	}
}

// Sum adds the elements of arrays a and b element-wise and stores the result in result.
func Sum(result, a, b []float64, len int) {
	for i := 0; i < len; i++ {
		result[i] = a[i] + b[i]
	}
}

type SplitComplex []complex128

// SplitComplex represents a buffer for split-complex representation of FFT results.
type SplitComplexOld struct {
	size int
	re   []float64
	im   []float64
}

// NewSplitComplex creates a new SplitComplex instance with the specified initial size.
/*func NewSplitComplex(initialSize int) *SplitComplex {
	sc := &SplitComplex{}
	sc.Resize(initialSize)
	return sc
}

// Clear clears the split-complex buffer.
func (sc *SplitComplex) Clear() {
	sc.re.Clear()
	sc.im.Clear()
	sc.size = 0
}

// Resize resizes the split-complex buffer to the specified size.
func (sc *SplitComplex) Resize(newSize int) {
	sc.re.Resize(newSize)
	sc.im.Resize(newSize)
	sc.size = newSize
}

// SetZero sets all elements of the split-complex buffer to zero.
func (sc *SplitComplex) SetZero() {
	sc.re.SetZero()
	sc.im.SetZero()
}

// CopyFrom copies the content of another SplitComplex instance to this instance.
func (sc *SplitComplex) CopyFrom(other *SplitComplex) {
	sc.re.CopyFrom(other.re)
	sc.im.CopyFrom(other.im)
}
*/
// Re returns the real part of the split-complex buffer.
func (sc *SplitComplexOld) Re() []float64 {
	return sc.re
}

// Im returns the imaginary part of the split-complex buffer.
func (sc *SplitComplexOld) Im() []float64 {
	return sc.im
}

// Size returns the size of the split-complex buffer.
func (sc *SplitComplexOld) Size() int {
	return sc.size
}
