package foxFFT

import (
	"fmt"
	"math"
	"math/bits"
	"math/cmplx"
	"sync"
)

const packageName = "foxFFT"

func Fft(x []complex128, inverse bool) error {
	if err := checkLength("FFT Input", len(x)); err != nil {
		return err
	}
	if inverse {
		ifft128(x)
	} else {
		fft128enhanced(x)
	}
	return nil
}

func FftUnchecked(x []complex128, inverse bool) error {

	if inverse {
		ifft128(x)
	} else {
		fft128enhanced(x)
	}
	return nil
}

// FFT package functions

func permute(x []complex128) {
	N := len(x)
	switch N {
	case 1, 2:
		return
	case 4:
		x[1], x[2] = x[2], x[1]
		return
	case 8:
		x[1], x[4] = x[4], x[1]
		x[3], x[6] = x[6], x[3]
		return
	}
	shift := 64 - uint64(bits.Len64(uint64(N-1)))
	N2 := N >> 1
	for i := 0; i < N; i += 2 {
		ind := int(bits.Reverse64(uint64(i)) >> shift)
		if ind > i {
			x[i], x[ind] = x[ind], x[i]
		}
		ind |= N2
		if ind > i+1 {
			x[i+1], x[ind] = x[ind], x[i+1]
		}
	}
}

// ===========================================
// 2. CACHE-OPTIMIZED FFT WITH BLOCK PROCESSING
// ===========================================
// Cache line size in bytes (64 bytes = 8 complex128)
const cacheLineComplex = 8

var twiddleCache = make(map[int][]complex128)
var twiddleCacheLock sync.RWMutex

func getTwiddles(N int) []complex128 {
	twiddleCacheLock.RLock()
	if twiddles, ok := twiddleCache[N]; ok {
		twiddleCacheLock.RUnlock()
		return twiddles
	}
	twiddleCacheLock.RUnlock()

	twiddleCacheLock.Lock()
	defer twiddleCacheLock.Unlock()

	if twiddles, ok := twiddleCache[N]; ok {
		return twiddles
	}

	twiddles := make([]complex128, N)
	// Use math.Sincos - it's measurably faster than separate Cos/Sin
	for k := 0; k < N; k++ {
		angle := -2 * math.Pi * float64(k) / float64(N)
		sin, cos := math.Sincos(angle) // Single function call
		twiddles[k] = complex(cos, sin)
	}

	twiddleCache[N] = twiddles
	return twiddles
}

//===========================

func IsPow2(N int) bool {
	if N == 0 {
		return false
	}
	return (uint64(N) & uint64(N-1)) == 0
}

func checkLength(Context string, N int) error {
	if !IsPow2(N) {
		return fmt.Errorf("%s must be a power of 2, got %d", Context, N)
	}
	return nil
}

// Instead of w = cmplx.Sqrt(w) in loop
// Pre-compute all needed roots of unity

func fft128enhanced(x []complex128) {
	N := len(x)

	permute(x)

	// First 2 steps
	for i := 0; i < N; i += 4 {
		f := complex(imag(x[i+2])-imag(x[i+3]), real(x[i+3])-real(x[i+2]))
		x[i], x[i+1], x[i+2], x[i+3] = x[i]+x[i+1]+x[i+2]+x[i+3], x[i]-x[i+1]+f, x[i]-x[i+2]+x[i+1]-x[i+3], x[i]-x[i+1]-f
	}

	twiddles := getTwiddles(N)

	for n := 4; n < N; n <<= 1 {
		m := n << 1
		step := N / m
		for o := 0; o < N; o += m {
			for k := 0; k < n; k++ {
				i := k + o
				f := twiddles[k*step] * x[i+n]
				x[i], x[i+n] = x[i]+f, x[i]-f
			}
		}
	}
}

// fft does the actual work for FFT

func ifft128(x []complex128) {
	N := len(x)
	// Reverse using XOR swap (slightly faster, no temp needed)
	for i := 1; i < N/2; i++ {
		j := N - i
		x[i], x[j] = x[j], x[i]
	}

	fft128enhanced(x)

	// Scale - compiler will optimize division by constant better
	invN := 1.0 / float64(N)
	for i := range x {
		x[i] = complex(real(x[i])*invN, imag(x[i])*invN)
	}
}

// TukeyFFT simple Cooley-Tukey based FFT
func TukeyFFT(X []complex128) []complex128 {
	N := len(X)
	if N <= 1 {
		return X
	}

	// Divide
	even := make([]complex128, N/2)
	odd := make([]complex128, N/2)
	for i := 0; i < N/2; i++ {
		even[i] = X[i*2]
		odd[i] = X[i*2+1]
	}

	// Conquer
	T := TukeyFFT(even)
	U := TukeyFFT(odd)
	x := make([]complex128, N)
	// Combine
	for i := 0; i < N/2; i++ {
		t := cmplx.Rect(1, -2*math.Pi*float64(i)/float64(N)) * U[i]
		x[i] = T[i] + t
		x[i+N/2] = T[i] - t
	}

	return x
}

// InverseTukeyFFT simple Cooley-Tukey based inverse FFT
func InverseTukeyFFT(X []complex128) []complex128 {
	N := len(X)
	x := make([]complex128, N)

	// Take the conjugate of the input sequence
	for i := 0; i < N; i++ {
		x[i] = cmplx.Conj(X[i])
	}

	// Compute the FFT of the conjugated sequence
	x = TukeyFFT(x)

	// Take the conjugate of the result and normalize
	for i := 0; i < N; i++ {
		x[i] = cmplx.Conj(x[i]) / complex(float64(N), 0)
	}

	return x
}
