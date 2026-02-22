foxAudioLib is library of go packages written to support the SqueezeDSP plugin and build the FoxLyrion DSP executable.
The packages can be used independently of SqueezeDSP.

foxAudioDecoder
A general purpose audio reader which allow allows plugins for wav and pcm formats to be loaded. Other formats may be added in due course

---

foxAudioEncoder
A general purpose audio reader which allow allows plugins for wav and pcm formats to be output. Other formats may be added in due course

---

foxBufferedStdinReader

---

foxConvolver
Audio convolver supporting direct and overlap save convolution using scientificgo FFT

---

foxConvolverAdapt
Audio convolver wrapper - allows different convolver and fft to be used and switched in and out for benchmark testing

---

foxConvolverPartition
Audio Convolver supporring Partitioned Convolution and RingBuffer for Audio Stream - uses local FFT and runs 2x as fast as foxConvolver using less memory, lower latency.

---

foxDSP

---

foxFFT
Optimisation of ArgusDusty FFT - the optimisation focusses around caching twiddle factors and is especially fast with smaller convolution < 17000 - needs power of 2 values

---

foxFFTHeartofRust
Wrapper for RustFFT - substantially faster with large FFT compared with Scientificgo or foxFFT as FFT size drops cgo boundary slows this down so it is slower than foxFFT

---

foxLog
Main logger

---

foxNormalizer
Various Normalize and peak gain calculations - NB I am now calculating peak gain from FFT within the partitioned convolver and applying the inverse as a multiplier to the signal as this is computationally cheaper

---

foxPEQ
Formulae for building parametric EQ and loudness compensation filters. Allows the filter coefficients to have IRR run vs a signal or be to be converted to an impulse

---

foxResampler
Experimental resampler this was to be used to match pre built impulse files to incoming audio rate - I now use SoX with the highest quality settings and import the call directly into a buffer

---

foxSignalGen
For generating test signals
<img width="368" height="364" alt="image" src="https://github.com/user-attachments/assets/f85678eb-87d4-4f4e-8ff0-c2741724d290" />

