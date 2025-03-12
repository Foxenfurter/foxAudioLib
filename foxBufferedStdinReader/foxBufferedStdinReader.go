package foxbufferedstdinreader

import (
	"bytes"
	"errors"
	"os"
	"sync"
	"time"
)

type BufferedStdinReader struct {
	buf     *bytes.Buffer
	readErr error
	closed  bool
	mutex   sync.Mutex
}

func NewBufferedStdinReader() *BufferedStdinReader {
	b := &BufferedStdinReader{
		buf: bytes.NewBuffer(make([]byte, 0, 65536)),
	}
	go b.continuousRead()
	return b
}

func (b *BufferedStdinReader) continuousRead() {
	tmp := make([]byte, 4096) // Match pipe buffer sizes
	for {
		n, err := os.Stdin.Read(tmp)
		b.mutex.Lock()
		if err != nil {
			b.readErr = err
			b.mutex.Unlock()
			return
		}
		b.buf.Write(tmp[:n])
		b.mutex.Unlock()
	}
}

func (b *BufferedStdinReader) PeekHeader(format string) ([]byte, error) {
	// LMS requires header processing within 2 seconds
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		b.mutex.Lock()
		switch format {
		case "WAV":
			if b.buf.Len() >= 44 { // Minimum WAV header size
				header := make([]byte, 44)
				copy(header, b.buf.Bytes()[:44])
				b.mutex.Unlock()
				return header, nil
			}
		case "PCM":
			if b.buf.Len() >= 1024 { // Arbitrary PCM safety buffer
				header := make([]byte, 0) // PCM has no header
				b.mutex.Unlock()
				return header, nil
			}
		}
		b.mutex.Unlock()
		time.Sleep(50 * time.Millisecond)
	}
	return nil, errors.New("header timeout")
}
