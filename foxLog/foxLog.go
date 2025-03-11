// Package allows logging in a way that is consistent with other Squeeze Clients
// squeezeDSP/foxLog/logger.go
package foxLog

import (
	"fmt"
	"log"
	"os"
	"path/filepath"
	"sync"
	"time"
)

// Logger represents the logger instance.
type Logger struct {
	mu           sync.Mutex
	LogFile      *os.File
	DebugEnabled bool
	ClientId     string
}

// Constants representing log types
const (
	Info       = "Info"
	Debug      = "Debug"
	Error      = "Error"
	Warn       = "Warn"
	FatalError = "FatalError"
)

// NewLogger creates a new Logger instance.
func NewLogger(logFilePath string, newClientId string, debugEnabled bool) (*Logger, error) {
	// Clean and standardize the file path
	logFilePath = filepath.ToSlash(filepath.Clean(logFilePath))
	logFile, err := os.OpenFile(logFilePath, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
	if err != nil {
		return nil, err
	}

	if newClientId == "" {
		newClientId = "xx:xx:xx:xx:xx:xx"
	}

	return &Logger{
		LogFile:      logFile,
		ClientId:     newClientId,
		DebugEnabled: debugEnabled,
	}, nil
}

// Debug logs a debug message with the specified description.
func (l *Logger) Debug(description string) {
	if !l.DebugEnabled {
		return // Return immediately if Debug is not enabled
	}

	l.Log(Debug, description)
}

// Info logs an informational message with the specified description.
func (l *Logger) Info(description string) {
	l.Log(Info, description)
}

// Warn logs an warning message with the specified description.
func (l *Logger) Warn(description string) {
	l.Log(Warn, description)
}

// Error logs an error message with the specified description.
func (l *Logger) Error(description string) {
	l.Log(Error, description)
}

// FatalError logs a fatal error message with the specified description.
func (l *Logger) FatalError(description string) {
	l.Log(FatalError, description)
	l.Close() // Ensure log file is closed and flushed
	os.Exit(1)
}

// Log logs a message with the specified log type and description.
func (l *Logger) Log(logType, description string) {
	l.mu.Lock()
	defer l.mu.Unlock()

	logEntry := fmt.Sprintf("%s %s [%s] %s\n", time.Now().Format("2006-01-02 15:04:05.999999"), l.ClientId, logType, description)

	if logType == Debug && !l.DebugEnabled {
		// Skip logging debug messages if DebugEnabled is off.
		return
	}

	if _, err := l.LogFile.WriteString(logEntry); err != nil {
		log.Printf("Error writing to log file: %v", err)
	} else {
		if err := l.LogFile.Sync(); err != nil {
			log.Printf("Error syncing log file: %v", err)
		}
	}
}

// Close closes the logger, ensuring that the log file is properly closed.
func (l *Logger) Close() {
	l.mu.Lock()
	defer l.mu.Unlock()

	if l.LogFile != nil {
		if err := l.LogFile.Close(); err != nil {
			log.Printf("Error closing log file: %v", err)
		}
	}
}
