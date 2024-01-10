// Package allows logging in a way that is constistent with othe Squeeze Clients
// squeezeDSP/foxLog/logger.go
package foxLog

import (
	"fmt"
	"log"
	"os"
	"sync"
	"time"
)

// Logger represents the logger instance.
type Logger struct {
	mu          sync.Mutex
	logFile     *os.File
	debugStatus bool
	clientId    string
}

// LogType represents the type of log entry.
type LogType int

const (
	Info LogType = iota
	Debug
	Error
	FatalError
)

// NewLogger creates a new Logger instance.
func NewLogger(logFilePath string, newClientId string, debugStatus bool) (*Logger, error) {
	logFile, err := os.OpenFile(logFilePath, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
	if err != nil {
		return nil, err
	}

	if newClientId == "" {
		newClientId = "xx:xx:xx:xx:xx:xx"
	}

	return &Logger{
		logFile:     logFile,
		clientId:    newClientId,
		debugStatus: debugStatus,
	}, nil
}

// Log logs a message with the specified log type and description.
func (l *Logger) Log(logType LogType, description string) {
	l.mu.Lock()
	defer l.mu.Unlock()

	logEntry := fmt.Sprintf("%s %s [%s] %s\n", time.Now().Format("2006-01-02 15:04:05.999999"), l.clientId, logTypeToString(logType), description)

	if logType == Debug && !l.debugStatus {
		// Skip logging debug messages if debugStatus is off.
		return
	}

	if _, err := l.logFile.WriteString(logEntry); err != nil {
		log.Printf("Error writing to log file: %v", err)
	}
}

// Close closes the logger, ensuring that the log file is properly closed.
func (l *Logger) Close() {
	l.mu.Lock()
	defer l.mu.Unlock()

	if l.logFile != nil {
		if err := l.logFile.Close(); err != nil {
			log.Printf("Error closing log file: %v", err)
		}
	}
}

// logTypeToString converts LogType to a string representation.
func logTypeToString(logType LogType) string {
	switch logType {
	case Info:
		return "Info"
	case Debug:
		return "Debug"
	case Error:
		return "Error"
	case FatalError:
		return "FatalError"
	default:
		return "Unknown"
	}
}
