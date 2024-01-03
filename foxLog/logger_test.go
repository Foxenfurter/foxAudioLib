package foxLog_test

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/Foxenfurter/pkg/foxLog"
)

func TestLogger(t *testing.T) {
	/*
		dir := "C:"
		subdirs := []string{"temp", "gotesting", "output"}
		filename := "logtest.txt"
	*/
	//logFilePath := filepath.Join(append([]string{dir}, subdirs...), filename)
	// bugger this let's make it simpler
	logFileDir := "C:/temp/gotesting/output"
	//logFileDir := "C:/temp"
	// Create directories if they do not exist
	err := os.MkdirAll((logFileDir), os.ModePerm)
	if err != nil {
		fmt.Println("Error creating directories:", err)
		return
	}

	debugStatus := true

	// Create a logger instance for testing
	logFilePath := filepath.Join(logFileDir, "logtest.txt")

	// Clean up (delete test log file)
	os.Remove(logFilePath)

	logger, err := foxLog.NewLogger(logFilePath, "", debugStatus)
	if err != nil {
		t.Fatalf("Error creating logger: %v", err)
	}
	defer logger.Close()

	// Log some entries
	logger.Log(foxLog.Info, "Test normal log message.")
	logger.Log(foxLog.Debug, "Test debug log message.")
	logger.Log(foxLog.Error, "Test error log message.")
	logger.Log(foxLog.FatalError, "Test fatal error log message.")

	// Check if log file is created and contains expected content
	content, err := os.ReadFile(logFilePath)
	if err != nil {
		t.Fatalf("Error reading log file: %v", err)
	}

	expectedLines := []string{
		"[Info] Test normal log message.",
		"[Debug] Test debug log message.",
		"[Error] Test error log message.",
		"[FatalError] Test fatal error log message.",
	}

	// split the lines
	actualLines := strings.Split(string(content), "\n")
	// check the line ending is present in the actual lines
	for i, expectedLine := range expectedLines {
		if !strings.HasSuffix(actualLines[i], expectedLine) {
			t.Errorf("Unexpected log file content. Expected line ending with:\n%s\nGot:\n%s", expectedLine, actualLines[i])
		}
	}

}

func TestMain(m *testing.M) {
	// Run tests
	fmt.Println("Running Logging Test:")
	exitCode := m.Run()

	// Exit with the test result code
	os.Exit(exitCode)
}
