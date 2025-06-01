package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"path/filepath"
	"strings"
)

func main() {
	if len(os.Args) < 2 {
		fmt.Println("Usage: arc3a-extractor <input.arc3a> [output_directory]")
		return
	}

	inputPath := os.Args[1]
	outputDir := "."
	if len(os.Args) > 2 {
		outputDir = os.Args[2]
	}

	if err := os.MkdirAll(outputDir, 0755); err != nil {
		log.Fatalf("Error creating output directory: %v", err)
	}

	if err := extractWAVsFromARC3a(inputPath, outputDir); err != nil {
		log.Fatalf("Extraction failed: %v", err)
	}

	fmt.Printf("Successfully extracted files from %s to %s\n", inputPath, outputDir)
}

func extractWAVsFromARC3a(inputPath, outputDir string) error {
	data, err := ioutil.ReadFile(inputPath)
	if err != nil {
		return fmt.Errorf("error reading file: %w", err)
	}

	// Verify ARC3a signature
	if len(data) < 16 || !bytes.Equal(data[0:8], []byte{'I', 'K', 'M', 'P', 'A', 'K', 0x03, 0x00}) {
		return fmt.Errorf("invalid ARC3a header signature")
	}

	log.Printf("Scanning %d bytes for WAV files with metadata prefixes...", len(data))

	// Scan for WAV files with metadata prefixes
	found := 0
	offset := 0

	for offset < len(data)-12 {
		// Look for "RIFF" header
		if bytes.Equal(data[offset:offset+4], []byte("RIFF")) {
			// Read RIFF chunk size
			if offset+8 >= len(data) {
				break
			}
			chunkSize := int(binary.LittleEndian.Uint32(data[offset+4 : offset+8]))
			totalSize := chunkSize + 8

			// Verify we have enough data
			if offset+totalSize > len(data) {
				log.Printf("Incomplete WAV at offset %d (needs %d bytes, %d available)",
					offset, totalSize, len(data)-offset)
				offset += 4
				continue
			}

			// Verify "WAVE" signature
			if offset+12 >= len(data) || !bytes.Equal(data[offset+8:offset+12], []byte("WAVE")) {
				log.Printf("Invalid WAVE header at offset %d", offset)
				offset += 4
				continue
			}

			// Extract metadata prefix (32 bytes before RIFF header)
			metaStart := offset - 32
			if metaStart < 0 {
				metaStart = 0
			}
			metaData := data[metaStart:offset]

			// Find last null terminator before RIFF
			nullPos := bytes.LastIndexByte(metaData, 0)
			if nullPos == -1 {
				nullPos = 0
			}

			// Extract printable string from null terminator to RIFF header
			prefix := string(bytes.Trim(metaData[nullPos:], "\x00"))
			prefix = strings.TrimSpace(prefix)
			prefix = strings.ReplaceAll(prefix, "/", "_")
			prefix = strings.ReplaceAll(prefix, "\\", "_")
			prefix = strings.ReplaceAll(prefix, ":", "_")

			if prefix == "" {
				prefix = fmt.Sprintf("segment_%d", found)
			}

			// Create filename with sequence number and prefix
			filename := fmt.Sprintf("%03d_%s.wav", found, prefix)
			outPath := filepath.Join(outputDir, filename)

			// Extract the complete WAV file
			wavData := data[offset : offset+totalSize]
			if err := ioutil.WriteFile(outPath, wavData, 0644); err != nil {
				log.Printf("Error writing %s: %v", outPath, err)
			} else {
				log.Printf("Extracted %s (%d bytes) - %s", outPath, len(wavData), prefix)
				found++
			}

			// Skip forward
			offset += totalSize
		} else {
			offset++
		}
	}

	if found == 0 {
		return fmt.Errorf("no WAV files found")
	}

	log.Printf("Extracted %d WAV files", found)
	return nil
}
