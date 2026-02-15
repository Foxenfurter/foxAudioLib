#!/bin/bash
echo "Docker Environment:"
echo "  GOOS: $GOOS"
echo "  GOARCH: $GOARCH"
echo "  Actual OS: $(uname -s)"
echo "  Actual Arch: $(uname -m)"
echo "  Go version: $(go version)"
echo "---"

cd foxFFTHeartofRust
go test -v