@echo off
echo Testing foxFFTHeartofRust on Linux via Docker...
echo.

docker run --rm -v "%CD%":/workspace -w /workspace -e GOOS=linux -e GOARCH=amd64 golang:1.25 bash -c "cd foxFFTHeartofRust && GOOS=linux GOARCH=amd64 go test -v"

echo.
echo Linux Docker test completed!