@echo off
echo Testing foxFFTHeartofRust on Linux via Docker...
echo.

docker run --rm -v "%CD%":/workspace -w /workspace golang:1.25 bash -c "
  echo 'Testing on Linux/' \$(go env GOARCH) 'with Go' \$(go version)
  echo '----------------------------------------'
  cd foxFFTHeartofRust && go test -v
"

echo.
echo Docker test completed!
pause