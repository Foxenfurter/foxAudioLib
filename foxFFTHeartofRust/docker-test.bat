echo off
echo ========================================
echo  Testing Go package in Docker (Linux)
echo ========================================
echo.

echo Building Docker image...
docker build -t foxfft-test .

if %errorlevel% neq 0 (
    echo.
    echo ERROR: Docker build failed!
    pause
    exit /b 1
)

echo.
echo Running tests in Docker container...
docker run --rm foxfft-test

echo.
echo ========================================
echo  Docker test completed!
echo ========================================
pause