@echo off
echo Copying fixed Rust libraries...

mkdir libs\windows_amd64 2>nul
mkdir libs\linux_amd64 2>nul
mkdir libs\linux_arm64 2>nul
mkdir libs\darwin_amd64 2>nul
mkdir libs\darwin_arm64 2>nul

echo Copying Windows library...
copy C:\Users\jonat\rust\rustfft-wrapper\target\x86_64-pc-windows-gnu\release\librustfft.a libs\windows_amd64\ >nul

echo Copying Linux AMD64 library...
copy C:\Users\jonat\rust\rustfft-wrapper\target\x86_64-unknown-linux-gnu\release\librustfft.a libs\linux_amd64\ >nul

echo Copying Linux ARM64 library...
copy C:\Users\jonat\rust\rustfft-wrapper\target\aarch64-unknown-linux-gnu\release\librustfft.a libs\linux_arm64\ >nul

echo Copying macOS Intel library...
copy C:\Users\jonat\rust\rustfft-wrapper\target\x86_64-apple-darwin\release\librustfft.a libs\darwin_amd64\ >nul

echo Copying macOS ARM64 library...
copy C:\Users\jonat\rust\rustfft-wrapper\target\aarch64-apple-darwin\release\librustfft.a libs\darwin_arm64\ >nul

echo All libraries copied successfully!
echo.
dir libs /s
