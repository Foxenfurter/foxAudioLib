FROM golang:1.25

WORKDIR /app

# Copy all Go files and the libs directory
COPY *.go .
COPY libs/ ./libs/

# Run the tests
RUN go test -v