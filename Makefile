CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra -pthread
LDFLAGS  := -lgpiod

SRC := main.cpp util.cpp PID.cpp Encoder.cpp Motoron.cpp Motor.cpp
BIN := main.out

all: main

main: main.cpp Encoder.cpp Motor.cpp Motoron.cpp PID.cpp util.cpp
    $(CXX) -o main main.cpp Encoder.cpp Motor.cpp Motoron.cpp PID.cpp util.cpp -lpthread -lgpiod

# Test build
test: tests/encoder_test.cpp Encoder.cpp
    $(CXX) -o encoder_test tests/encoder_test.cpp Encoder.cpp -lpthread -lgpiod
    @echo "Run ./encoder_test to test encoder"

clean:
    rm -f main encoder_test *.o