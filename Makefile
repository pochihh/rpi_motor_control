# Makefile for encoder test

CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall
LDFLAGS  := -lgpiod -lpthread

SRC  := enc_test_gpiod.cpp
BIN  := enc_test_gpiod.out

all: $(BIN)

$(BIN): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(BIN)