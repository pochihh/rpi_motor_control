CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra -pthread
LDFLAGS  := -lgpiod

SRC := main.cpp util.cpp PID.cpp Encoder.cpp Motoron.cpp Motor.cpp
BIN := main.out

all: $(BIN)

$(BIN): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(BIN)