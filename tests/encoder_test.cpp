#include "../Encoder.h"
#include <cstdio>
#include <thread>
#include <atomic>
#include <csignal>

static std::atomic<bool> running{true};

int main()
{
    std::signal(SIGINT, [](int){ running = false; });

    // Adjust these lines for your hardware
    Encoder enc("/dev/gpiochip0", 5, 6, 5);

    while (running.load()) {
        std::printf("Count: %d | Illegal: %u\n", enc.count(), enc.illegal());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}