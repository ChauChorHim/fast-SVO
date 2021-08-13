#include "Timer.hpp"
#include <iostream>

namespace fast_SVO {
Timer::Timer() : runTime_{0}, initTime_{std::chrono::high_resolution_clock::now()} {
}

Timer::~Timer() {
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration {endTime - initTime_};
    runTime_ = duration.count() * 1000.0f;
}

/* --------------------------------------------------- */

LoopTimer::LoopTimer() : Timer(), 
                         startTime_{std::chrono::high_resolution_clock::now()}, 
                         pauseTime_{std::chrono::high_resolution_clock::now()}, 
                         isPause_{false}{ }

LoopTimer::~LoopTimer() {

}

void LoopTimer::stop() {
    if (isPause_)
        isPause_ = false;
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration {endTime - startTime_};
    auto runTime = duration.count() * 1000.0f;
    std::cout << "Currently this module costs time: " << runTime << " ms\n";
}

void LoopTimer::start() {
    if (isPause_) {
        isPause_ = false;
        auto now = std::chrono::high_resolution_clock::now();
        startTime_ += (now - pauseTime_); 
    }
}
void LoopTimer::pause() {
    if (isPause_)
        return;
    else {
        isPause_ = true;
        pauseTime_ = std::chrono::high_resolution_clock::now();
    }
} 
}