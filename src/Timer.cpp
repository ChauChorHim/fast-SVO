#include "Timer.hpp"
#include <iostream>

namespace fast_SVO {
void LoopTimer::showTiming(std::chrono::time_point<std::chrono::high_resolution_clock> *time, const std::string &msg) {
    endTime_ = std::chrono::high_resolution_clock::now();
    duration_  = endTime_ - *time;
    std::string unit;
    if (duration_.count() < 0.000001) {
        runTime_ = duration_.count() * 1000000000;
        unit = "ns";
    } else if (duration_.count()< 0.001) {
        runTime_ = duration_.count() * 1000000;
        unit = "us";
    } else if (duration_.count() < 1) {
        runTime_ = duration_.count() * 1000;
        unit = "ms";
    } else if (duration_.count() > 60) {
        runTime_ = duration_.count() / 60;
        unit = "m";
    } else if (duration_.count() > 86400) {
        runTime_ = duration_.count() / 360;
        unit = "h";
    } else {
        runTime_ = duration_.count();
        unit = "s";
    }
    std::cout << msg << " spend time: " << runTime_ << " " << unit << std::endl;
}

LoopTimer::LoopTimer(const std::string &msg) :  
                     startTime_{std::chrono::high_resolution_clock::now()}, 
                     pauseTime_{std::chrono::high_resolution_clock::now()}, 
                     endTime_{std::chrono::high_resolution_clock::now()}, 
                     isPause_{false}, 
                     runTime_{0},
                     msg_{msg} {}

LoopTimer::~LoopTimer() {
    //msg_ = "------------------------------------------\ntotal " + msg_;
    showTiming(&startTime_, msg_);
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