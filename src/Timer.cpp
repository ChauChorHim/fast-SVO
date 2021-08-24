#include "Timer.hpp"
#include <iostream>

namespace fast_SVO {
void LoopTimer::showTiming(std::chrono::time_point<std::chrono::high_resolution_clock> *time0, 
                           std::chrono::time_point<std::chrono::high_resolution_clock> *time1,
                           const std::string &msg) {
    duration_  = *time1 - *time0;
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
    std::cout << msg << " spend time: " << runTime_ << " " << unit << "\n\n";
}

LoopTimer::LoopTimer(const std::string &msg) :  
                     startTime_{std::chrono::high_resolution_clock::now()}, 
                     pauseTime_{std::chrono::high_resolution_clock::now()}, 
                     endTime_{std::chrono::high_resolution_clock::now()}, 
                     isPause_{false}, 
                     runTime_{0},
                     msg_{msg},
                     isShow_{false} {}

LoopTimer::~LoopTimer() {
    //msg_ = "------------------------------------------\ntotal " + msg_;
    if (isShow_) {
        endTime_ = std::chrono::high_resolution_clock::now();
        showTiming(&startTime_, &endTime_, msg_);
    }
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
void LoopTimer::reset() {
    isPause_ = true;
    startTime_ = std::chrono::high_resolution_clock::now();
    pauseTime_ = std::chrono::high_resolution_clock::now();
    isShow_ = true;
}
std::chrono::time_point<std::chrono::high_resolution_clock>* LoopTimer::getStartTime() {
    return &startTime_;
}

std::chrono::time_point<std::chrono::high_resolution_clock>* LoopTimer::getPauseTime() {
    return &pauseTime_;
}
}