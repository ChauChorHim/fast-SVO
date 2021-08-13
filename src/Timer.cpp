#include "Timer.hpp"
#include <iostream>

namespace fast_SVO {
Timer::Timer() : runTime_{0}, initTime_{std::chrono::high_resolution_clock::now()}, msg_("") {}
Timer::Timer(const std::string &msg) : runTime_{0}, initTime_{std::chrono::high_resolution_clock::now()}, msg_(msg) {}
void Timer::showTiming(const std::string &msg) {
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration {endTime - initTime_};
    std::string unit;
    if (duration.count() < 0.000001) {
        runTime_ = duration.count() * 1000000000;
        unit = "ns";
    } else if (duration.count()< 0.001) {
        runTime_ = duration.count() * 1000000;
        unit = "us";
    } else if (duration.count() < 1) {
        runTime_ = duration.count() * 1000;
        unit = "ms";
    } else if (duration.count() > 60) {
        runTime_ = duration.count() / 60;
        unit = "m";
    } else if (duration.count() > 86400) {
        runTime_ = duration.count() / 360;
        unit = "h";
    } else {
        runTime_ = duration.count();
        unit = "s";
    }
    std::cout << msg << " spend time: " << runTime_ << " " << unit << std::endl;
}
Timer::~Timer() {
    if (msg_ != "") {
        showTiming(msg_);     
    }
    
}

/* --------------------------------------------------- */

LoopTimer::LoopTimer(const std::string &msg) : Timer(), 
                         startTime_{std::chrono::high_resolution_clock::now()}, 
                         pauseTime_{std::chrono::high_resolution_clock::now()}, 
                         isPause_{false}, 
                         msg_(msg) {}

LoopTimer::~LoopTimer() {
    //msg_ = "------------------------------------------\ntotal " + msg_;
    showTiming(msg_);
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