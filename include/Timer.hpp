#ifndef TIMER
#define TIMER

#include <ctime>
#include <chrono>

namespace fast_SVO
{

class Timer {
private:
    float runTime_;
    std::chrono::time_point<std::chrono::high_resolution_clock> initTime_;

public:
    Timer();
    virtual ~Timer();
    std::chrono::time_point<std::chrono::high_resolution_clock> getInitTime() { return initTime_; }
};

/* ----------------------------------------------------------------------------- */

class LoopTimer : public Timer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
    std::chrono::time_point<std::chrono::high_resolution_clock> pauseTime_;
    bool isPause_;
public:
    LoopTimer();
    ~LoopTimer();
    bool isPause() { return isPause_ ? true : false; }
    void start();
    void pause();
    void stop();
};

} // namespace fast_SVO


#endif