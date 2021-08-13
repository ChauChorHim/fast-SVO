#ifndef TIMER
#define TIMER

#include <ctime>
#include <chrono>
#include <string>

namespace fast_SVO
{

class Timer {
private:
    double runTime_;
    std::chrono::time_point<std::chrono::high_resolution_clock> initTime_;
    std::string msg_;

protected:
    void showTiming(const std::string &msg);

public:
    explicit Timer();
    explicit Timer(const std::string &msg);
    virtual ~Timer();
    std::chrono::time_point<std::chrono::high_resolution_clock> getInitTime() { return initTime_; }
};

/* ----------------------------------------------------------------------------- */

class LoopTimer : public Timer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
    std::chrono::time_point<std::chrono::high_resolution_clock> pauseTime_;
    bool isPause_;
    std::string msg_;

public:
    explicit LoopTimer(const std::string &msg);
    ~LoopTimer();
    bool isPause() { return isPause_ ? true : false; }
    void start();
    void pause();
};

} // namespace fast_SVO


#endif