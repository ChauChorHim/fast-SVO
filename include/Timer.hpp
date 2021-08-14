#ifndef TIMER
#define TIMER

#include <ctime>
#include <chrono>
#include <string>

namespace fast_SVO
{

class LoopTimer{
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
    std::chrono::time_point<std::chrono::high_resolution_clock> pauseTime_;
    std::chrono::time_point<std::chrono::high_resolution_clock> endTime_;
    std::chrono::duration<double> duration_;
    bool isPause_;
    double runTime_;
    std::string msg_;

public:
    explicit LoopTimer(const std::string &msg);
    ~LoopTimer();
    void showTiming(std::chrono::time_point<std::chrono::high_resolution_clock> *time0, 
                    std::chrono::time_point<std::chrono::high_resolution_clock> *time1,
                    const std::string &msg);
    bool isPause() { return isPause_ ? true : false; }
    void start();
    void pause();
    std::chrono::time_point<std::chrono::high_resolution_clock>* getStartTime();
    std::chrono::time_point<std::chrono::high_resolution_clock>* getPauseTime();
};

} // namespace fast_SVO


#endif