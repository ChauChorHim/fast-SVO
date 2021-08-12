#ifndef TIMER
#define TIMER

#include <chrono>

namespace fast_SVO
{

// Timer constructor needs std::shared_ptr<float> runTime to store the run time.
// After the Timer object destruct, runTime will be modified.
class Timer {
public:
    Timer(float &runTime);
    ~Timer();

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_, end_;
    float *runTime_;
};

} // namespace fast_SVO


#endif