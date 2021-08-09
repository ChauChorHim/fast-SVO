#ifndef TIMER
#define TIMER

#include <chrono>
#include <memory>

namespace fast_SVO
{

// Timer constructor needs std::shared_ptr<float> runTime to store the run time.
// After the Timer object destruct, runTime will be modified.
class Timer {
public:
    Timer(std::shared_ptr<float> runTime);
    ~Timer();

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_, end_;
    std::shared_ptr<float> runTime_;
};

} // namespace fast_SVO


#endif