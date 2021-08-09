#ifndef TIMER
#define TIMER

#include <chrono>

namespace fast_SVO
{
class Timer {
public:
    Timer();
    ~Timer();

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_, end_;
    std::chrono::duration<float> duration_;
};
    
} // namespace fast_SVO


#endif