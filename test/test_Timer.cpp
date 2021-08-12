/* This is a test file for class Timer */

#include <chrono>
#include <memory>
#include <iostream>

class Timer {
public:
    Timer(float &runTime);
    ~Timer();

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_, end_;
    float *runTime_;
};

Timer::Timer(float &runTime) : start_{std::chrono::high_resolution_clock::now()}, 
                                               end_{std::chrono::high_resolution_clock::now()},
                                               runTime_{&runTime} {
}

Timer::~Timer() {
    end_ = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration {end_ - start_};
    *runTime_ = duration.count() * 1000.0f;
}

void test_timer_cost(int nums, float &runTime) {
    float oneRunTime = 0;
    Timer timer = Timer(runTime);
    for (int i = 0; i < nums; ++i) {
        Timer oneTimer = Timer(oneRunTime);
    }
}

int main() {
    float runTime = 0;
    int nums = 100000;
    test_timer_cost(nums, runTime);
    std::cout << nums << " Timer construct and destruct takes " << runTime << " ms\n";
    return 0;
}