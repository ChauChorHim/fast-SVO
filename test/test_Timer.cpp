/* This is a test file for class Timer */

#include <chrono>
#include <memory>
#include <iostream>

class Timer {
public:
    Timer(std::shared_ptr<float> runTime);
    ~Timer();

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_, end_;
    std::shared_ptr<float> runTime_;
};

Timer::Timer(std::shared_ptr<float> runTime) : start_{std::chrono::high_resolution_clock::now()}, 
                                               end_{std::chrono::high_resolution_clock::now()},
                                               runTime_{runTime} {
}

Timer::~Timer() {
    end_ = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration {end_ - start_};
    *runTime_ = duration.count() * 1000.0f;
}

void test_timer_cost(std::shared_ptr<float> &runTime) {
    std::shared_ptr<float> oneRunTime(new float(0));
    Timer timer = Timer(runTime);
    for (int i = 0; i < 10000; ++i) {
        Timer oneTimer = Timer(oneRunTime);
    }
}

int main() {
    std::shared_ptr<float> runTime(new float(0));
    test_timer_cost(runTime);
    std::cout << "10000 Timer construct and destruct takes " << *runTime << " ms\n";
    return 0;
}