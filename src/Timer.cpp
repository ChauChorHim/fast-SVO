#include "Timer.hpp"

namespace fast_SVO {
    Timer::Timer(std::shared_ptr<float> runTime) : start_{std::chrono::high_resolution_clock::now()}, 
                                                   end_{std::chrono::high_resolution_clock::now()},
                                                   runTime_{runTime} {
    }

    Timer::~Timer() {
        end_ = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration {end_ - start_};
        *runTime_ = duration.count() * 1000.0f;
    }
}