#include "Timer.hpp"

namespace fast_SVO {
    Timer::Timer() : start_{std::chrono::high_resolution_clock::now()}, 
                     end_{std::chrono::high_resolution_clock::now()},
                     duration_{end_ - start_} {
    }

    Timer::~Timer() {
        end_ = std::chrono::high_resolution_clock::now();
        duration_ = end_ - start_;
        float ms = duration_.count() * 1000.0f;
    }
}