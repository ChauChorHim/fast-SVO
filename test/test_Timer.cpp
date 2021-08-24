/* This is a test file for class Timer */

#include <chrono>
#include <memory>
#include <iostream>
#include "Timer.hpp"

void test_timer_cost(int pauseNum) {
    fast_SVO::LoopTimer timer = fast_SVO::LoopTimer("Loop Timer");
    for (int i = 0; i < pauseNum; ++i) {
        timer.start();
        timer.pause();
    }
}

int main() {
    int pauseNum = 100000;
    std::cout << "Loop Timer pause " << pauseNum << " times ---->\n";
    test_timer_cost(pauseNum);
    return 0;
}