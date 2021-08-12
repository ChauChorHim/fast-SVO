#include <iostream>
#include <chrono>
#include <memory>

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

class Module {
public:
    Module() : moduleRunTime_{0}, moduleTimer_{Timer(moduleRunTime_)} { std::cout << "Module()\n"; }
    virtual ~Module() { std::cout << "~Module()\n"; }
    virtual void logInfo() = 0;
    float getRunTime() { return moduleRunTime_; }

private:
    Timer moduleTimer_;
    float moduleRunTime_;
};

class Derived : public Module {
public:
    Derived() : Module() { std::cout << "Derived()\n"; }
    ~Derived() { std::cout << "~Derived()\n Takes time: " << getRunTime() << "\n"; }
    void logInfo() {
        std::cout << "Derived: logInfo()\n";
    }
};

int main() {
    Derived derived = Derived();
    derived.logInfo();
    return 0;
}