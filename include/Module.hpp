/*-----------------------------

This is a interface base class for modules.
It should contains function for log infomation...

-----------------------------*/

#ifndef MODULE
#define MODULE

#include "Timer.hpp"

namespace fast_SVO
{
class Module {
public:
    Module();
    virtual ~Module();
    virtual void logInfo() = 0;

private:
    Timer timer;
};

    
} // namespace fast_SVO



#endif 