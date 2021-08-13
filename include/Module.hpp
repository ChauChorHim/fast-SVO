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
    explicit Module();
    explicit Module(const std::string &msg);
    virtual ~Module() {};
    virtual void logInfo() = 0;

private:
    Timer moduleTimer_;
};

    
} // namespace fast_SVO



#endif 