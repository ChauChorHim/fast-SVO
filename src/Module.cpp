#include "Module.hpp"

namespace fast_SVO
{
    Module::Module() : moduleRunTime_{new float(0)}, moduleTimer_{Timer(moduleRunTime_)} {
    }

    Module::~Module() {

    }

    
} // namespace fast_SVO
