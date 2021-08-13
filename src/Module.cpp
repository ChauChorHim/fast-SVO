#include "Module.hpp"

namespace fast_SVO
{
    Module::Module() 
    : moduleTimer_{Timer()} {}
    Module::Module(const std::string &msg) 
    : moduleTimer_{Timer(msg)} {}

} // namespace fast_SVO
