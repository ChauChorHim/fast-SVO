#include "ORBextractor.h"

namespace fast_SVO 
{
ORBextractor::ORBextractor(int features, float scaleFactor, int levels, int iniThFAST, int minThFAST) : 
    features_(features), scaleFactor_(scaleFactor), levels_(levels), iniThFAST_(iniThFAST), minThFAST_(minThFAST) {

}
}