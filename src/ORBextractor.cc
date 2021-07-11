#include "ORBextractor.h"

namespace fast_SVO 
{
ORBextractor::ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST) : 
    nfeatures_(nfeatures), scaleFactor_(scaleFactor), nlevels_(nlevels), iniThFAST_(iniThFAST), minThFAST_(minThFAST) {

}
}