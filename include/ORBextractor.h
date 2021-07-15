#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

namespace fast_SVO
{

class ORBextractor {
public:
    ORBextractor(int features, float scaleFactor, int levels,
                 int iniThFAST, int minThFAST);

private:
    int features_;
    double scaleFactor_;
    int levels_;
    int iniThFAST_;
    int minThFAST_;
};
}

#endif//ORBEXTRACTOR_H