#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

namespace fast_SVO
{

class ORBextractor {
public:
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

private:
    int nfeatures_;
    double scaleFactor_;
    int nlevels_;
    int iniThFAST_;
    int minThFAST_;
};
}

#endif//ORBEXTRACTOR_H