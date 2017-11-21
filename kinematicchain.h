#ifndef KINEMATICCHAIN_H
#define KINEMATICCHAIN_H
#include "frame.h"

namespace knt {
class KinematicChain {
public:
private:
    Frame* root_ = nullptr;
    std::vector<Frame*> all_frames_;
    std::vector<Eigen::MatrixXd> Twf_;
};
}

#endif // KINEMATICCHAIN_H
