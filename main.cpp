#include "frame.h"
#include <Eigen/Dense>
#include <iostream>
using namespace std;

int main()
{
    knt::Frame f;
    knt::Frame6DoF ff;
    knt::RevoluteJoint r(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    knt::PrismaticJoint p(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    return 0;
}
