#include "frame.h"
#include <Eigen/Dense>
#include <iostream>
using namespace std;

int main()
{
    Eigen::MatrixXd A;
    A = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX()).toRotationMatrix();
    std::cout << knt::CheckSO3(A) << std::endl;

    return 0;
}
