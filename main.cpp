#include "frame.h"
#include "tracer.h"
#include "viewer.h"
#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <vector>
using namespace std;

int main()
{
    std::vector<double> a = { 0, 0, 10, -10, 0, 96 };
    std::vector<double> alpha = { M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2 };
    std::vector<double> d = { 0, 0, 371, 0, 280, 0 };
    std::vector<double> theta = { 0, M_PI, M_PI, -M_PI / 2, M_PI, M_PI / 2 };
    std::vector<double> maxlimits = { 5 * M_PI / 9, M_PI / 2, 115 * M_PI / 180, M_PI / 2, 115 * M_PI / 180, 85 * M_PI / 180 };
    std::vector<double> minlimits = { -5 * M_PI / 9, -M_PI / 2, -115 * M_PI / 180, -40 * M_PI / 180, -115 * M_PI / 180, -85 * M_PI / 180 };
    std::vector<knt::Frame*> robot;

    for (int i = 0, n = a.size(); i < n; ++i) {
        knt::Frame* ptr = new knt::RevoluteJoint(a[i], alpha[i], d[i], theta[i], minlimits[i], maxlimits[i]);
        robot.push_back(ptr);
    }

    for (int i = 0, n = robot.size() - 1; i < n; ++i) {
        robot[i]->AddNextFrame(robot[i + 1]);
        robot[i + 1]->SetParentFrame(robot[i]);
    }
    Eigen::MatrixXd Tww = Eigen::MatrixXd::Identity(4, 4);
    robot[0]->Update(Tww, false);

    for (int i = 0, n = robot.size(); i < n; ++i) {
        std::cout << robot[i]->Twf_ << std::endl;
    }

    Viewer::GetInstance().AddDrawFunction(std::bind(&knt::Frame::DebugDraw, robot[0]));

    std::thread viewer_thread(&Viewer::Run, &Viewer::GetInstance());

    Eigen::VectorXd update(1);
    update(0) = 0.00001;
    while (1) {
        for (int i = 0, n = robot.size(); i < n; ++i) {
            robot[i]->Oplus(update);
        }
        robot[0]->Update(Tww, false);
    }
    return 0;
}
