#ifndef VIEWER_H
#define VIEWER_H
#include "tracer.h"
#include <functional>
#include <pangolin/pangolin.h>
#include <vector>

class Viewer {
public:
    static Viewer& GetInstance();
    Viewer();
    ~Viewer();

    void AddDrawFunction(const std::function<void(void)>& draw_funtion);

    void Run();

private:
    double height_ = 600;
    double width_ = 800;
    double f_ = 400;
    double z_near_ = 0.2;
    double z_far_ = 10000;

    // model look at
    double ex_ = 100, ey_ = 500, ez_ = 200;
    double lx_ = 100, ly_ = 0, lz_ = 200;
    pangolin::AxisDirection dir_ = pangolin::AxisZ;
    std::vector<std::function<void(void)>> draw_functions_;
};

#endif // VIEWER_H
