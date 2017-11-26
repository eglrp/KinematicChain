#ifndef VIEWER_H
#define VIEWER_H
#include <functional>
#include <pangolin/pangolin.h>
#include <vector>

class Viewer {
public:
    static Viewer& GetInstance()
    {
        static Viewer instance;
        return instance;
    }

    Viewer();
    ~Viewer() {}

    void Run()
    {
        // Create window
        pangolin::CreateWindowAndBind("Display", width_, height_);
        glEnable(GL_DEPTH_TEST);

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(width_, height_, f_, f_, width_ * 0.5, height_ * 0.5, z_near_, z_far_),
            pangolin::ModelViewLookAt(ex_, ey_, ez_, lx_, ly_, lz_, dir_));

        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam);
        pangolin::View& d_cam = pangolin::CreateDisplay()
                                    .SetBounds(0.0, 1.0, 0.0, 1.0, -width_ / height_)
                                    .SetHandler(&handler);

        while (!pangolin::ShouldQuit()) {
            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);

            // Render OpenGL Cube
            pangolin::glDrawColouredCube();

            for (auto& draw : draw_functions_) {
                draw();
            }

            // Swap framesC and Process Events
            pangolin::FinishFrame();
        }
    }

    double height_ = 600;
    double width_ = 800;
    double f_ = 400;
    double z_near_ = 0.2;
    double z_far_ = 100;

    // model look at
    double ex_ = -2, ey_ = 2, ez_ = -2;
    double lx_ = 0, ly_ = 0, lz_ = 0;
    pangolin::AxisDirection dir_ = pangolin::AxisY;
    std::vector<std::function<void(void)>> draw_functions_;
};

#endif // VIEWER_H
