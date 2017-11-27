#include "viewer.h"

Viewer& Viewer::GetInstance()
{
    static Viewer instance;
    return instance;
}

Viewer::Viewer() {}
Viewer::~Viewer() {}

void Viewer::AddDrawFunction(const std::function<void(void)>& draw_funtion)
{
    draw_functions_.push_back(draw_funtion);
}

void Viewer::Run()
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
        static int count = 0;
        Tracer::TraceCounter("gl", count);
        ++count;
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        for (auto& draw : draw_functions_) {
            draw();
        }

        // Swap framesC and Process Events
        pangolin::FinishFrame();
    }
}
