TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += eigen3

unix: PKGCONFIG += opencv

# g2o
LIBS += -lg2o_cli\
-lg2o_core\
-lg2o_csparse_extension\
-lg2o_ext_freeglut_minimal\
-lg2o_incremental\
-lg2o_interactive\
-lg2o_interface\
-lg2o_opengl_helper\
-lg2o_parser\
-lg2o_simulator\
-lg2o_solver_cholmod\
-lg2o_solver_csparse\
-lg2o_solver_dense\
-lg2o_solver_eigen\
-lg2o_solver_pcg\
-lg2o_solver_slam2d_linear\
-lg2o_solver_structure_only\
-lg2o_stuff\
-lg2o_types_data\
-lg2o_types_icp\
-lg2o_types_sba\
-lg2o_types_sclam2d\
-lg2o_types_sim3\
-lg2o_types_slam2d_addons\
-lg2o_types_slam2d\
-lg2o_types_slam3d_addons\
-lg2o_types_slam3d\
-lg2o_viewer
