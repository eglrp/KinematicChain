#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#define USE_ANALYTIC 1

class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl()
    {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double* update)
    {
        _estimate += Eigen::Map<const Eigen::Vector3d>(update);
    }

    virtual bool read(std::istream& in) {}
    virtual bool write(std::ostream& out) const {}
};

class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x)
        : BaseUnaryEdge()
        , _x(x)
        , _x_2(x * x)
    {
    }

    virtual void computeError()
    {
        const CurveFittingVertex* vertex = static_cast<const CurveFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc = vertex->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0) * _x_2 + abc(1) * _x + abc(2));
    }

#if USE_ANALYTIC
    virtual void linearizeOplus()
    {
        if (level() == 1) {
            _jacobianOplusXi = Eigen::Matrix<double, 1, 3>::Zero();
            return;
        }

        CurveFittingVertex* vertex = static_cast<CurveFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc = vertex->estimate();
        _jacobianOplusXi = Eigen::Matrix<double, 1, 3>();
        double e = std::exp(abc(0) * _x_2 + abc(1) * _x + abc(2));
        _jacobianOplusXi << -_x_2 * e, -_x * e, -e;
        return;
    }
#endif

    virtual bool read(std::istream& in)
    {
    }
    virtual bool write(std::ostream& out) const {}

private:
    double _x;
    double _x_2;
};

int main()
{
    double a = 1.0, b = 2.0, c = 1.0;
    int N = 100;
    double w_sigma = 1.0;
    cv::RNG rng;
    double abc[3] = { 0, 0, 0 };

    std::vector<double> x_data, y_data;

    std::cout << "generating data: " << std::endl;

    for (int i = 0; i < N; ++i) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(std::exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
        std::cout << x_data[i] << " " << y_data[i] << std::endl;
    }

    using Block = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
    auto linearSolver = std::make_unique<g2o::LinearSolverDense<Block::PoseMatrixType>>();
    auto solver_ptr = std::make_unique<Block>(std::move(linearSolver));
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(3, 3, 3));
    v->setId(0);
    optimizer.addVertex(v);

    for (int i = 0; i < N; ++i) {
        CurveFittingEdge* edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(y_data[i]);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
        optimizer.addEdge(edge);
    }

    std::cout << "start optimization" << std::endl;
    auto t0 = std::chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    auto t1 = std::chrono::steady_clock::now();
    std::cout << "solve time cost = " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << " (us)" << std::endl;

    Eigen::Vector3d abc_estimate = v->estimate();
    std::cout << "estimate model: " << abc_estimate.transpose() << std::endl;
    return 0;
}
