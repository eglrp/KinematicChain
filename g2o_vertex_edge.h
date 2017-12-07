#ifndef G2O_VERTEX_EDGE_H
#define G2O_VERTEX_EDGE_H
#include "frame.h"
#include <Eigen/Dense>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <memory>

namespace knt {

class Frame6DoFVertex : public g2o::BaseVertex<6, Eigen::Matrix4d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Frame6DoFVertex(Frame6DoF* frame)
        : g2o::BaseVertex<6, Eigen::Matrix4d>()
        , frame_(frame)
    {
    }

    virtual void setToOriginImpl()
    {
        _estimate = frame_->setToOriginImpl();
    }

    virtual void oplusImpl(const double* update)
    {
        _estimate = frame_->oplusImpl(update);
    }

    virtual bool read(std::istream& in) {}
    virtual bool write(std::ostream& out) const {}

private:
    Frame6DoF* frame_;
};

class RevoluteVertex : public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RevoluteVertex(RevoluteJoint* joint)
        : g2o::BaseVertex<1, double>()
        , joint_(joint)
    {
    }

    virtual void setToOriginImpl()
    {
        _estimate = joint_->setToOriginImpl();
    }

    virtual void oplusImpl(const double* update)
    {
        _estimate = joint_->oplusImpl(update[0]);
    }

    virtual bool read(std::istream& in) {}
    virtual bool write(std::ostream& out) const {}

private:
    RevoluteJoint* joint_;
};

class PrismaticVertex : public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PrismaticVertex(PrismaticJoint* joint)
        : g2o::BaseVertex<1, double>()
        , joint_(joint)
    {
    }

    virtual void setToOriginImpl()
    {
        _estimate = joint_->setToOriginImpl();
    }

    virtual void oplusImpl(const double* update)
    {
        _estimate = joint_->oplusImpl(update[0]);
    }

    virtual bool read(std::istream& in) {}
    virtual bool write(std::ostream& out) const {}

private:
    PrismaticJoint* joint_;
};

class RevoluteEdge6 : public g2o::BaseUnaryEdge<6, Eigen::Matrix4d, RevoluteVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RevoluteEdge6(Frame* frame)
        : frame_(frame)
    {
    }

private:
    Frame* frame_;
};

class RevoluteEdge3 : public g2o::BaseUnaryEdge<3, Eigen::VectorXd, RevoluteVertex> {
};
}
#endif // G2O_VERTEX_EDGE_H
