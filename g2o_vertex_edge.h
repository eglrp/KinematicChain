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

    RevoluteJoint* GetRevoluteJoint() const
    {
        return joint_;
    }

private:
    RevoluteJoint* joint_;
};

//                                         output dimension, target dimension, corredpondence vertex
class RevoluteEdge3 : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, RevoluteVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RevoluteEdge3(Frame* frame, Frame* root)
        : frame_(frame)
        , root_(root)
    {
    }
    ~RevoluteEdge3() {}

    virtual void computeError()
    {
        root_->Update(Eigen::Matrix4d::Identity(), false);
    }

    virtual void linearizeOplus()
    {
        if (level() == 1) {
            return;
        }
        return;
    }

    virtual bool read(std::istream& in) {}
    virtual bool write(std::ostream& out) const {}

private:
    Frame* frame_;
    Frame* root_;
};
}
#endif // G2O_VERTEX_EDGE_H
