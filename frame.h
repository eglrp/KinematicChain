#ifndef FRAME_H
#define FRAME_H
#include <Eigen/Dense>
#include <exception>
#include <vector>
namespace knt {

enum {
    NUM_DOF_FRAME = 0,
    NUM_DOF_REVOLUTE = 1,
    NUM_DOF_PRISMATIC = 1,
    NUM_DOF_FRAME6DOF = 6
};

bool CheckSO3(const Eigen::MatrixXd& R)
{
    static double eps = 1e-3;
    if (R.cols() != 3 || R.rows() != 3)
        return false;

    double det = R.determinant();
    if (std::abs(1.0 - det) > eps) {
        return false;
    }

    Eigen::MatrixXd RRt_eye = R * R.transpose() - Eigen::MatrixXd::Identity(3, 3);
    double norm = RRt_eye.norm();
    if (norm > eps) {
        return false;
    }

    return true;
}

bool CheckSE3(const Eigen::MatrixXd& T)
{
    static double eps = 1e-3;
    if (T.cols() != 4 || T.rows() != 4)
        return false;

    if (CheckSO3(T.block<3, 3>(0, 0))) {
        return false;
    }

    Eigen::MatrixXd buttom = T.block<1, 4>(3, 0);
    buttom(3) = buttom(3) - 1;
    if (buttom.norm() > eps) {
        return false;
    }

    return true;
}

class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Frame()
    {
        Tf_1_f_ = Eigen::MatrixXd::Identity(4, 4);
    }
    Frame(const Eigen::MatrixXd& Tf_1_f)
    {
        if (CheckSE3(Tf_1_f))
            throw std::runtime_error("input matrix is not SE(3)");
        Tf_1_f_ = Tf_1_f;
    }
    virtual ~Frame() {}
    void AddNextFrame(Frame* next)
    {
        next_frames_.push_back(next);
    }

    virtual int DoFSize() const { return NUM_DOF_FRAME; }

    virtual Eigen::MatrixXd Jij(const Eigen::VectorXd& tvec_wt) const
    {
        return Eigen::MatrixXd();
    }

    virtual Eigen::MatrixXd Jij(const Eigen::MatrixXd& Twt) const
    {
        return Eigen::MatrixXd();
    }
    virtual void Oplus(const Eigen::VectorXd& update)
    {
    }

protected:
    Eigen::MatrixXd Twf_;
    Eigen::MatrixXd Tf_1_f_;
    std::vector<Frame*> next_frames_;
};

class Frame6DoF : public Frame {
public:
    Frame6DoF()
    {
        Tf_1_f_ = Eigen::MatrixXd::Identity(4, 4);
    }
    Frame6DoF(const Eigen::MatrixXd& Tf_1_f)
    {
        if (CheckSE3(Tf_1_f))
            throw std::runtime_error("input matrix is not SE(3)");
        Tf_1_f_ = Tf_1_f;
    }
    ~Frame6DoF() {}

    int DoFSize() const override
    {
        return NUM_DOF_FRAME6DOF;
    }

    Eigen::MatrixXd Jij(const Eigen::VectorXd& tvec_wt) const override
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, NUM_DOF_FRAME6DOF);
        J.block<3, 1>(0, 0) = Twf_.block<3, 1>(0, 0);
        J.block<3, 1>(0, 1) = Twf_.block<3, 1>(0, 1);
        J.block<3, 1>(0, 2) = Twf_.block<3, 1>(0, 2);
        Eigen::MatrixXd r = tvec_wt - Twf_.block<3, 1>(0, 3);
        J.block<3, 1>(0, 3) = Twf_.block<3, 1>(0, 0).cross(r);
        J.block<3, 1>(0, 4) = Twf_.block<3, 1>(0, 1).cross(r);
        J.block<3, 1>(0, 5) = Twf_.block<3, 1>(0, 2).cross(r);

        return J;
    }

    Eigen::MatrixXd Jij(const Eigen::MatrixXd& Twt) const override
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, NUM_DOF_FRAME6DOF);
        J.block<3, 1>(0, 0) = Twf_.block<3, 1>(0, 0);
        J.block<3, 1>(0, 1) = Twf_.block<3, 1>(0, 1);
        J.block<3, 1>(0, 2) = Twf_.block<3, 1>(0, 2);
        Eigen::MatrixXd r = Twt.block<3, 1>(0, 3) - Twf_.block<3, 1>(0, 3);
        J.block<3, 1>(0, 3) = Twf_.block<3, 1>(0, 0).cross(r);
        J.block<3, 1>(0, 4) = Twf_.block<3, 1>(0, 1).cross(r);
        J.block<3, 1>(0, 5) = Twf_.block<3, 1>(0, 2).cross(r);

        J.block<3, 1>(3, 3) = Twf_.block<3, 1>(0, 0);
        J.block<3, 1>(3, 4) = Twf_.block<3, 1>(0, 1);
        J.block<3, 1>(3, 5) = Twf_.block<3, 1>(0, 2);

        return J;
    }

    void Oplus(const Eigen::VectorXd& update) override
    {
        Eigen::VectorXd rvec = update.tail(3);
        double th = rvec.norm();
        rvec /= th;
        Eigen::MatrixXd Tffp = Eigen::MatrixXd::Identity(4, 4);
        Tffp.block<3, 3>(0, 0) = Eigen::AngleAxisd(th, rvec).toRotationMatrix();
        Tffp.block<3, 1>(0, 3) = update.head(3);
        Tf_1_f_ = Tf_1_f_.eval() * Tffp;
    }

private:
};

class RevoluteJoint : public Frame {
public:
    RevoluteJoint();
    ~RevoluteJoint();

    int DoFSize() const override
    {
        return NUM_DOF_REVOLUTE;
    }

    Eigen::MatrixXd Jij(const Eigen::VectorXd& tvec_wt) const override
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, NUM_DOF_REVOLUTE);
        Eigen::MatrixXd r = tvec_wt - Twf_.block<3, 1>(0, 3);
        J = Twf_.block<3, 1>(0, 2).cross(r);
        return J;
    }

    Eigen::MatrixXd Jij(const Eigen::MatrixXd& Twt) const override
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, NUM_DOF_REVOLUTE);
        Eigen::MatrixXd r = Twt - Twf_.block<3, 1>(0, 3);
        J.block<3, 1>(0, 0) = Twf_.block<3, 1>(0, 2).cross(r);
        J.block<3, 1>(3, 0) = Twf_.block<3, 1>(0, 2);
        return J;
    }

    void Oplus(const Eigen::VectorXd& update) override;

private:
};

class PrismaticJoint : public Frame {
public:
    PrismaticJoint();
    ~PrismaticJoint();

    int DoFSize() const override
    {
        return NUM_DOF_PRISMATIC;
    }

    Eigen::MatrixXd Jij(const Eigen::VectorXd& tvec_wt) const override
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, NUM_DOF_PRISMATIC);
        J = Twf_.block<3, 1>(0, 2);
        return J;
    }

    Eigen::MatrixXd Jij(const Eigen::MatrixXd& Twt) const override
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, NUM_DOF_PRISMATIC);
        J.block<3, 1>(0, 0) = Twf_.block<3, 1>(0, 2);
        return J;
    }

    void Oplus(const Eigen::VectorXd& update) override;

private:
};
}

#endif // FRAME_H
