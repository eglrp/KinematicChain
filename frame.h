#ifndef FRAME_H
#define FRAME_H
#include <Eigen/Dense>
#include <exception>
#include <vector>
namespace knt {

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

    virtual int DoFSize() const { return 0; }
    virtual Eigen::MatrixXd Jij(const Eigen::MatrixXd& Twt) const
    {
        return Eigen::MatrixXd();
    }
    virtual void Oplus(const Eigen::MatrixXd& update) const {}

private:
    Eigen::MatrixXd Tf_1_f_;
    std::vector<Frame*> next_frames_;
};

class Frame6DoF : public Frame {
public:
    Frame6DoF();
    Frame6DoF(const Eigen::MatrixXd& Tf_1_f);
    ~Frame6DoF();

    int DoFSize() const override;
    Eigen::MatrixXd Jij(const Eigen::MatrixXd& Twt) const override;
    void Oplus(const Eigen::MatrixXd& update) const override;

private:
};

class RevoluteJoint : public Frame {
public:
    RevoluteJoint();
    ~RevoluteJoint();

    int DoFSize() const override;
    Eigen::MatrixXd Jij(const Eigen::MatrixXd& Twt) const override;
    void Oplus(const Eigen::MatrixXd& update) const override;

private:
};

class PrismaticJoint : public Frame {
public:
    PrismaticJoint();
    ~PrismaticJoint();

    int DoFSize() const override;
    Eigen::MatrixXd Jij(const Eigen::MatrixXd& Twt) const override;
    void Oplus(const Eigen::MatrixXd& update) const override;

private:
};
}

#endif // FRAME_H
