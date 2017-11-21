#ifndef FRAME_H
#define FRAME_H
#include <Eigen/Dense>
#include <vector>
namespace knt {

class Frame {
public:
    Frame();
    Frame(const Eigen::MatrixXd& Tf_1_f);
    virtual ~Frame();
    void AddNextFrame(Frame* next);
    virtual int DoFSize() const;
    virtual Eigen::MatrixXd Jij(const Eigen::MatrixXd& Twt) const;
    virtual void Update(const Eigen::MatrixXd& update) const;

private:
    Eigen::MatrixXd Tf_1_f_;
    std::vector<Frame*> next_frames_;
};

class Frame6DoF : public Frame {
public:
private:
};

class RevoluteJoint : public Frame {
public:
private:
};

class PrismaticJoint : public Frame {
public:
private:
};
}

#endif // FRAME_H
