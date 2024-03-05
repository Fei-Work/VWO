#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Eigen/Core"

#include "WheelEncoder.h"
#include "Converter.h"

namespace ORB_SLAM2
{

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R);
Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v);
Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z);
Eigen::Matrix3d Shat(const Eigen::Vector3d &s);

class EdgeInertial : public g2o::BaseMultiEdge<6, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeInertial(WHEEL::Preintegrated *pInt);

    virtual bool read(std::istream &is) { return false; }
    virtual bool write(std::ostream &os) const { return false; }

    void computeError();
    virtual void linearizeOplus();

    // // 关于pose1与2 的旋转平移速度，以及之间的偏置的信息矩阵
    // Eigen::Matrix<double, 24, 24> GetHessian()
    // {
    //     linearizeOplus();
    //     Eigen::Matrix<double, 9, 24> J;
    //     J.block<9, 6>(0, 0) = _jacobianOplus[0];
    //     J.block<9, 3>(0, 6) = _jacobianOplus[1];
    //     J.block<9, 3>(0, 9) = _jacobianOplus[2];
    //     J.block<9, 3>(0, 12) = _jacobianOplus[3];
    //     J.block<9, 6>(0, 15) = _jacobianOplus[4];
    //     J.block<9, 3>(0, 21) = _jacobianOplus[5];
    //     return J.transpose() * information() * J;
    // }

    // // 没用
    // Eigen::Matrix<double, 18, 18> GetHessianNoPose1()
    // {
    //     linearizeOplus();
    //     Eigen::Matrix<double, 9, 18> J;
    //     J.block<9, 3>(0, 0) = _jacobianOplus[1];
    //     J.block<9, 3>(0, 3) = _jacobianOplus[2];
    //     J.block<9, 3>(0, 6) = _jacobianOplus[3];
    //     J.block<9, 6>(0, 9) = _jacobianOplus[4];
    //     J.block<9, 3>(0, 15) = _jacobianOplus[5];
    //     return J.transpose() * information() * J;
    // }

    // 关于pose2 的旋转平移信息矩阵
    Eigen::Matrix<double, 6, 6> GetHessian2()
    {
        linearizeOplus();
        Eigen::Matrix<double, 6, 6> J;
        J.block<6, 6>(0, 0) = _jacobianOplus[1];
        return J.transpose() * information() * J;
    }


    WHEEL::Preintegrated *mpInt;  // 预积分
    const double dt;  // 预积分时间
};


}// end of ORBSLAM2
#endif