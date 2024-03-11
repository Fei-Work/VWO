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

    Sophus::SE3<float> Tcb;
    Sophus::SE3<float> Tbc;
};


}// end of ORBSLAM2
#endif