#include "Opt_type.h"



namespace ORB_SLAM2
{

/** 
 * @brief 局部地图中imu的局部地图优化（此时已经初始化完毕不需要再优化重力方向与尺度）
 * @param pInt 预积分相关内容
 */
EdgeInertial::EdgeInertial(WHEEL::Preintegrated *pInt): mpInt(pInt), dt(pInt->dT)
{
    // 准备工作，把预积分类里面的值先取出来，包含信息的是两帧之间n多个imu信息预积分来的
    // 2元边
    resize(2);

    // 2. 读取协方差矩阵的前6*6部分的逆矩阵，该部分表示的是预积分测量噪声的协方差矩阵
    Matrix6d Info = pInt->C.block<6,6>(0,0).cast<double>().inverse();
    // 3. 强制让其成为对角矩阵
    Info = (Info+Info.transpose())/2;
    // 4. 让特征值很小的时候置为0，再重新计算信息矩阵（暂不知这么操作的目的是什么，先搞清楚操作流程吧）
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6> > es(Info);
    Eigen::Matrix<double,6,1> eigs = es.eigenvalues();
    for(int i=0;i<6;i++)
        if(eigs[i]<1e-12)
            eigs[i]=0;
    // asDiagonal 生成对角矩阵
    Info = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
    setInformation(Info);
}


/** 
 * @brief 计算误差
 */
void EdgeInertial::computeError()
{
    const g2o::VertexSE3Expmap* VP1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);           //位姿Ti
    const g2o::VertexSE3Expmap* VP2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);           //位姿Tj

    const Eigen::Matrix3d dR = mpInt->dR;
    const Eigen::Vector3d dP = mpInt->dP;

    Eigen::Matrix3d R1 = VP1->estimate().rotation().toRotationMatrix();
    Eigen::Matrix3d R2 = VP2->estimate().rotation().toRotationMatrix();

    const Eigen::Vector3d er = LogSO3(dR.transpose() * R1.transpose() * R2);
    const Eigen::Vector3d ep = R1.transpose()*(VP2->estimate().translation() - VP1->estimate().translation()) - dP;

    _error << er, ep;
}


// 计算雅克比矩阵
void EdgeInertial::linearizeOplus()
{
    const g2o::VertexSE3Expmap* VP1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);           //位姿Ti
    const g2o::VertexSE3Expmap* VP2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);           //位姿Tj


    const Eigen::Matrix3d R1 = VP1->estimate().rotation().toRotationMatrix();  // Ri
    const Eigen::Matrix3d R2 = VP2->estimate().rotation().toRotationMatrix();  // Rj
    const Eigen::Vector3d P1 = VP1->estimate().translation();  // Pi
    const Eigen::Vector3d P2 = VP2->estimate().translation();  // Pj

    const Eigen::Matrix3d dR = mpInt->dR;
    const Eigen::Matrix3d eR = dR.transpose() * R1.transpose() * R2;        // r△Rij
    const Eigen::Vector3d er = LogSO3(eR);                      // r△φij
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);  // Jr^-1(log(△Rij))

    // _jacobianOplus个数等于边的个数，里面的大小等于观测值维度（也就是残差）× 每个节点待优化值的维度
    // Jacobians wrt Pose 1
    // _jacobianOplus[0] 9*6矩阵 总体来说就是三个残差分别对pose1的旋转与平移（p）求导
    _jacobianOplus[0].setZero();
    // rotation
    // (0,0)起点的3*3块表示旋转残差对pose1的旋转求导
    _jacobianOplus[0].block<3,3>(0,0) = -invJr*R2.transpose()*R1; // OK
    // (3,0)起点的3*3块表示位置残差对pose1的旋转求导
    _jacobianOplus[0].block<3,3>(3,0) = Shat(R1.transpose()*(P2 - P1)); // OK
    // translation
    // (0,3)起点的3*3块表示旋转残差对pose1的位置求导
    // (3,3)起点的3*3块表示位置残差对pose1的位置求导
    _jacobianOplus[0].block<3,3>(3,3) = -Eigen::Matrix3d::Identity(); // OK

    // Jacobians wrt Pose 2
    // _jacobianOplus[1] 6*6矩阵 总体来说就是三个残差分别对pose2的旋转与平移（p）求导
    _jacobianOplus[1].setZero();
    // (0,0)起点的3*3块表示旋转残差对pose2的旋转求导
    _jacobianOplus[1].block<3,3>(0,0) = -invJr; // OK
    // (3,0)起点的3*3块表示位置残差对pose2的旋转求导
    // translation
    // (0,3)起点的3*3块表示旋转残差对pose2的位置求导
    // (3,3)起点的3*3块表示位置残差对pose2的位置求导
    _jacobianOplus[1].block<3,3>(3,3) = R1.transpose()*R2; // OK
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R)
{
    const double tr = R(0,0)+R(1,1)+R(2,2);
    Eigen::Vector3d w;
    w << (R(2,1)-R(1,2))/2, (R(0,2)-R(2,0))/2, (R(1,0)-R(0,1))/2;
    const double costheta = (tr-1.0)*0.5f;
    if(costheta>1 || costheta<-1)
        return w;
    const double theta = acos(costheta);
    const double s = sin(theta);
    if(fabs(s)<1e-5)
        return w;
    else
        return theta*w/s;
}

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v)
{
    return InverseRightJacobianSO3(v[0],v[1],v[2]);
}

Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
        return Eigen::Matrix3d::Identity();
    else
        return Eigen::Matrix3d::Identity() + W/2 + W*W*(1.0/d2 - (1.0+cos(d))/(2.0*d*sin(d)));
}

Eigen::Matrix3d Shat(const Eigen::Vector3d &s)
{
    Eigen::Matrix3d W;
    W.setZero();
    W(0,1) = -s(2);
    W(0,2) = s(1);
    W(1,2) = -s(0);
    W = W - W.transpose().eval();

    return W;
}

}