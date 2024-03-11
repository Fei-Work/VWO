#include "WheelEncoder.h"

namespace ORB_SLAM2{

namespace WHEEL{

PulseCount::PulseCount(){}

PulseCount::PulseCount(double _time, double _WheelLeft, double _WheelRight):
    time(_time), 
    WheelLeft(_WheelLeft), WheelRight(_WheelRight)
{}

Preintegrated::Preintegrated(const Calibration _mCalib)
{
    dP.setZero();
    dR.setIdentity();
    C.setZero();
    Info.setZero();
    avgA.setZero();  // 平均加速度
    avgW.setZero();  // 平均角速度
    dT = 0;
    Nga = _mCalib.Cov;

    mCalib = _mCalib;
}

void Preintegrated::IntegrateNewMeasurement(const Eigen::Vector3f &velocity, const float &base_w, const float &dt)
{
    // Matrices to compute covariance
    // Step 1.构造协方差矩阵
    // 噪声矩阵的传递矩阵，这部分用于计算i到j-1历史噪声或者协方差
    Eigen::Matrix<float, 6, 6> A;
    A.setIdentity();
    // 噪声矩阵的传递矩阵，这部分用于计算j-1新的噪声或协方差，这两个矩阵里面的数都是当前时刻的，计算主要是为了下一时刻使用
    Eigen::Matrix<float, 6, 6> B;
    B.setZero();

    // 记录平均加速度和角速度
    // avgA = (dT * avgA + dR * acc * dt) / (dT + dt);
    // avgW = (dT * avgW + accW * dt) / (dT + dt);
    Eigen::Matrix3f rightJ;

    // Eigen::Vector3d axis(0, 0, 1.0);
    Eigen::Vector3f axis(0, 0, 1.0);
    Eigen::AngleAxisf rotation(-base_w * dt, axis);
    Eigen::Matrix3f RotationMatrix = rotation.toRotationMatrix();
    dP = dP + dR * velocity * dt;

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    // 根据η_ij = A * η_i,j-1 + B_j-1 * η_j-1中的Ａ矩阵和Ｂ矩阵对速度和位移进行更新
    Eigen::Matrix3f Wv;
    Wv.setZero();
    Wv(0,1) = -velocity(2);
    Wv(0,2) = velocity(1);
    Wv(1,2) = -velocity(0);
    Wv = Wv - Wv.transpose().eval();

    A.block<3, 3>(3, 0) = -dR * Wv * dt;
    B.block<3, 3>(3, 3) = dR * dt;

    // Update delta rotation
    // Step 2. 构造函数，会根据更新后的bias进行角度积分
    dR = dR * RotationMatrix;

    // Compute rotation parts of matrices A and B
    // 补充AB矩阵
    Eigen::Vector3f rotationVector = rotation.axis() * rotation.angle();

    rightJ = RightJacobianSO3(rotationVector);
    A.block<3, 3>(0, 0) = RotationMatrix.transpose();
    B.block<3, 3>(0, 0) = rightJ * dt;

    // 小量delta初始为0，更新后通常也为0，故省略了小量的更新
    // Update covariance
    // Step 3.更新协方差，frost经典预积分论文的第63个公式，推导了噪声（ηa, ηg）对dR dV dP 的影响
    C = A * C * A.transpose() + B * Nga * B.transpose();  // B矩阵为9*6矩阵 Nga 6*6对角矩阵，3个陀螺仪噪声的平方，3个加速度计噪声的平方
    // Total integrated time
    // 更新总时间
    dT += dt;
}

// No use now.
cv::Mat Preintegrated::GetRecentPose(const cv::Mat LastTwc)
{
    Eigen::Matrix<float,3,3> LastR = Converter::toMatrix3d(LastTwc.rowRange(0,3).colRange(0,3)).cast<float>();
    Eigen::Matrix<float,3,1> Lastt = Converter::toVector3d(LastTwc.rowRange(0,3).col(3)).cast<float>();
    
    // 通过读取 WheelBaseTransR 与 WheelBaseTransP （预积分值）得到新的位姿数据
    Eigen::Matrix<float,3,3> NewPoseR = dR * LastR;
    Eigen::Matrix<float,3,1> NewPoset = dR * Lastt + dP;

    // 转化为opencv格式的T
    cv::Mat cvMat = Converter::toCvSE3(NewPoseR.cast<double>(), NewPoset.cast<double>());

    return cvMat.clone();
}


WheelEncoderDatas::WheelEncoderDatas(const PulseCount mLastPulseCount, std::vector<PulseCount> vPc, WHEEL::Calibration* WheelCalib)
{
    int vPcSize = vPc.size();
    double Resolution = WheelCalib->eResolution;
    double LeftWheelDiameter = WheelCalib->eLeftWheelDiameter;
    double RightWheelDiameter = WheelCalib->eRightWheelDiameter;
    double WheelBase = WheelCalib->eWheelBase;

    double left_ditance, right_distance, left_velocity, right_velocity, base_w, base_theta;
    Eigen::Vector3d base_velocity;
    double pi = M_PI;

    PulseCount mLast;

    // 设定初始值
    base_velocity.setZero();
    WheelBaseTransR.setIdentity();
    WheelBaseTranst.setZero();

    int i_wheel;
    // 当累计的不为空时
    if(vPcSize >= 1){
        // 此处设置为0的主要原因为可能视频数据到位前wheel还没开启
        if(mLastPulseCount.time == 0){
            mLast = vPc[0];
            i_wheel = 1;
        }
        else{
            mLast = mLastPulseCount;
            i_wheel = 0;
        }
        
        while(i_wheel < vPcSize)
        {
            // 预处理
            during_time = (vPc[i_wheel].time - mLast.time);
            left_ditance = (vPc[i_wheel].WheelLeft - mLast.WheelLeft)/Resolution * pi * LeftWheelDiameter;
            right_distance = (vPc[i_wheel].WheelRight - mLast.WheelRight)/Resolution * pi * RightWheelDiameter;

            distance += (left_ditance + right_distance)/2;

            left_velocity = left_ditance / during_time;
            right_velocity = right_distance / during_time;

            base_velocity(2) = (left_velocity + right_velocity)/2;
            base_w = (left_velocity - right_velocity)/WheelBase;

            // 进行预积分运算
            WheelBaseTranst += -WheelBaseTransR * base_velocity * during_time;

            Eigen::Vector3d axis(0, 1.0, 0);
            base_theta = -base_w * during_time;
            Eigen::AngleAxis rotation(base_theta, axis);
            WheelBaseTransR = WheelBaseTransR * rotation.toRotationMatrix();
                       
            mLast = vPc[i_wheel];
            i_wheel++;
        }
    }
    else{
        during_time = 0;
        distance = 0;
    }
}

cv::Mat WheelEncoderDatas::GetNewPose(const cv::Mat LastTwc)
{
    Eigen::Matrix<double,3,3> LastR = Converter::toMatrix3d(LastTwc.rowRange(0,3).colRange(0,3));
    Eigen::Matrix<double,3,1> Lastt = Converter::toVector3d(LastTwc.rowRange(0,3).col(3));
    
    // 通过读取 WheelBaseTransR 与 WheelBaseTransP （预积分值）得到新的位姿数据
    Eigen::Matrix<double,3,3> NewPoseR = WheelBaseTransR * LastR;
    Eigen::Matrix<double,3,1> NewPoset = WheelBaseTransR * Lastt + WheelBaseTranst;

    // 转化为opencv格式的T
    cv::Mat cvMat = Converter::toCvSE3(NewPoseR, NewPoset);

    return cvMat.clone();

}

void WheelEncoderDatas::clear()
{
    
}

Calibration::Calibration(const Calibration &Calib)
{
    eResolution = Calib.eResolution;
    eLeftWheelDiameter = Calib.eLeftWheelDiameter;
    eRightWheelDiameter = Calib.eRightWheelDiameter;
    eWheelBase = Calib.eWheelBase;

    mTbc = Calib.mTbc;
    mTcb = Calib.mTcb;

    Cov = Calib.Cov;
}

void Calibration::Set(const Sophus::SE3f &sophTbc, const float &_eResolution, const float &_eLeftWheelDiameter, const float &_eRightWheelDiameter, const float & _eWheelBase)
{
    eResolution = _eResolution;
    eLeftWheelDiameter = _eLeftWheelDiameter;
    eRightWheelDiameter = _eRightWheelDiameter;
    eWheelBase = _eWheelBase;

    mTbc = sophTbc;
    mTcb = mTbc.inverse();

    // 噪声协方差
    Cov.diagonal() << 0.001, 0.001, 0.001, 0.006, 0.006, 100;
}


Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v)
{
    return RightJacobianSO3(v[0],v[1],v[2]);
}

Eigen::Matrix3f RightJacobianSO3(const float x, const float y, const float z)
{
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);

    Eigen::Matrix3f W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
        return Eigen::Matrix3f::Identity();
    else
        return Eigen::Matrix3f::Identity()*sin(d)/(d) + (1-sin(d)/d)*(W*W+Eigen::Matrix3f::Identity()) + (1-cos(d))/d * W;
}

}
}