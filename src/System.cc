/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
// #include <boost/filesystem.hpp>
#include <filesystem> //此为C17的新特性，可引入（低于gcc9.1需要连接stdc++fs）
namespace fs = std::filesystem;

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, string mdir):
               mSensor(sensor), 
               mpViewer(static_cast<Viewer*>(NULL)), 
               mbReset(false),
               mbActivateLocalizationMode(false),
               mbDeactivateLocalizationMode(false),
               mbStopAndSave(false),
               mbWaitAndView(false)

{
    // Output welcome message
    // cout << endl <<
    // "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    // "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    // "This is free software, and you are welcome to redistribute it" << endl <<
    // "under certain conditions. See LICENSE.txt." << endl << endl;
    ResultDirName = "";
    dirName1st = "result";
    if(mdir != " "){
        dirName2nd = mdir;
    }
    else{
        if(strSettingsFile.length()>5)
        {
            size_t lastSlash = strSettingsFile.find_last_of("/");
            std::string fileName = strSettingsFile.substr(lastSlash + 1);
            size_t extensionPos = fileName.find(".yaml");
            if (extensionPos != std::string::npos) {
                fileName = fileName.substr(0, extensionPos);
            }
            dirName2nd = fileName;
        }
        else{
            cout<< "Error strSettingsFile name";
            exit(-1);
        }
    }

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==WHEEL_STEREO)
        cout << "WHEEL_STEREO" << endl;

    //Check settings file 
    //cv::FileStorage
    // ? 在cv::FileStorage中可以读取XML\YAML\JSON的文件，可以返回string/int/list/map等类型的数据
    // ? yaml 返回字典，可以直接使用float fx = fSettings["Camera.fx"]读取
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    resetTimes = 0;
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO or check your parameters." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    // 对每个输入的图像正式开始tracking
    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

// 增加Wheel_Encoder track 功能
cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const vector<WHEEL::PulseCount> &vWheelMeas, const double &timestamp)
{
    if(mSensor!=WHEEL_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to WHEEL_STEREO or check your parameters." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    // 获取wheel meas信息
    if(mSensor==WHEEL_STEREO){
        mpTracker->GrabWheelEncoder(vWheelMeas);
    }
    
    // 对每个输入的图像正式开始tracking
    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    // 检查是否即刻停止并保存结果
    while(CheckWaitComand()){
        usleep(100);
    }
    return Tcw;
}


cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    SaveBodyKeyFrameTrajectoryTUM();
    AddRestTimes();
    mbReset = true;
}

int System::GetResetTimes()
{
    unique_lock<mutex> lock(mMutexResetTimes);
    return resetTimes;
}
void System::AddRestTimes()
{
    unique_lock<mutex> lock(mMutexResetTimes);
    resetTimes = resetTimes + 1;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("VWO: Map Viewer");
}

void System::CreateResultDir()
{
    if(ResultDirName.length()!=0)
        return;
    // 判断结果文件夹是否存在
	// string dirName("result");
	fs::path url(dirName1st);
	if (!fs::exists(url)) {
		cout<< "result not exist" << "\n";
        // 创建单层目录
	    bool okey = fs::create_directory(dirName1st);
	    cout << "create_directory(" << dirName1st << "), result=" << okey << "\n";
	}
	else {
		cout << "directory'result' exist" << "\n";
	}

    //检查二级目录
    fs::path url2(dirName1st + "/"+ dirName2nd);
    if (!fs::exists(url2)) {
		cout << dirName1st + "/"+ dirName2nd << "not exist" << "\n";
        // 创建单层目录
	    bool okey = fs::create_directory(dirName1st + "/"+ dirName2nd);
	    cout << "create_directory(" << dirName1st + "/"+ dirName2nd << "), result=" << okey << "\n";
	}
	else {
		cout << "directory:" << dirName1st + "/"+ dirName2nd << " exist" << "\n";
	}

    ResultDirName = dirName1st + "/" + dirName2nd + "/" ;
}

string System::GetResultDir()
{
    return ResultDirName;
}

void System::SaveTrajectoryTUM()
{
    CreateResultDir();
    string filename = ResultDirName + "TrajectoryTUM" + to_string(resetTimes) + ".txt";
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM()
{
    CreateResultDir();
    string filename = ResultDirName + "KeyframeTrajectoryTUM" + to_string(resetTimes) + ".txt";
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveBodyKeyFrameTrajectoryTUM()
{
    CreateResultDir();
    string filename = ResultDirName + "KeyframeBodyTrajectoryTUM" + to_string(resetTimes) + ".txt";
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        Eigen::Matrix3d Re = pKF->GetWheelRotation().cast<double>();
        Eigen::Vector3d te = pKF->GetWheelPosition().cast<double>();
        cv::Mat R = Converter::toCvMat(Re);
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = Converter::toCvMat(te);
        
        f << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI()
{
    CreateResultDir();
    string filename = ResultDirName + "TrajectoryKITTI.txt";
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SetUserComand()
{
    unique_lock<mutex> lock(mMutexmbStopAndSave);
    mbStopAndSave = true;
}

bool System::CheckUserComand()
{
    unique_lock<mutex> lock(mMutexmbStopAndSave);
    return mbStopAndSave;
}

void System::SetWaitComand(bool tag)
{
    unique_lock<mutex> lock(mMutexmbStopAndSave);
    mbWaitAndView = tag;
}

bool System::CheckWaitComand()
{
    unique_lock<mutex> lock(mMutexmbWaitAndView);
    return mbWaitAndView;
}



int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
