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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("VWO: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuWaitandView("menu.Wait and View",false,true);
    pangolin::Var<bool> menuSeeCam("menu.SeeCam",true,true);
    pangolin::Var<bool> menuCapture("menu.Capture",false,false);
    pangolin::Var<bool> menuFinishAndSave("menu.FinishAndSave",false,false);

    // Define Camera Render Object (for view / scene browsing)
    // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
    // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
    //                观测目标位置：(0, 0, 0)
    //                观测的方位向量：(0.0,-1.0, 0.0)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    // 定义显示面板大小，orbslam中有左右两个面板，昨天显示一些按钮，右边显示图形
    // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
    // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
    // 最后一个参数（-1024.0f/768.0f）为显示长宽比
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("VWO: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;


    std::string window_filename = mpSystem->GetResultDir() + "window.avi";
    std::string capture_filename = mpSystem->GetResultDir() + "capture.avi";
    // 创建OpenCV VideoWriter
    cv::VideoWriter window_writer;
    cv::VideoWriter capture_writer;


    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //绘制当前相机
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();

        if(menuSeeCam)
        {
            cv::Mat im = mpFrameDrawer->DrawFrame();
            cv::imshow("VWO: Current Frame",im);
            cv::waitKey(mT);
        }
        else{
            cv::destroyAllWindows();
        }

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
            menuWaitandView = false;
        }

        /*
        实现视频的本地保存
        */
        if (menuCapture) 
        {
            cv::Mat im = mpFrameDrawer->DrawFrame();
            if (!capture_writer.isOpened()) {
                capture_writer.open(capture_filename, cv::VideoWriter::fourcc('H', '2', '6', '4'), 1e3/mT, cv::Size(im.cols, im.rows));
            }
            if (!window_writer.isOpened()) {
                window_writer.open(window_filename, cv::VideoWriter::fourcc('H', '2', '6', '4'), 1e3/mT, cv::Size(1024, 768));
            }


            // 为了读取Pangolin窗口内容，需要在OpenGL的上下文中执行
            glReadBuffer(GL_FRONT);

            // 读取窗口内容
            std::vector<uint8_t> image_data(3 * 1024 * 768);
            glReadPixels(0, 0, 1024, 768, GL_BGR, GL_UNSIGNED_BYTE, image_data.data());

            // 将图像数据转为OpenCV Mat
            cv::Mat window_frame(768, 1024, CV_8UC3);
            window_frame.data = image_data.data();
            cv::flip(window_frame,window_frame, 0); //坐标系是反着的，故采用flip0策略
            window_writer.write(window_frame);

            cv::Mat capture_frame(im.rows, im.cols, CV_8UC3);
            capture_frame.data = im.data;
            capture_writer.write(capture_frame);
        } 

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(menuWaitandView){
            mpSystem->SetWaitComand(true);           
        }
        else{
            mpSystem->SetWaitComand(false);     
        }
        
        if(menuFinishAndSave){
            mpSystem->SetUserComand();
            SetFinish();
        }
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
