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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimestamps);
void LoadEncoder(const string &strFile, vector<double> &vWheelEncoderLeft, vector<double> &vWheelEncoderRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_wheel path_to_vocabulary path_to_sequence" << endl;
        return 1;
    }

    // load images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestampsCam;
   
    string img_strFile = string(argv[3])+"/sensor_data/stereo_stamp.csv";
    LoadImages(img_strFile, vstrImageLeft, vstrImageRight, vTimestampsCam);
    int nImages = vstrImageLeft.size();

    // load wheel_encoder
    vector<double> vWheelEncoderLeft;
    vector<double> vWheelEncoderRight;
    vector<double> vTimestampsEncoder;
    string encoder_strFile = string(argv[3])+"/sensor_data/encoder.csv";
    LoadEncoder(encoder_strFile, vWheelEncoderLeft, vWheelEncoderRight, vTimestampsEncoder);
    int nEncoder = vWheelEncoderLeft.size();

    // wheel_count用于统计之后读取的wheel位置
    int wheel_count = 0;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // 初始化各个线程，localmapping、loopClosing处于waiting姿态（分别为等待关键帧）
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::WHEEL_STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    cout << "WheelEncoder in the sequence: " << nEncoder << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight, left_BGR, right_BGR, left_GRAY, right_GRAY;
    vector<ORB_SLAM2::WHEEL::PulseCount> vWheelMeas;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        imLeft = cv::imread(string(argv[3]) + vstrImageLeft[ni],cv::IMREAD_GRAYSCALE);
        imRight = cv::imread(string(argv[3]) + vstrImageRight[ni],cv::IMREAD_GRAYSCALE);
        double tframe = vTimestampsCam[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageLeft[ni] << endl;
            return 1;
        }

        cv::cvtColor(imLeft, left_BGR, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(imRight, right_BGR, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(left_BGR, left_GRAY, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_BGR, right_GRAY, cv::COLOR_BGR2GRAY);

        //get wheel encoder during this time
        vWheelMeas.clear();

        while(vTimestampsEncoder[wheel_count] <= tframe){
            vWheelMeas.push_back(ORB_SLAM2::WHEEL::PulseCount(vTimestampsEncoder[wheel_count], vWheelEncoderLeft[wheel_count], vWheelEncoderRight[wheel_count]));
            wheel_count++;
        }
        
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Pass the image and wheel to the SLAM system
        SLAM.TrackStereo(left_GRAY, right_GRAY, vWheelMeas, tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame // 手动设置帧数，所以记得要对其时间单位
        double T=0;
        if(ni<nImages-1)
            T = vTimestampsCam[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestampsCam[ni-1];
        
        T = T/1e10; //适用于该数据集的时间对齐到s        
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
        
        //
        // 检查是否即刻停止并保存结果
        if(SLAM.CheckUserComand()){
            break;
        }
    } // end of for(main loop)

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM();
    SLAM.SaveTrajectoryTUM();

    return 0;
}

void LoadImages(const string &strFile,  vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first line
    string s0;
    getline(f,s0);
    // Read the timetamps and filenames 
    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string s_left;
            string s_right;
            ss >> t;
            vTimestamps.push_back(t);

            s_left = "/image/stereo_left/" + s + ".png";
            s_right = "/image/stereo_right/" + s + ".png";
            vstrImageLeft.push_back(s_left);
            vstrImageRight.push_back(s_right);
        }
    }
}


void LoadEncoder(const string &strFile, vector<double> &vWheelEncoderLeft, vector<double> &vWheelEncoderRight, vector<double> &vTimestamps){
    ifstream f;
    f.open(strFile.c_str());
    string line;
    // Read the timetamps and filenames 
    while (getline(f, line)) {
        istringstream iss(line);
        string token;
        vector<double> numbers;

        while (getline(iss, token, ',')) {
            // 将字符串转换为 double 并存储
            numbers.push_back(std::stod(token));
        }
        if(numbers.size() == 3)
        {
            vTimestamps.push_back(numbers[0]);
            vWheelEncoderLeft.push_back(numbers[1]);
            vWheelEncoderRight.push_back(numbers[2]);
        }
        else {
            std::cerr << "Error: Each line should contain three numbers." << std::endl;
        }
    }
}
