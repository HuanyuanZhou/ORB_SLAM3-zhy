/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../include/ImuTypes.h"

using namespace std;

ros::Publisher path_publish;
ros::Publisher pose_publish;
ros::Publisher loop_publish;
bool LastKeyframeDecision;

class ImuGrabber
{
public:
    ImuGrabber(){};

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
};

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}

class ImageGrabber
{
public:
    ImageGrabber(){};

    queue<sensor_msgs::ImageConstPtr> imgBuf;
    std::mutex mBufMutex;

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
};

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutex.lock();
    imgBuf.push(img_msg);
    mBufMutex.unlock();
    return;
}

class InfoSync
{
public:
    InfoSync(ORB_SLAM3::System* pSLAM, ImuGrabber* pImuGb, ImageGrabber* pImgGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mpImgGb(pImgGb), mbClahe(bClahe){}

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber* mpImuGb;
    ImageGrabber* mpImgGb;

    ORB_SLAM3::IMU::Point fGetImu(const sensor_msgs::ImuConstPtr& imu_msg);
    cv::Mat fGetImg(const sensor_msgs::ImageConstPtr& img_msg);
    void fGetPathPose(nav_msgs::Path& result_path, std::vector<std::pair<cv::Mat, double>>& pose_vector);
    void fSync();

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    int receive_frame_counter = 0;
};

void InfoSync::fGetPathPose(nav_msgs::Path& result_path, std::vector<std::pair<cv::Mat, double>>& pose_vector)
{
    Eigen::Matrix4d temp_matrix, temp_matrix_inverse;
    Eigen::Matrix4d trans_form = Eigen::Matrix4d::Identity();

    for(int i=0; i<pose_vector.size(); i++)
    {
        geometry_msgs::PoseStamped this_pose;

        for(int row_i = 0; row_i < 4; row_i ++)
        {
            for(int col_i = 0; col_i < 4; col_i ++)
            {
                temp_matrix(row_i, col_i) = pose_vector[i].first.at<float>(row_i, col_i);
            }
        }

        this_pose.header.stamp = pose_vector[i].second;

        temp_matrix_inverse = trans_form * temp_matrix.inverse(); // Twc
        Eigen::Quaterniond rotation_q(temp_matrix_inverse.block<3, 3>(0, 0));
        this_pose.pose.position.x = temp_matrix_inverse(0, 3);
        this_pose.pose.position.y = temp_matrix_inverse(1, 3);
        this_pose.pose.position.z = temp_matrix_inverse(2, 3);
        this_pose.pose.orientation.x = rotation_q.x();
        this_pose.pose.orientation.y = rotation_q.y();
        this_pose.pose.orientation.z = rotation_q.z();
        this_pose.pose.orientation.w = rotation_q.w();
        result_path.poses.push_back(this_pose);
    }
}

void InfoSync::fSync()
{
    while(1)
    {
        cv::Mat im, track_result;
        double tIm = 0;

        // Imu buffer 或 Img buffer为空则等待
        if(mpImgGb->imgBuf.empty() || mpImuGb->imuBuf.empty())
            continue;

        tIm = mpImgGb->imgBuf.front()->header.stamp.toSec();
        
        // Img时间戳大于Imu最后时间则等待
        if(tIm > mpImuGb->imuBuf.back()->header.stamp.toSec())
            continue;
        
        // Img时间戳小于Imu最前时间
        if(tIm < mpImuGb->imuBuf.front()->header.stamp.toSec())
        {
            mpImgGb->mBufMutex.lock();
            mpImgGb->imgBuf.pop();
            mpImgGb->mBufMutex.unlock();
            continue;
        }

        //取出Img
        {
            mpImgGb->mBufMutex.lock();
            im = fGetImg(mpImgGb->imgBuf.front());
            mpImgGb->imgBuf.pop();
            mpImgGb->mBufMutex.unlock();
        }

        //取出所有小于tIm的Imu数据
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            mpImuGb->mBufMutex.lock();
            while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
            {
                vImuMeas.push_back(fGetImu(mpImuGb->imuBuf.front()));
                mpImuGb->imuBuf.pop();
            }
            mpImuGb->mBufMutex.unlock();
        }

        //图像前景后景处理
        if(mbClahe)
            mClahe->apply(im, im);

        //运行跟踪
        track_result = mpSLAM->TrackMonocular(im, tIm, vImuMeas);

        //如果需要重置则清空Buffer
        if(mpSLAM->GetIfReset() || mpSLAM->GetIfResetActiveMap())
        {
            ROS_INFO("Reset Signal");
            {
                mpImuGb->mBufMutex.lock();
                mpImgGb->mBufMutex.lock();
                
                mpImgGb->imgBuf =queue<sensor_msgs::ImageConstPtr>();
                mpImuGb->imuBuf = queue<sensor_msgs::ImuConstPtr>();

                mpImuGb->mBufMutex.unlock();
                mpImgGb->mBufMutex.unlock();
            }
            continue;
        }

        while(mpSLAM->LocalMappingStopped())
            void();

        //Publisher
        std::vector<std::pair<cv::Mat, double>> poses_vector;

        mpSLAM->GetAllPoses(poses_vector);

        if(poses_vector.size() == 0) continue;

        //the fixed path publisher
        nav_msgs::Path result_path;
        result_path.header.stamp = ros::Time().fromSec(tIm);
        result_path.header.frame_id = "world";
        fGetPathPose(result_path, poses_vector);
        path_publish.publish(result_path);

        //get the last key frame compare to the current frame
        double reference_stamp;
        reference_stamp = mpSLAM->GetRelativePose();

        int reference_index = 0;
        double time_diff = 1e9;

        for (int i = 0; i < poses_vector.size(); i++)
        {
            double this_time_diff = fabs(poses_vector[i].second - reference_stamp);
            if (this_time_diff < time_diff)
            {
                reference_index = i;
                time_diff = this_time_diff;
            }
        }

        LastKeyframeDecision = mpSLAM->GetKeyframeDecision();

        //current frame pose Twc publisher
        nav_msgs::Odometry this_odometry;
        this_odometry.header.stamp = ros::Time().fromSec(tIm);
        this_odometry.header.frame_id = "world";
        Eigen::Matrix4d T_cw, T_wc;

        for (int row_i = 0; row_i < 4; row_i++)
        {
            for (int col_i = 0; col_i < 4; col_i++)
            {
                T_cw(row_i, col_i) = track_result.at<float>(row_i, col_i);
            }
        }

        T_wc = T_cw.inverse();
        Eigen::Quaterniond rotation_q(T_wc.block<3, 3>(0, 0));
        this_odometry.pose.pose.position.x = T_wc(0, 3);
        this_odometry.pose.pose.position.y = T_wc(1, 3);
        this_odometry.pose.pose.position.z = T_wc(2, 3);
        this_odometry.pose.pose.orientation.x = rotation_q.x();
        this_odometry.pose.pose.orientation.y = rotation_q.y();
        this_odometry.pose.pose.orientation.z = rotation_q.z();
        this_odometry.pose.pose.orientation.w = rotation_q.w();
        if (LastKeyframeDecision)
            this_odometry.pose.covariance[0] = 1;
        else
            this_odometry.pose.covariance[0] = 0;
        this_odometry.pose.covariance[1] = reference_index;
        pose_publish.publish(this_odometry);

        //get loop index
        sensor_msgs::PointCloud ros_loop_info;
        ros_loop_info.header.stamp = ros::Time().fromSec(tIm);
        ros_loop_info.header.frame_id = "this_is_loop_info";
        std::vector<std::pair<double, double>> loop_result;
        mpSLAM->GetLoopInfo(loop_result);
        sensor_msgs::ChannelFloat32 loop_channel;
        for (int i = 0; i < loop_result.size() && i < 35; i++)
        {
            int first_index = -1;
            int second_index = -1;

            for (int j = 0; j < poses_vector.size(); j++)
            {
                if (poses_vector[j].second == loop_result[i].first)
                    first_index = j;
                if (poses_vector[j].second == loop_result[i].second)
                    second_index = j;
            }

            if (first_index > 0 && second_index > 0)
            {
                printf("Publisher: the loop info %d <---> %d\n", first_index, second_index);
                loop_channel.values.push_back(loop_result[i].first);
                loop_channel.values.push_back(loop_result[i].second);
            }
            else
            {
                printf("Publisher: cannot find corresponding!\n");
            }
        }

        ros_loop_info.channels.push_back(loop_channel);
        loop_publish.publish(ros_loop_info);

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

ORB_SLAM3::IMU::Point InfoSync::fGetImu(const sensor_msgs::ImuConstPtr& imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    cv::Point3f acc(imu_msg->linear_acceleration.x,
                    imu_msg->linear_acceleration.y,
                    imu_msg->linear_acceleration.z);
    cv::Point3f gyr(imu_msg->angular_velocity.x,
                    imu_msg->angular_velocity.y,
                    imu_msg->angular_velocity.z);
    return ORB_SLAM3::IMU::Point(acc, gyr, t);
}

cv::Mat InfoSync::fGetImg(const sensor_msgs::ImageConstPtr& img_msg)
{
    receive_frame_counter++;

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    bool bEqual = false;
    
    if(argc < 3 || argc > 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
        ros::shutdown();
        return 1;
    }

    if(argc==4)
    {
        std::string sbEqual(argv[3]);
        if(sbEqual == "true")
          bEqual = true;
    }
    
    //ORB System
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true);

    ImuGrabber imugb;
    ImageGrabber imggb;
    InfoSync infogb(&SLAM, &imugb, &imggb, bEqual);

    // Maximum delay, 5 seconds
    ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img0 = n.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage, &imggb);

    path_publish = n.advertise<nav_msgs::Path>("/orb_slam/path", 1000);
    pose_publish = n.advertise<nav_msgs::Odometry>("/orb_slam/pose", 1000);
    loop_publish = n.advertise<sensor_msgs::PointCloud>("/orb_slam/loop", 1000);

    std::thread sync_thread(&InfoSync::fSync, &infogb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    //Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}