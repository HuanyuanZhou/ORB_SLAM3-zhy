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
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    //统计加入图片的数量并存储时间戳
    int receive_counter = 0;
    std::vector<ros::Time> receive_time_stamp;
};


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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb, bEqual); // TODO

    // Maximum delay, 5 seconds
    ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img0 = n.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage, &igb);

    path_publish = n.advertise<nav_msgs::Path>("/orb_slam/path", 1000);
    pose_publish = n.advertise<nav_msgs::Odometry>("/orb_slam/pose", 1000);
    loop_publish = n.advertise<sensor_msgs::PointCloud>("/orb_slam/loop", 1000);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    //Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    receive_counter++;
    receive_time_stamp.push_back(img_msg->header.stamp);

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

void ImageGrabber::SyncWithImu()
{
    cout.precision(17);

    while(1)
    {
        cv::Mat im, track_result;
        double tIm = 0;
        //sensor_msgs::ImageConstPtr pIm;

        //如果没有接受到对应消息则等待不继续往下执行
        if(img0Buf.empty() || )
        if (!img0Buf.empty() && !mpImuGb->imuBuf.empty()) {
            tIm = img0Buf.front()->header.stamp.toSec();
            if (tIm > mpImuGb->imuBuf.back()->header.stamp.toSec())
            {
                continue;
            }

            {
                this->mBufMutex.lock();
                im = GetImage(img0Buf.front());
                img0Buf.pop();
                this->mBufMutex.unlock();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty()) {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm) {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                                    mpImuGb->imuBuf.front()->linear_acceleration.y,
                                    mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                                    mpImuGb->imuBuf.front()->angular_velocity.y,
                                    mpImuGb->imuBuf.front()->angular_velocity.z);
                    //cout << t << ' ' << acc.x << ' ' << acc.y << ' ' << acc.z << ' ' << gyr.x << ' ' << gyr.y << ' ' << gyr.z << endl;
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            if (mbClahe)
                mClahe->apply(im, im);

            track_result = mpSLAM->TrackMonocular(im, tIm, vImuMeas);

            //发送重置信号并对当有的vector进行清空
            if(mpSLAM->GetIfReset() || mpSLAM->GetIfResetActiveMap())
            {
                ROS_INFO("Reset");
                continue;
            }

            while (mpSLAM->LocalMappingStopped()) {
                void();
            }

            //Publisher
            std::vector<std::pair<cv::Mat, double>> result_vector;

            //获取得到的序列位姿，按照时间戳由小到大排序
            mpSLAM->GetAllPoses(result_vector);

            if(result_vector.size() == 0) continue;

            //获取局部路径
            nav_msgs::Path result_path;
            result_path.header.stamp = ros::Time().fromSec(tIm);
            result_path.header.frame_id = "world";
            Eigen::Matrix4d temp_matrix, temp_matrix_inverse;
            Eigen::Matrix4d trans_form = Eigen::Matrix4d::Identity();
            // trans_form << 0,0,1,0, -1,0,0,0, 0,-1,0,0, 0,0,0,1;
            for(int i = 0; i < result_vector.size(); i++)
            {
                geometry_msgs::PoseStamped this_pose;

                for (int j=receive_time_stamp.size()-1; j>=0; j--)
                {
                    if (fabs(receive_time_stamp[j].toSec() - result_vector[i].second) < 0.001)
                    {
                        this_pose.header.stamp = receive_time_stamp[j];
                        break;
                    }
                }

//            for (int j = 0; j < receive_time_stamp.size(); j++) {
//                if (fabs(receive_time_stamp[j].toSec() - result_vector[i].second) < 0.001) {
//                    this_pose.header.stamp = receive_time_stamp[j];
//                    break;
//                }
//            }

                for(int row_i = 0; row_i < 4; row_i ++)
                {
                    for(int col_i = 0; col_i < 4; col_i ++)
                    {
                        temp_matrix(row_i, col_i) = result_vector[i].first.at<float>(row_i, col_i);
                    }
                }

                temp_matrix_inverse = trans_form * temp_matrix.inverse();
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

            path_publish.publish(result_path);  // vector of Twc

            //寻找RefKeyFrame对应的index
            double reference_stamp;
            reference_stamp = mpSLAM->GetRelativePose(); //path KF timestamp

            int reference_index = 0;
            double time_diff = 1e9;

            for (int i = 0; i < result_vector.size(); i++)
            {
                double this_time_diff = fabs(result_vector[i].second - reference_stamp);
                if (this_time_diff < time_diff)
                {
                    reference_index = i;
                    time_diff = this_time_diff;
                }
            }
            if (time_diff < 0.01)
                printf("Publisher: the reference keyframe is %d, keyframe number %d.\n", reference_index, result_vector.size());
            else
                printf("Publisher: cannot find the reference keyframe! time difference %f, the stamp is %f, current is %f.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",
                       time_diff,
                       reference_stamp,
                       tIm);

            // 判断当前帧是否为KeyFrame
            LastKeyframeDecision = mpSLAM->GetKeyframeDecision();
            if (LastKeyframeDecision)
                printf("Publisher: this is keyframe.\n");

            // 获取当前帧位姿
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

            // get loop index
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

                for (int j = 0; j < result_vector.size(); j++)
                {
                    if (result_vector[j].second == loop_result[i].first)
                        first_index = j;
                    if (result_vector[j].second == loop_result[i].second)
                        second_index = j;
                }

                if (first_index > 0 && second_index > 0)
                {
                    printf("Publisher: the loop info %d <---> %d\n", first_index, second_index);
                    loop_channel.values.push_back(first_index);
                    loop_channel.values.push_back(second_index);
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
}