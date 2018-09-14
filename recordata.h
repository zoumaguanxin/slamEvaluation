/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef RECORDATA_H
#define RECORDATA_H
#include<cstring>
#include <boost/concept_check.hpp>
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<tf/transform_listener.h>
#include<Eigen/Dense>
#include<ros/subscriber.h>
#include<ros/node_handle.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>

#include<fstream>



using namespace std;


class recordata
{
public:
recordata()
{}
recordata(const ros::NodeHandle &nh_);

void callbackScan(sensor_msgs::LaserScan::ConstPtr scanPtr);

void callbackImuData(sensor_msgs::Imu::ConstPtr posefromImuPtr);

void callbackOdomData(nav_msgs::Odometry::ConstPtr posefromOdomPtr);

inline void setFrames(const string &base_frame_, const string &laser_frame_, const string & odom_frame_ ,const string &map_frame_)
{
  base_frame=base_frame_;
  odom_frame=odom_frame_;
  laser_frame=laser_frame_;
  map_frame=map_frame_;
  }
  
/**
 * \brief 设置给累文件名字，以及存放的目录
 *@param poseFileName 存放从base_frame相对odom_frame的位姿态 
 *@param scanFileName_ 存放的是激光扫描文件的名字,需要注意的是存放的不是扫描数据，而是经过处理之后在base坐标系下的点云数据
 * @param maposeFileName 存放base相对map坐标系的位姿
 * @param fileDir 存放文件的目录
 */
  inline void
  setFileNames(const string &poseFileName_,const string &scanFileName_,const string &maposeFileName_,const string &fileDir_)
  {
    fileDir=fileDir_;
    poseFileName=poseFileName_;
    scanFileName_prefix=scanFileName_;
    maposeFileName=maposeFileName_;
  }
  
  
  inline void setReposeDistance(const float &reposeDistance_)
  {
   responseDistance=reposeDistance_;
  }
  
  /**
 * \brief this function is responsible for transforming scan into point cloud data in any target frame
 *\param[in] curScan the scan data
 *\param[in] target_frame you want to transform the curScan into target frame
 *\param[out] pcout the point cloud in target frame 
 * \return if the transform has been executed successsly, the function return true, otherwise false
 */
bool scan2foopc( const sensor_msgs::LaserScan& curScan,const string &target_frame, sensor_msgs::PointCloud& pcout);


/**
 * \brief 得到转换关系，如果成功则返回true， 否则返回false
 */
bool getTransform(const string &target_frame, const string &source_frame, const ros::Time&time, tf::StampedTransform &pose);


/**
 * \brief 得到base在里程计下的3个自由度的位姿，注意适用于二维的情况
 */
bool getOdomPose(const string & odom_frame,const string &base_frame,const ros::Time&time, Eigen::Vector3f &pose);

bool getPoseInTargetFrame(const string& target_frame, const string& base_frame, const ros::Time& time, Eigen::Vector3f& pose);
  
 void init();
  
 void recordAction();
 
    
~recordata();


private:
    string base_frame;
    string laser_frame;
    string odom_frame;
    string map_frame;
    string scanFileName_prefix;
    string maposeFileName;
    string fileDir;
    string poseFileName;
    float responseDistance=0.8;
    float responseTheta=0.3444f;
    ros::NodeHandle nh;
    ros::V_Subscriber v_sub;
    sensor_msgs::Imu posefromImu;
    sensor_msgs::LaserScan current_scan;
    nav_msgs::Odometry posefromOdom ;
    tf::TransformListener tf_listener;
};

#endif // RECORDATA_H
