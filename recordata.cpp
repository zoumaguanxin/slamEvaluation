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

#include "recordata.h"


recordata::recordata(const ros::NodeHandle& nh_):nh(nh_), tf_listener(nh_,ros::DURATION_MAX)
{

}

void recordata::init()
{
 ros::Subscriber subscan= nh.subscribe<sensor_msgs::LaserScan>("scan",1,boost::bind(&recordata::callbackScan,this,_1));
 ros::Subscriber subImuData=nh.subscribe<sensor_msgs::Imu>("/imu/data",1, boost::bind(&recordata::callbackImuData,this,_1));
 ros::Subscriber subOdomData=nh.subscribe<nav_msgs::Odometry>("/husky_velocity_controller/odom",1,boost::bind(&recordata::callbackOdomData,this,_1));
 v_sub.push_back(subscan);
 v_sub.push_back(subImuData);
 v_sub.push_back(subOdomData);
 nh.param<string>("filedir",fileDir,"/home/shaoan/projects/slamEvaluation/data/");
 nh.param<string>("odom_frame",odom_frame,"odom");
 nh.param<string>("laser_frame",laser_frame,"laser");
 nh.param<string>("base_frame",base_frame,"base_link");
 nh.param<string>("map_frame",map_frame,"map");
 nh.param<float>("reponse_dist", responseDistance, 2.0f); 
 nh.param<string>("poseFileName",poseFileName, "pose.txt");
 nh.param<string>("scanFileNameprefix",scanFileName_prefix,"scan");
 nh.param<string>("optimizedposeFileName",maposeFileName,"optimizedpose.txt");
}

void recordata::callbackScan(sensor_msgs::LaserScan::ConstPtr scanPtr)
{
    current_scan=*scanPtr;
}


void recordata::callbackImuData(sensor_msgs::Imu::ConstPtr posefromImuPtr)
{
  
   posefromImu=*posefromImuPtr;
}

void recordata::callbackOdomData(nav_msgs::Odometry::ConstPtr posefromOdomPtr)
{
   posefromOdom=*posefromOdomPtr;
}


bool recordata::scan2foopc(const sensor_msgs::LaserScan& curScan, const string& target_frame, sensor_msgs::PointCloud& pcout)
{
  //首先转换为激光坐标系下的点云
   sensor_msgs::PointCloud pcin;
  for(int i=0;i<curScan.ranges.size();i++)
  {
    geometry_msgs::Point32 point;
    if(current_scan.ranges[i]<current_scan.range_max)
    {
    point.x=curScan.ranges[i]*cos(curScan.angle_min+i*curScan.angle_increment);
    point.y=curScan.ranges[i]*sin(curScan.angle_min+i*curScan.angle_increment);
    point.z=0;
    pcin.points.push_back(point);
    }
  }
      pcin.header.frame_id=curScan.header.frame_id;
      pcin.header.stamp=current_scan.header.stamp;
      
      //使用transform进行转换
      bool transflag=false;
      geometry_msgs::TransformStamped tfstamp;
      try{
	    tf_listener.transformPointCloud(target_frame,pcin,pcout);
	    transflag=true;
	   // cout<<"transform is ok"<<endl;
	}
     catch(tf::TransformException &ex)
       {
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();  
       }
    return transflag; 

}





bool recordata::getTransform(const string& target_frame, const string& source_frame, const ros::Time& time, tf::StampedTransform& pose)
{
  bool flag=false;
  try{
    tf_listener.lookupTransform(target_frame,source_frame,time,pose);
    flag=true;
  }
  catch(tf::LookupException &ex)
  {
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();  
  }
  return flag;
}



bool recordata::getOdomPose(const string& odom_frame, const string& base_frame, const ros::Time& time, Eigen::Vector3f& pose)
{
 
  tf::StampedTransform odompose;
  bool flag=false;
  try{
    tf_listener.lookupTransform(odom_frame,base_frame,time,odompose);
    pose(0)=odompose.getOrigin().x();
    pose(1)=odompose.getOrigin().y();
    pose(2)=tf::getYaw(odompose.getRotation());
    flag=true;
  }
  catch(tf::LookupException &ex)
  {
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();  
  }
  return flag;
}


bool recordata::getPoseInTargetFrame(const string& target_frame, const string& base_frame, const ros::Time& time, Eigen::Vector3f& pose)
{
 
  tf::StampedTransform tempose;
  bool flag=false;
  try{
    tf_listener.lookupTransform(target_frame,base_frame,time,tempose);
    pose(0)=tempose.getOrigin().x();
    pose(1)=tempose.getOrigin().y();
    pose(2)=tf::getYaw(tempose.getRotation());
    flag=true;
  }
  catch(tf::LookupException &ex)
  {
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();  
  }
  return flag;
}


void recordata::recordAction()
{
  ros::Rate r(50);
  Eigen::Vector3f last_odompose, current_odompose;
  bool beginflag=false;
  ofstream file,maposefile,pcfile,odomfile,imudatafile;
  assert(!poseFileName.empty());
  char f1[100],f2[100],imuposefile[100],odomposefile[100];
  sprintf(f1,"%s%s",fileDir.c_str(),poseFileName.c_str());
  sprintf(imuposefile,"%s%s",fileDir.c_str(),"imudata.txt");
  sprintf(odomposefile,"%s%s",fileDir.c_str(),"odom.txt");
  cout<<f1<<endl;
  file.open(f1,ios_base::out);
  assert(!maposeFileName.empty());
  sprintf(f2,"%s%s",fileDir.c_str(),maposeFileName.c_str());
  maposefile.open(f2,ios_base::out);
  odomfile.open(odomposefile,ios_base::out);
  imudatafile.open(imuposefile,ios_base::out);
  char pointcloudFileName[100];
  int n=0;
  while(ros::ok())
  {
    current_scan.ranges.clear();
    ros::spinOnce();
    Eigen::Vector3f odompose,mapose;
    sensor_msgs::PointCloud pc;
    if( getOdomPose(odom_frame,base_frame,ros::Time(0),odompose)&&(!current_scan.ranges.empty())&&scan2foopc(current_scan,base_frame,pc)&&getPoseInTargetFrame(map_frame,base_frame,ros::Time(0),mapose))
    {     
      if(!beginflag)
      {  	
	  last_odompose=odompose;
	  beginflag=true;
	  
		if(maposefile.good())
		{
		   cout<<"写入优化后的位姿base_link到map: x,y,Yaw"<<endl;
		  maposefile<<mapose(0)<<" "<<mapose(1)<<" "<<mapose(2)<<endl;
		}
		else{
		  cout<<"写入base_link到map的tf异常"<<std::endl;
		  exit(0);
		}
		
		
		  if(file.good())
		    {
		        cout<<"写入base_link到odom的tf: x,y,Yaw"<<endl;
		      file<<odompose(0)<<" "<<odompose(1)<<" "<<odompose(2)<<endl;
		    }
		    else{
		      cout<<"写入base_link到odom的tf异常"<<endl;
		      exit(0);
		    }
		    
		    
		    sprintf(pointcloudFileName,"%sscan%.3d.txt",fileDir.c_str(),n);
	             pcfile.open(pointcloudFileName,ios_base::out);
		    if(pcfile.good())
		    {
		       cout<<"写入点云"<<endl;
		      for(int i=0;i<pc.points.size();i++)
			{			  
			  pcfile<<pc.points[i].x<<" "<<pc.points[i].y<<" "<<pc.points[i].z<<std::endl;
			}
			n++;
		      pcfile.close();        
		    }
		    else{
		      cout<<"写入点云异常"<<endl;
		      exit(0);
		    }
		    	    
		    
		    if(odomfile.good())
		    {
		       std::cout<<"写入odom话题：x,y, Yaw"<<std::endl;
		      odomfile<<posefromOdom.pose.pose.position.x<<" "<<posefromOdom.pose.pose.position.y<<" ";
		      odomfile<<tf::getYaw(posefromOdom.pose.pose.orientation)<<std::endl;
		      }
		      else{
			cout<<"写入里程计topic异常"<<std::endl;
			exit(0);
		      }
		      
		      if(imudatafile.good())
		      {
			std::cout<<"写入Imu话题：偏行角，三个方向的角速度，三个方向的线性加速度"<<std::endl;
			imudatafile<<tf::getYaw(posefromImu.orientation)<<" ";
			imudatafile<< posefromImu.angular_velocity.x<<" ";
			imudatafile<<posefromImu.angular_velocity.y<<" ";
			imudatafile<<posefromImu.angular_velocity.z<<" ";
			imudatafile<<posefromImu.linear_acceleration.x<<" ";
			imudatafile<<posefromImu.linear_acceleration.y<<" ";
			imudatafile<<posefromImu.linear_acceleration.z<<std::endl;
		      }
		    else{
		      cout<<"The error occured: write the data into "<<imuposefile<<std::endl;
		      exit(0);
		    }
	 }
      else{
	double dist_diff=sqrt(pow<float>(odompose(0)-last_odompose(0),2)+pow<float>(odompose(1)-last_odompose(1),2));
	bool dist_sample_flag=(dist_diff>responseDistance);
	double angular_diff=odompose(2)-last_odompose(2);
	bool angular_sample_flag=(abs(angular_diff)>=responseTheta);
	bool scan_flag=scan2foopc(current_scan,base_frame,pc);
	bool tf_base2map_flag=getPoseInTargetFrame(map_frame,base_frame,ros::Time(0),mapose);
	if((dist_sample_flag||angular_sample_flag)&&scan_flag&&tf_base2map_flag)
	{
	              last_odompose=odompose;
	
		      if(maposefile.good())
		      {
			cout<<"写入优化后的位姿base_link到map: x,y,Yaw"<<endl;
			maposefile<<mapose(0)<<" "<<mapose(1)<<" "<<mapose(2)<<endl;
		      }
		      else{
			cout<<"写入异常"<<endl;
			exit(0);
		      }   
		
		  
		  sprintf(pointcloudFileName,"%sscan%.3d.txt",fileDir.c_str(),n);
		  pcfile.open(pointcloudFileName,ios_base::out);
		  if(pcfile.good())
		    {		
		      cout<<"写入点云:x,y,z"<<endl;
		      for(int i=0;i<pc.points.size();i++)
			{			  
			  pcfile<<pc.points[i].x<<" "<<pc.points[i].y<<" "<<pc.points[i].z<<endl;
			}
			n++;
		      pcfile.close();
		    }
		    else{
		      cout<<"写入点云异常"<<endl;
		      exit(0);
		    }
		    
		    
		    if(file.good())
		    {
		         cout<<"写入base_link到odom的tf: x,y,Yaw"<<endl;
		      file<<odompose(0)<<" "<<odompose(1)<<" "<<odompose(2)<<endl;
		     }
		    else{
		            cout<<"写入base_link到odom的tf异常"<<endl;
		      exit(0);
		    } 
	    
	    
		    
		    if(odomfile.good())
		     {
		      std::cout<<"写入odom话题：x,y, Yaw"<<std::endl;
		      odomfile<<posefromOdom.pose.pose.position.x<<" "<<posefromOdom.pose.pose.position.y<<" ";
		      odomfile<<tf::getYaw(posefromOdom.pose.pose.orientation)<<std::endl;
		      }
		      else{
			cout<<"写入里程计topic异常"<<std::endl;
			exit(0);
		      }
		      
		      
		      if(imudatafile.good())
		      {
			std::cout<<"写入Imu话题：偏行角，三个方向的角速度，三个方向的线性加速度"<<std::endl;
			imudatafile<<tf::getYaw(posefromImu.orientation)<<" ";
			imudatafile<< posefromImu.angular_velocity.x<<" ";
			imudatafile<<posefromImu.angular_velocity.y<<"  ";
			imudatafile<<posefromImu.angular_velocity.z<<" ";
			imudatafile<<posefromImu.linear_acceleration.x<<" ";
			imudatafile<<posefromImu.linear_acceleration.y<<" ";
			imudatafile<<posefromImu.linear_acceleration.z<<std::endl;
		      }
		    else{
		      cout<<"The error occured: write the data into "<<imuposefile<<std::endl;
		      exit(0);
		    }	  
	}
      }
    }
    r.sleep();
  }
  file.close();
  maposefile.close();
  odomfile.close();
  imudatafile.close();
  
}


recordata::~recordata()
{

}
