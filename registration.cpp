#include "predeal.h"
#include "pairwiseICP.hpp"
#include "viz.hpp"
#include <boost/graph/graph_concepts.hpp>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/registration/transforms.h>
#include<ctime>
#include<cmath>




using namespace std;
int main()
{
  
  //*************************
  //读取两个pose文件
  //************************
  string posedir="/home/shaoan/projects/slamEvaluation/data/pose.txt";
  vector<Eigen::Vector3f> v_pose;
  predeal::readposefile(posedir,v_pose);
    string maposedir="/home/shaoan/projects/slamEvaluation/data/optimizedpose.txt";
  vector<Eigen::Vector3f> v_mapose;
  predeal::readposefile(maposedir,v_mapose);
  
  if(v_mapose.size()!=v_pose.size())
  {
    cout<<"数据记录有误"<<endl;
    exit(0);
  }
  
  
  //**************************************************************
  //创建两个新文件，用来保存手动配准的位姿和slam算法给出的位姿
  //**************************************************************
ofstream rgsdPosefile, maPosefile;
  string rgsPosefileName="/home/shaoan/projects/slamEvaluation/data/rgspose.txt";
 rgsdPosefile.open(rgsPosefileName.c_str(),ios_base::out);
 string maPosefileName="/home/shaoan/projects/slamEvaluation/data/maPose.txt";
 maPosefile.open(maPosefileName.c_str(),ios_base::out);
 
 
  
  //************************************************
  //手动配准，并观察效果，决定是否记录
  //************************************************
  time_t begin,finish ;
  
 //const int numFile=v_mapose.size();
  const int numFile=9;
 for(int i=0;i<numFile-1;i++)
  {
    char target_pcddir[100];
    char source_pcddir[100];
    pcl::PointCloud<pcl::PointXYZ> source_pcd, target_pcd;
    string dir="/home/shaoan/projects/slamEvaluation/data/";
    sprintf(target_pcddir,"%sscan%.3d.txt",dir.c_str(),i);
    predeal::readpcdfile(target_pcddir,target_pcd);
    sprintf(source_pcddir,"%sscan%.3d.txt",dir.c_str(),i+1);
    predeal::readpcdfile(source_pcddir,source_pcd);
    
    /*
    Eigen::Vector3f tgt_pose,src_pose;
    tgt_pose=v_pose[i];
    src_pose=v_pose[i+1];
     src_pose(2)=2*M_PI+src_pose(2);
      tgt_pose(2)=2*M_PI+tgt_pose(2);
     Eigen::Quaternion<float> q_w_target(cos(tgt_pose(2)/2),0,0,sin(tgt_pose(2)/2));
    Eigen::Quaternion<float> q_w_source(cos(src_pose(2)/2),0,0,sin(src_pose(2)/2));
    Eigen::Vector3f t_w_target, t_w_source;
   t_w_target<<tgt_pose(0), tgt_pose(1),0.0f;
   t_w_source<<src_pose(0), src_pose(1),0.0f;
    Eigen::Matrix3f R_initial=q_w_target.toRotationMatrix().transpose()*q_w_source.toRotationMatrix();
    Eigen::Vector3f t_initial=q_w_target.toRotationMatrix().transpose()*(t_w_source-t_w_target);
    */

    
    Eigen::Matrix3f R_initial;
    Eigen::Vector3f t_initial;    
      Eigen::Vector3f odomEdge=predeal::computeConstraint(v_pose[i],v_pose[i+1]);    
    predeal::vector3ftoRationTtrans(odomEdge,R_initial,t_initial);  
    Eigen::Vector3f maposeEdge;
    maposeEdge=predeal::computeConstraint(v_mapose[i],v_mapose[i+1]);
  //predeal::vector3ftoRationTtrans(maposeEdge,R_initial,t_initial);  
  
    

   Eigen::Matrix3f R=R_initial;
   Eigen::Vector3f t=t_initial;
   

   map3d::ICP icp;
   begin=clock();
   icp.setInputCloud(source_pcd.makeShared(),target_pcd.makeShared());
   icp.setNum(100);
   icp.setParamsConvergence(200, 2, 0.02, 0.05);
   icp.setReJectThrehold(2, 0.1,0.01,0.01);
   icp.solve(R,t);
   finish=clock();
  cout<<R<<endl;
  cout<<t<<endl;

  
  Eigen::Matrix4f T=Eigen::Matrix4f::Identity();
  T.block(0,0,3,3)=R;T.block(0,3,3,1)=t;
  cout<<T<<endl;
  
  //**************************************************
   //可视化
   //***************************************************
  pcl::PointCloud<pcl::PointXYZ> transformedpcd;
  pcl::transformPointCloud(source_pcd,transformedpcd,T);
  assert(!source_pcd.empty());
 // vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr > v_pcd;
 // v_pcd.push_back(target_pcd.makeShared());v_pcd.push_back(transformedpcd.makeShared());
  boost::shared_ptr<pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer);
  viewer=map3d::normalVis(target_pcd.makeShared(),transformedpcd.makeShared());
  while(!viewer->wasStopped())
  {
    viewer->spinOnce();
     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  
  
  //***************************************
  //保存数据
  //***************************************
  cout<<"是否保存当前数据用于评估（y/n）"<<endl;
  char keyValue;
 L1: cin>>keyValue;
  if(keyValue=='y')
  {
    if(rgsdPosefile.good()&&maPosefile.good())
    {
      Eigen::Quaternion<float> q(R);
      float theta;
      float a1=acos(q.w());
      float a2=-acos(q.w());
      if(abs(sin(a1)-q.z())<0.001f)
	{
	  theta=2*a1;
	}
	else if(abs(sin(a2)-q.z())<0.001f)
	{
	  theta=2*a2;
	  //cout<<v(2)<<endl;
	}
      rgsdPosefile<<t(0)<<" "<<t(1)<<" "<<theta<<endl;
      maPosefile<<maposeEdge(0)<<" "<<maposeEdge(1)<<" "<<maposeEdge(2)<<endl;
    }
    else
    {
      cout<<"未能保存位姿"<<endl;
      exit(0);
    }
  }
  else if(keyValue=='n')
  {
    continue;
  }
  else
  {
    cout<<"未知输入，请重新输入"<<endl;
    goto L1;
  }
  
  }
  cout<<"ICP 花费的时间："<<double(finish-begin)/CLOCKS_PER_SEC<<endl;  
  rgsdPosefile.close();
  maPosefile.close();
  
  
  
  
}