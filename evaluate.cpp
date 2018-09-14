#include "evaluation.h"
#include "predeal.h"
#include <boost/graph/graph_concepts.hpp>

using namespace std;


Eigen::Vector3f posediff(const Eigen::Vector3f& x, const Eigen::Vector3f &y)
{
  Eigen::Vector3f v;
  v(0)=abs(x(0)-y(0));
  v(1)=abs(x(1)-y(1));
  float x2,y2;
  if(x(2)<0)
  {
    x2=2*M_PI+x(2);
  }
  else
  {
    x2=x(2);
  }
  if(y(2)<0)
  {
    y2=2*M_PI+y(2);
  }
  else
  {
    y2=y(2);
  }
  v(2)=abs(x2-y2);
  return v;
}

int main()
{
   string posedir="/home/shaoan/projects/slamEvaluation/data/rgspose.txt";
  vector<Eigen::Vector3f> v_pose;

  predeal::readposefile(posedir,v_pose);
  string maposedir="/home/shaoan/projects/slamEvaluation/data/maPose.txt";
  vector<Eigen::Vector3f> v_mapose;

  predeal::readposefile(maposedir,v_mapose);

 assert(v_mapose.size()==v_pose.size());
 assert(v_pose.size()>0);
 
     vector<Eigen::Vector3f> differnce;
     for(int i=0;i<v_pose.size();i++)
     {
       Eigen::Vector3f v;
       v=posediff(v_mapose[i],v_pose[i]);
       //v=v_mapose[i]-v_pose[i];
       //cout<<v<<endl;
       differnce.push_back(v);
    }

    
    evaluation eval(differnce);
    Eigen::Vector2f v2;
   v2= eval.computeMSE();   
   cout<<"平均绝对误差："<<endl;
   cout<<v2<<endl;
   Eigen::Vector3f v3;
   v3=eval.computeMAE();
  cout<<"平均绝对误差："<<endl;
  cout<<v3<<endl; 
    
}

