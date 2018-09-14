#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace map3d{



boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> MutiVis (std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr > cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "registration cloud");
  int step=(255+255+255)/cloud.size();
  int R,G,B;
  for(int i=0;i<cloud.size();i++)
  {
    std::string cloudName;
   char cloudname[50];
   sprintf(cloudname,"cloud%.3d",i);
    if(step*(i+1)<=255)
    {
      R=(i+1)*step;
      G=0;
       B=0;
    }
   else if(step*(i+1)>=255&&step*(i+1)<=510)
    {
     R=255;
     G=(i+1)*step-255;
     B=0;
    }
    else{
       R=255;
      G=255;
      B=(i+1)*step-510;
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud[i], R, G, B);
    viewer->addPointCloud<pcl::PointXYZ>(cloud[i],single_color,cloudName);
  }
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc2)
{
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point"));
  viewer->setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pc1, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> n_color(pc2, 0,0,255);
  //重载函数
 viewer->addPointCloud<pcl::PointXYZ>(pc1,single_color,"sample cloud");
 
 //首先设置渲染特性，PCL_VISUALIZER_POINT_SIZE是一种. 
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"sample cloud");
 //注意后面这个必须在命名时与上面的不同，不然与上面的点云颜色相同
 viewer->addPointCloud<pcl::PointXYZ >(pc2,n_color,"normals");
 viewer->addCoordinateSystem(1.0);
 viewer->initCameraParameters();
return viewer;  
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc, pcl::PointCloud<pcl::Normal>::ConstPtr Normals)
{
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point with Normal"));
  viewer->setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pc, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal> n_color(Normals, 0,0,255);
  //重载函数
 viewer->addPointCloud<pcl::PointXYZ>(pc,single_color,"sample cloud");
 
 //首先设置渲染特性，PCL_VISUALIZER_POINT_SIZE是一种. 
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"sample cloud");
 //注意后面这个必须在命名时与上面的不同，不然与上面的点云颜色相同
 viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal >(pc,Normals,10,5,"normals");
 viewer->addCoordinateSystem(1.0);
 viewer->initCameraParameters();
return viewer;  
}
}