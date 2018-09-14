#include <iostream>
#include <ros/ros.h>
#include<ros/node_handle.h>
#include "recordata.h"

int main(int argc, char **argv) {
 
  ros::init(argc,argv,"recordata");
  ros::NodeHandle nh;
  recordata rd(nh);
  rd.init();
  rd.recordAction();  
}
