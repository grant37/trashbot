#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/pcl_base.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef sensor_msgs::PointCloud2 PointCloudSnsrMsg;

PointCloudSnsrMsg cloud;


void cloudPub(const  sensor_msgs::PointCloud2ConstPtr& input)
{
         cloud = *input;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "candidate_detection_tester");

  ros::NodeHandle n;

  std::string pubTopic = "segment_this_cloud";

  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(pubTopic, 1, true);//keeps 1 cloud in queue to always get most recent

  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1, cloudPub);
  
  ros::Rate  loop_rate(0.5);
  
  while(ros::ok())
  {
          pub.publish(cloud);
          ros::spinOnce();
          
          loop_rate.sleep();
  }
  
  

  return 0;
}
