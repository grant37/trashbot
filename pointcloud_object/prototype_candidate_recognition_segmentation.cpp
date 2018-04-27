#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


tf::TransformListener listener;

PointCloudT::Ptr cloud (new PointCloudT);

//Variables for cloudPrep
bool tranformed = 0;

void cloudPrep(const  sensor_msgs::PointCloud2ConstPtr& input)
{
	//Creating container for transformedCloud
	sensor_msgs::PointCloud2 transformedCloud


	//Transform to baselink frame
	transformed = pcl_ros::transformPointCloud(&"***BaselinkFrame***", &input, &transformedCloud, &listener);
	//error catching transform
	std::cerr<<"Transformed?: " << transformed << endl;


  	//Convert to PCL format
	pcl::fromROSMsg(*transformedCloud, *cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "candidate_detection");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("***PointCloudPublishingTopic***", 1000, cloudPrep);


  ros::spin();

  return 0;
}
