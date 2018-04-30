#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


tf::TransformListener listener;

PointCloudT::Ptr cloud (new PointCloudT);

//Variables for cloudPrep
bool tranformed = 0;

/*
cloudPrep is callback of pointcloud topic subscriber 
Takes pointcloud in camera frame of reference, transforms to baselink frame, and converts to PCL format
Stores prepared cloud in global cloud variable
*/
void cloudPrep(const  sensor_msgs::PointCloud2ConstPtr& input)
{
	//Creating container for transformedCloud
	sensor_msgs::PointCloud2 transformedCloud


	//Transform to baselink frame
	transformed = pcl_ros::transformPointCloud(&"/baselink", &input, &transformedCloud, &listener);
	//error catching transform
	//std::cerr<<"Transformed?: " << transformed << endl;

  	//Convert to PCL format
	pcl::fromROSMsg(*transformedCloud, *cloud);
}


void planeSegmenting(double distThresh){

	//Maybe z-dir pass thru filter to get rid of points higher than some threshold height?

	
	//From PCL planar segmentation tutorial http://pointclouds.org/documentation/tutorials/planar_segmentation.php
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  // Create the segmentation object
	  pcl::SACSegmentation<pcl::PointXYZ> seg;
	  // Optional
	  seg.setOptimizeCoefficients (true);
	  // Mandatory
	  seg.setModelType (pcl::SACMODEL_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setDistanceThreshold (distThresh);

	  //look for a plane perpendicular to a given axis
	  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (1000);
	  seg.setDistanceThreshold (0.01);

	  seg.setInputCloud (cloud);
	  seg.segment (*inliers, *coefficients);

	  if (inliers->indices.size () == 0)
	  {
	    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	    return (-1);
	  }

	  //How to get model coefficients out? How will we use them? 

	  //Outputing for debugging
	  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;


	  //Is this the largest plane? Will it always detect the ground plane?
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "candidate_detection");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1, cloudPrep);//keeps 1 cloud in queue to always get most recent

  ros::ServiceServer getCandidates_service = n.advertiseService("getCandidates", planeSegmenting);

  ros::spin();

  return 0;
}
