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

#include <stdlib.h>
//template <typename PointT> class PointCloud;



typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointX> PointCloudX;


PointCloudX::Ptr cloud (new PointCloudX);

void planeSegmenting(){
	  std::cerr << "Inside planeSegmenting" << std::endl;
         double distThresh = 10;
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
                //cloud needs to by xyz not xyzrgb
	  seg.setInputCloud (cloud);
	  seg.segment (*inliers, *coefficients);

	  if (inliers->indices.size () == 0)
	  {
	    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	  }

	  //How to get model coefficients out? How will we use them? 

	  //Outputing for debugging
	  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;


	  //Is this the largest plane? Will it always detect the ground plane?
	  
	  std::cerr << "End planeSegmenting" << std::endl;
}

/*
cloudPrep is callback of pointcloud topic subscriber 
Takes pointcloud in camera frame of reference, transforms to baselink frame, and converts to PCL format
Stores prepared cloud in global cloud variable
*/
void cloudPrep(const  sensor_msgs::PointCloud2ConstPtr& input)
{
	  std::cerr << "Inside cloudPrep" << std::endl;
        //Convert to PCL format
        PointCloudT inputPCL;
        
	pcl::fromROSMsg(*input, inputPCL);     
           std::cerr << "Converted to pcl" << std::endl;
        //Creating container for transformedCloud
	PointCloudT transformedCloud;

        //destination frame: /map/odom/base_footprint/base_link
        const std::string destFrame = "/map->/odom->/base_footprint->/base_link";
        
	//Transform to baselink frame
	tf::TransformListener listener;
	//listener.waitForTransform(destFrame, inputPCL.header.frame_id, ros::Time::now(), ros::Duration(0.1));
	bool transformed = pcl_ros::transformPointCloud<PointT>(destFrame, inputPCL, transformedCloud, listener);
        //http://docs.ros.org/kinetic/api//html/namespacepcl__ros.html#aad1ce4ad90ab784aae6158419ad54d5f
        
	//error catching transform
	std::cerr<<"Transformed?: " << transformed << std::endl;
	
    if(transformedCloud.size() > 0){
		    planeSegmenting();

	}    
  	std::cerr << "End cloudPrep" << std::endl;
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "candidate_detection");
  
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("segment_this_cloud", 1, cloudPrep);//keeps 1 cloud in queue to always get most recent

  ros::spin();
  


  return 0;
}

