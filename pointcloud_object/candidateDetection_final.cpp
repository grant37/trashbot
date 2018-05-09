#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/Pose.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointX> PointCloudX;

//Global pointers
boost::shared_ptr<PointCloudX> transformedCloud;
boost::shared_ptr<tf::TransformListener> listener;
ros::Publisher cands;


void clustering() {
	
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = *transformedCloud;
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
	
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered3);
  //Z- filter:  remove points more than 0.5m above baselink
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_filtered3);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 0.5);
  pass.filter(*cloud_filtered2);
  //Y- filter:  remove points more than 15m away in y-dir
  pass.setInputCloud(cloud_filtered2);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0.0, 15);
  pass.filter(*cloud_filtered1);
  //X- filter: remove points more than 15m away in x direcion
  pass.setInputCloud(cloud_filtered1);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.0, 15);
  pass.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

 
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  std::cerr << cluster_indices.size() << "Clusters found" << std::endl;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

	//Calcuating centroid and publishing
    int x = 0, y = 0, i;
    geometry_msgs::Pose centroid;
    for(i = 0; i < cloud_cluster->points.size(); i++){
		x += cloud_cluster->points[i].x;
		y += cloud_cluster->points[i].y;
	}
	centroid.position.x = x/cloud_cluster->points.size();
	centroid.position.y = y/cloud_cluster->points.size();
	cands.publish(centroid);
	std::stringstream ss;
	ss << "cloud_cluster_" << j << ".pcd";
	writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);
	j++;
	
  }
}


/*
cloudPrep is callback of pointcloud topic subscriber 
Takes pointcloud in camera frame of reference, transforms to baselink frame, and converts to PCL format
Stores prepared cloud in global cloud variable
*/
void cloudPrep(const  sensor_msgs::PointCloud2ConstPtr& input)
{
    
    //Transform to baselink frame
    const std::string destFrame = "/base_link";   //destination frame: /map/odom/base_footprint/base_link

	sensor_msgs::PointCloud2 out;
    bool transformed = pcl_ros::transformPointCloud(destFrame, *input, out, *listener);  
        //http://docs.ros.org/kinetic/api//html/namespacepcl__ros.html#aad1ce4ad90ab784aae6158419ad54d5f
        
    //*transformedCloud  = out;
    pcl::PCLPointCloud2 pcl2;
    pcl_conversions::toPCL(out, pcl2);
    pcl::fromPCLPointCloud2(pcl2, *transformedCloud);
      
	//error catching transform
	std::cerr<<"Transformed?: " << transformed << std::endl;
	std::cerr << "cloud size before call: " << transformedCloud->size() << std::endl;
    if(transformedCloud->size() > 0){
		    std::cerr << "Clustering:" << std::endl;
		    clustering();
	}    
  	std::cerr << "End cloudPrep" << std::endl;
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "tutorial_candidate_detection");
  
  ros::NodeHandle n;
  
  listener = boost::make_shared<tf::TransformListener>();
  transformedCloud = boost::make_shared<PointCloudX>();
  
  ros::Subscriber sub = n.subscribe("segment_this_cloud", 1, cloudPrep);//keeps 1 cloud in queue to always get most recent

  cands = n.advertise<geometry_msgs::Pose>("Candidates", 1000); 

  ros::spin();
  

  return 0;
}
