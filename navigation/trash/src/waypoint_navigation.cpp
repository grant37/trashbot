#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <vector>

#define PI 3.14159265359

class detectionListener
{
public:
	void detector_callback(std_msgs::Bool data);
	bool detectionResult;
}

void detectionListener::detector_callback(std_msgs::Bool data) {

	this.detectionResult = data.data;

}


// wait for next waypoint instruction
void sleepok(int t, ros::NodeHandle &nh)
{
	if (nh.ok())
		sleep(t);
}


// move to desired location
int move_turtle_bot (double x, double y, double yaw, bool turn)
{
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  	ac.waitForServer(); // wait to make sure the service is there
   	move_base_msgs::MoveBaseGoal goal;
 
	
	std::cerr << "Going to :" << x  << y;
	
	// set the header
	goal.target_pose.header.stamp = ros::Time::now();
	
	// specify frame
	if (!turn)
		goal.target_pose.header.frame_id = "/map";
	else
		goal.target_pose.header.frame_id = "base_link";
  
	//set relative x, y, and angle
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.position.z = 0.0;

	// set the orientation to face forward if we've moved forward but are using base_link
	if ((x != 0 or y != 0) and turn)
		yaw = atan(y/x);

	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	// send the goal
	ac.sendGoal(goal);

	//block until the action is completed
	ac.waitForResult();
	//std::cout<<"\n \n waiting for " << sleep_time<<" seconds \n \n";
	sleep(1.0);
  
  	return 0;
}

// the order is:
// 0: right outside lab
// 1: right outside collaborative lounge
// 2: right outside kitchenette
// 3: close to entrnace stairs

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "move_base_client");
	ros::NodeHandle n;

	std_msgs::Bool search;
	search.data = false;

	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;


    // trash detection handle
	detectionListener listener;
	ros::Subsciber trash_sub;

	// publishers
	ros::Publisher init_check = n.advertise<std_msgs::Bool>("search_with_cv", 50);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// don't look for trash while wandering the halls
	init_check.publish(search);
	
	// Note! check the locations on the map to make sure they match
	double home_location[3] = {12.70, 24.27, 0.0};
	
	int num_locations = 2;
	double locations[2][3] = { {13.00, 24.77, 0.0}, {12.70, 24.27, 0.0} };
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
	ac.waitForServer();
	move_base_msgs::MoveBaseGoal goal;
  
  	// track of locations	
	int c = 0;

	// set footprint hack
	move_turtle_bot(0.5, 0.0, 0.0, true);
	
	while (ros::ok()) {
		
		// move to next location
		// move_turtle_bot(locations[c][0],locations[c][1],locations[c][2], false);
		
		// Rotate in place to search for trash
        ros::Rate loop_rate(0.2);
        int i = 0;

		while (i < 12){
			trash_sub = n.subscribe("trash_detector/status", 10, &detectionListener::detector_callback, &listener);
			i++;
			std::cerr << "rotate behavior /n" << std::endl;
			std::cerr << i << std::endl;

			search.data = true;
            init_check.publish(search); // tell trash_detector to start looking

            loop_rate.sleep();
        }

        if (listener.detectionResult) {
        	std::cerr << "result" << std::endl;
        	marker.pose.position.x = locations[c][0];
   			marker.pose.position.y = locations[c][1];
    		marker.pose.position.z = locations[c][2];
    		marker.pose.orientation.x = 0.0;
    		marker.pose.orientation.y = 0.0;
    		marker.pose.orientation.z = 0.0;
    		marker.pose.orientation.w = 1.0;

    		ros::Subsciber marker_sub = n.subscribe("visualization_marker", 10);

    		marker_pub.publish(marker);
    	}


        // reset in order to roam again
        search.data = false;
        init_check.publish(search);
		
		std::cerr << "out of while loop" << std::endl;
		sleepok(5,n);
		std::cerr << "out of sleepok" << std::endl;
		
		// increment location
		c++;

		// allows loop to run continuously
		if (c >= num_locations) {
			c = 0;
		}
	}
	 
	return 0;
}
