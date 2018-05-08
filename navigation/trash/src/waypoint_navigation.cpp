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

#include <vector>

#define PI 3.14159265359

void publish_image();

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
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	// send the goal
	ac.sendGoal(goal);

	//block until the action is completed
	ac.waitForResult();
	//std::cout<<"\n \n waiting for " << sleep_time<<" seconds \n \n";
	//sleep(sleep_time);
  
  	return 0;
}

int main(int argc, char **argv)
{
	
	std::cerr << "starting up..." << std::endl;
	ros::init(argc, argv, "move_base_client");
	ros::NodeHandle n;
    
    // ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("camera/rgb/image_raw", 50);
	// std::cerr << "Publishing image..." << std::endl;

	ros::Publisher init_check = n.advertise<std_msgs::Bool>("search_with_cv", 50);


	// the order is:
	// 0: right outside lab
	// 1: right outside collaborative lounge
	// 2: right outside kitchenette
	// 3: close to entrnace stairs
	

	//check the locations on the map to make sure they match
	double home_location[3] = {12.70,24.27,0.0};
	
	int num_locations = 2;
	double locations[2][3] = { {13.00,24.77,0.0}, {12.70, 24.27, 0.0} };
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
	ac.waitForServer(); //wait to make sure the service is there
	move_base_msgs::MoveBaseGoal goal;
  
	double x1,y1,x2,y2,x3,y3,x4,y4 = 0;
	
	int c = 0;  
	
	while (ros::ok()) {
		
		// move to next location
		// move_turtle_bot(locations[c][0],locations[c][1],locations[c][2], false);
		
        ros::Rate loop_rate(10);
		for (int i = 0; i < 12; i ++){
			std::cerr << "rotate behavior /n" << std::endl;
			std::cerr << i << std::endl;
			// turn
			move_turtle_bot(0.0, 0.0, PI/4, true);            
  			
		 	int count = 0;
				     
            // sensor_msgs::Image view;
            // view.header.stamp = ros::Time::now();
            // view.header.frame_id = "camera/rgb/image_raw";
            
            // image_pub.publish(view);

            ros::Time startTime = ros::Time::now();
            ros::Duration seconds_to_spend = ros::Duration(4);
            ros::Time endTime = seconds_to_spend + startTime;

            while(ros::Time::now() < endTime) {
            	init_check.publish(true);
            	ros::Duration(0.1).sleep();
            }

            init_check.publish(false); // tell trash_detector to stop looking
            
	    	++count;

            loop_rate.sleep();
        }
		
		std::cerr << "out of while loop" << std::endl;
		sleepok(10,n);
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
