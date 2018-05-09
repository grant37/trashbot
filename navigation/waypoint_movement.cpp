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

#include <vector>
#include <navigation/waypoint_destinations.h>

#define PI 3.14159265359

void publish_image();

//wait for next waypoint instruction
void sleepok(int t, ros::NodeHandle &nh)
{
	if (nh.ok())
		sleep(t);
}


//move to desired location
int move_turtle_bot (double x, double y, double yaw)
{
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  	ac.waitForServer(); //wait to make sure the service is there
   	move_base_msgs::MoveBaseGoal goal;
 
	//print desired location
	std::cout<<"Going to :"<< x  << y;

    	goal.target_pose.header.stamp = ros::Time::now();
    	goal.target_pose.header.frame_id = "/map";
	  
    	goal.target_pose.pose.position.x = x;
    	goal.target_pose.pose.position.y = y;
    	goal.target_pose.pose.position.z = 0.0;
    	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	//send the goal
    	ac.sendGoal(goal);
    
    	//block until the action is completed
    	ac.waitForResult();
    	//std::cout<<"\n \n waiting for " << sleep_time<<" seconds \n \n";
    	//sleep(sleep_time);
  
  	return 0;
}


////turn in a circle
//int turn_turtle_bot(double yaw)
//{
//    //set new yaw goal
//    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + 2*PI);
//
//    ac.sendGoal(goal);
//    ac.waitForResult();
//
//    return 0;
//}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_base_client");
	ros::NodeHandle n;
    
    ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("camera/rgb/image_raw", 50)

	double home_location[3] = {5.65,13.8,0.0};
	double locations[waypoint_destination.num_locations][3] 
	
	for (int j = 0, j < waypoint_destination,num_locations, j ++){
		location[j][1] = location_point[j][1];
		location[j][2] = location_point[j][2];
		location[j][3] = location_point[j][3];
	}

	
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
	ac.waitForServer(); //wait to make sure the service is there
	move_base_msgs::MoveBaseGoal goal;
  
	double x1,y1,x2,y2,x3,y3,x4,y4 = 0;
	
	int c = 0;  
	
	while (ros::ok()) {
		
		//move to next location
		move_turtle_bot(locations[c][0],locations[c][1],locations[c][2]);
		
		ros::Subscriber pos_candidates = n.subscribe("/geometry_msgs",1000, Candidates);
        ros::Rate loop_rate(1.0);
		for (int i = 0; i < 5; i ++){

			//get coordinates of candidate locations
			//geometry_msgs.Pose.Candidates = new_goal;
			
			//move towards candidates
			//move_turtle_bot(new_goal.x,new_goal.y,0.0);
			
			
			// turn
			move_turtle_bot(0, 0, 1.0);

			// ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);
            
  			

//              unsigned int num_points = 100;

		 	int count = 0;
		  	ros::Rate r(1.0);
		  	while(n.ok()){
                
//                sensor_msgs::PointCloud cloud;
//                cloud.header.stamp = ros::Time::now();
//                cloud.header.frame_id = "sensor_frame";
//
//                cloud.points.resize(num_points);
//
//                cloud_pub.publish(cloud);
                
                sensor_msgs::Image view;
                view.header.stamp = ros::Time::now();
                view.header.frame_id = "camera/rgb/image_raw";
                
                image_pub.publish(view);
                
		    	++count;
		    	r.sleep();
		  	}
					
            loop_rate.sleep();
        }
		
		sleepok(10,n);

		
		// increment location
		c++;

		// allows loop to run continuously
		if (c >= num_locations) {
			c = 0;
		}
	}
	 
	return 0;

}
