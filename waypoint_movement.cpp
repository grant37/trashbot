#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

#include <vector>

#define PI 3.14159265359



//wait for next waypoint instruction
void sleepok(int t, ros::NodeHandle &nh)
{
	if (nh.ok())
		sleep(t);
}


//move to desired location
int move_turtle_bot (double x, double y, double yaw)
{
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
  	ac.waitForServer(); //wait to make sure the service is there
   	move_base_msgs::MoveBaseGoal goal;
 
	
	std::cout<<"Going to :"<< x  << y;
	
	//set the header
    	goal.target_pose.header.stamp = ros::Time::now();
    	goal.target_pose.header.frame_id = "/map";
	  
    	//set relative x, y, and angle
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


//turn in a circle
int turn_turtle_bot(double yaw)
{
	//set new yaw goal 
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + 2*PI);
	
	ac.sendGoal(goal);
	ac.waitForResult();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_base_client");
	ros::NodeHandle n;



	//the order is:
	// 0: right outside lab
	// 1: right outside collaborative lounge
	// 2: right outside kitchenette
	// 3: close to entrnace stairs
	

//check the locations on the map to make sure they match
	double home_location[3] = {5.65,13.8,0.0};
	
	int num_locations = 7;
	double locations[7][3] = { {21.7,13.7,0.0},{21.8,5.9,0.0},{-0.329,6.21,0.0},{1.0,13.6,0.0},	{5.65,13.8,0.0},{7.5,9.8,0.0},{21.3,19.5,0.0} };
	
	

	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
	ac.waitForServer(); //wait to make sure the service is there
	move_base_msgs::MoveBaseGoal goal;
  
	double x1,y1,x2,y2,x3,y3,x4,y4 = 0;
	
	int c = 0;  
	
	while (ros::ok()) {
		
		//move to next location
		move_turtle_bot(locations[c][0],locations[c][1],locations[c][2]);
		
		for (int p = 0; p < 3; p ++){
			//call function to turn in a circle and search for trash
			sleepok(//time,n);
			move_turtle_bot(locations[c][0],locations[c][1],locations[c][2]+(p+1)*PI/2);
		}
		
		sleepok(10,n);

		
		//increment location 
		c++;

		//allows loop to run continuously
		if (c >= num_locations){
			c = 0;
		}
	}
	 
	return 0;

}
