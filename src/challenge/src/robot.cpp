// ROS libraries
#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"

// C++ libraries
#include <sstream>
#include <iostream> 
#include <queue>
#include <cmath>

using namespace std;

queue <turtlesim::Pose> q;

// Math functions
double degrees2radians(double angle_in_degrees){
	return angle_in_degrees * M_PI / 180.0;
}

double getDistance(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}


// Robot class. 

class Robot {
	private:
		ros::NodeHandle n;

		ros::Publisher vel_pub;		//Object that will publish the turtle velocity
		ros::Publisher feedback_pub;	

		ros::Subscriber pose_sub;	//Object that will listen for the turtle position
		ros::Subscriber messages_sub;	//Object that will listen for directions

		turtlesim::Pose turtlesim_pose;	//Object that stores de turtle actual position

	public:
		turtlesim::Pose goal_pose;	//Object that stores goal position
		turtlesim::Pose aux_pose;

		geometry_msgs::Pose2D feedback_msg;
		geometry_msgs::Twist vel_msg;

		Robot(){ // Default constructor
			
			// Publish in "/turtle1/cmd_vel" topic.
			vel_pub = n.advertise<geometry_msgs::Twist> ("/turtle1/cmd_vel", 10);

			// Publish the current position in feedback topic.
			feedback_pub = n.advertise<geometry_msgs::Pose2D>("feedback", 10);

			// Listen from "directions" topic
			messages_sub = n.subscribe("directions", 10, &Robot::dirCallback, this, ros::TransportHints().tcpNoDelay());

			// Listen from "/turtle1/pose" to know the position of the turtle
			pose_sub = n.subscribe("/turtle1/pose", 10, &Robot::poseCallback, this);
		}

		// Method that moves the robot (turtle) to "goal_pose" given by parameter. Includes a tolerance value to know when to stop
		void to_goal(turtlesim::Pose goal_pose, double tolerance) {
			const double K1 = 1.5;	// Control constants
			const double K2 = 3.5;	// the values where chosen via trial and error

			// 10Hz rate for publishing in "/turtle1/cmd_vel"
			ros::Rate loop_rate(10);

			while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > tolerance) {

				vel_msg.linear.x = K1 * getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
				vel_msg.linear.y = 0;
				vel_msg.linear.z = 0;
				vel_msg.angular.x = 0;
				vel_msg.angular.y = 0;
				vel_msg.angular.z = K2 * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);

				vel_pub.publish(vel_msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
			
			// Once goal is reached, set velocity to 0 and publish to make turtle stop
			vel_msg.linear.x = 0;
			vel_msg.angular.z = 0;
			vel_pub.publish(vel_msg);
			ros::spinOnce();
		}

		// Directions topic callback. Method that handles new goal_pose. This function will be called every time a direction is read.
		void dirCallback(const geometry_msgs::Pose2D::ConstPtr &pose_message) {
			aux_pose.x = pose_message->x;
			aux_pose.y = pose_message->y;
			aux_pose.theta = pose_message->theta;
			q.push(aux_pose);
			ROS_INFO("Robot received xCord: %f \n", aux_pose.x);
			ROS_INFO("Robot received yCord: %f \n", aux_pose.y);
		}

		// Turtlesim current position callback. Method that handles robot current position. This function will be called every time a position is read.
		void poseCallback(const turtlesim::Pose::ConstPtr &pose_message) {
			/* Record current position */
			turtlesim_pose.x = pose_message->x;
			turtlesim_pose.y = pose_message->y;
			turtlesim_pose.theta = pose_message->theta;

			/* Send current position as feedback */
			feedback_msg.x = pose_message->x;
			feedback_msg.y = pose_message->y;
			feedback_msg.theta = pose_message->theta;

			feedback_pub.publish(feedback_msg);
			ros::spinOnce();
		}

};
int main(int argc, char **argv)
{	

	ROS_INFO("Starting robot node \n");
	// Initialize ROS node with name "Robot"
	ros::init(argc, argv, "Robot");

	// Instance of class Robot called "r"
	Robot turtle1;
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {

		// Check if there are goals in queue. If there are, use them first
		if(!q.empty()) {
			turtle1.goal_pose = q.front();
			q.pop();
		}

		// Call method to move robot
		turtle1.to_goal(turtle1.goal_pose, 0.01);

		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}

