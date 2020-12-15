#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include <string>
#include <regex>

using namespace std;
#define WALL_BORDER_SUP 11	// Max wall border position. 
#define WALL_BORDER_INF 0	// Min wall border position.

// Auxiliary functions to validate keyboard goal input

bool is_float (string input_string) {
	string float_regex = "[+-]?([0-9]*[.])?[0-9]+";
	return regex_match (input_string, regex(float_regex));
}

bool is_in_bound (float number, float min, float max){
	return number > min && number <= max;
}

float process_input_coord (){
	float aux_float = 0;
	string input_string = "";
	bool valid_float = false;
	bool first_pass = true;

	while (!valid_float){

		if (!first_pass){
			cout << "Please enter a valid coord: ";			
		}

		cin >> input_string;
		if (input_string == "q"){
			ROS_INFO("Exiting give_dirs node \n");
			exit(0);
		}

		if (is_float(input_string) && is_in_bound(stof(input_string), WALL_BORDER_INF, WALL_BORDER_SUP)){
			valid_float = true;
		}
		first_pass = false;
	}

	aux_float = stof(input_string);

	return aux_float;

}



// The function of this node is to take the user input ([x,y] goal), save it in the corresponding geometry_msg and publish it in a topic so that the controller node can listen to it and controll the robot.

int main(int argc, char **argv){

	ROS_INFO("Starting give directions node \n");
	// Initialize ROS node with name "give_dirs"
	ros::init(argc, argv, "give_dirs");

	ros::NodeHandle n;

	// Advertise that we will publish to "directions" topic
	ros::Publisher dirs_pub = n.advertise<geometry_msgs::Pose2D>("directions", 10);

	// Variable where goal is saved to later be sent to the topic directions
	geometry_msgs::Pose2D goal_pose;

	// 2 Hz sample rates for input goal
	ros::Rate loop_rate(2);

	cout << "Enter 'q' at any time to exit" << endl;

	while (ros::ok()){

		// Save goal in variable through std input. Validate that it is of float type

		cout << "Enter X coord: ";
		goal_pose.x = process_input_coord();

		cout << "Enter Y coord: ";
		goal_pose.y = process_input_coord();

		goal_pose.theta = 0;

		// Publish goal in "directions" topic
		dirs_pub.publish(goal_pose);
		
		ros::spinOnce();

		// Sleep for Rate
		loop_rate.sleep();

	}
  return 0;
}

