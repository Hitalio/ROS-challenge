#include "ros/ros.h"					// ros library
#include "std_msgs/String.h"	// basic string, it will change to coordinates
#include <sstream>						// c++ class for string streams

// Given the two examples from the ROS tutorial in which you create a listener and a publisher, i will create a class that does both. 

class talker_listener {

	private:
		ros::NodeHandle n;		// NodeHandle Object from ros library
		ros::Publisher pub;		// Publisher object
		ros::Subscriber sub;	// Subscriber object

	public:
		talker_listener() {
			sub = n.subscribe("chatter", 1000, &talker_listener::callback, this);
			pub = n.advertise<std_msgs::String>("feedback", 1000);
	  }

		void callback(const std_msgs::String::ConstPtr &msg) {
			std_msgs::String pub_str;
			std::stringstream ss;
			ROS_INFO("Talker_listener heard: [%s]", msg->data.c_str());   
			ss << "position set to: " << msg->data.c_str();
			pub_str.data = ss.str();

			std::cout << pub_str.data.c_str() << std::endl;

			pub.publish(pub_str);
			ros::spinOnce();
		}
};

int main (int argc, char **argv){

	ros::init(argc, argv, "talker_listener");
	talker_listener TLinstance;
	ros::spin();

	return 0;
}
