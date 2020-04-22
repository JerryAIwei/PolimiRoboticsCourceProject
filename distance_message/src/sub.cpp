#include "ros/ros.h"
#include "std_msgs/String.h"
#include "distance_message/Status.h"

void chatterCallback(const distance_message::Status::ConstPtr& msg){
  ROS_INFO("Distance: %f, status: %s.", msg->distance, msg->status.c_str());
}

int main(int argc, char **argv){
  	
	ros::init(argc, argv, "status_listener");

	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("/status", 1000, chatterCallback);

  	ros::spin();

  return 0;
}


