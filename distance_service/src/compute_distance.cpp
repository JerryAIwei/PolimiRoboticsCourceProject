#include "ros/ros.h"
#include "distance_service/ComputeDistance.h"

#include <math.h>
bool distance(distance_service::ComputeDistance::Request  &req,
         distance_service::ComputeDistance::Response &res)
{
  res.distance = sqrt(pow(req.carX-req.obsX,2)+pow(req.carY-req.obsY,2)+pow(req.carY-req.obsY,2));
  ROS_INFO("request: carX=%f, carY=%f, carZ=%f,", (double)req.carX, (double)req.carY,(double)req.carZ);
  ROS_INFO("request: obsX=%f, obsY=%f, obsZ=%f,", (double)req.obsX, (double)req.obsY,(double)req.obsZ);
  ROS_INFO("sending back response: [%f]", (double)res.distance);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_distance_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("compute_distance", distance);
  ROS_INFO("Ready to compute distance.");
  ros::spin();

  return 0;
}
