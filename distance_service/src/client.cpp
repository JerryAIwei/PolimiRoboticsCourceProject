#include "ros/ros.h"
#include "distance_service/ComputeDistance.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_distance_client");
  if (argc != 7)
  {
    ROS_INFO("usage: compute_distance_client x1 y1 z1 x2 y2 z2");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<distance_service::ComputeDistance>("compute_distance");
  distance_service::ComputeDistance srv;
  srv.request.carX = atoll(argv[1]);
  srv.request.carY = atoll(argv[2]);
  srv.request.carZ = atoll(argv[3]);
  srv.request.obsX = atoll(argv[4]);
  srv.request.obsY = atoll(argv[5]);
  srv.request.obsZ = atoll(argv[6]);
  if (client.call(srv))
  {
    ROS_INFO("distance: %f", (double)srv.response.distance);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
