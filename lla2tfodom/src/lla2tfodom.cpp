#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <string>

class Lla2tfOdomPublisher
{
private:
  ros::NodeHandle n;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster br;
  ros::Subscriber sub;
  std::string msgPath;
  std::string name;

  //position
  float xEast = 0.0;
  float yNorth = 0.0;
  float zUp = 0.0;

  float lastx = 0.0;
  float lasty = 0.0;
  float lastz = 0.0;

  //time
  ros::Time current_time, last_time;
  // fixed position
  float latitude_init;
  float longitude_init;
  float h0;
  void lla2neu(const sensor_msgs::NavSatFix::ConstPtr &msg)
  {
    ROS_INFO("Input position: [%f,%f, %f]", msg->latitude, msg->longitude, msg->altitude);

    // fixed values

    constexpr double a = 6378137;
    constexpr double b = 6356752.3142;
    constexpr double f = (a - b) / a;
    constexpr double e_sq = f * (2 - f);
    constexpr float deg_to_rad = 0.0174533;

    // input data from msg
    float latitude = msg->latitude;
    float longitude = msg->longitude;
    float h = msg->altitude;

    //lla to ecef
    float lamb = deg_to_rad * (latitude);
    float phi = deg_to_rad * (longitude);
    float s = sin(lamb);
    float N = a / sqrt(1 - e_sq * s * s);

    float sin_lambda = sin(lamb);
    float cos_lambda = cos(lamb);
    float sin_phi = sin(phi);
    float cos_phi = cos(phi);

    float x = (h + N) * cos_lambda * cos_phi;
    float y = (h + N) * cos_lambda * sin_phi;
    float z = (h + (1 - e_sq) * N) * sin_lambda;

    //ROS_INFO("ECEF position: [%f,%f, %f]", x, y,z);

    // ecef to enu

    lamb = deg_to_rad * (latitude_init);
    phi = deg_to_rad * (longitude_init);
    s = sin(lamb);
    N = a / sqrt(1 - e_sq * s * s);

    sin_lambda = sin(lamb);
    cos_lambda = cos(lamb);
    sin_phi = sin(phi);
    cos_phi = cos(phi);

    float x0 = (h0 + N) * cos_lambda * cos_phi;
    float y0 = (h0 + N) * cos_lambda * sin_phi;
    float z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

    float xd = x - x0;
    float yd = y - y0;
    float zd = z - z0;

    xEast = -sin_phi * xd + cos_phi * yd;
    yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    ROS_INFO("ENU position: [%f,%f, %f]", xEast, yNorth, zUp);
  }

public:
  Lla2tfOdomPublisher(std::string msgPath = "/swiftnav/front/gps_pose", std::string name = "car_odom", float latitude_init = 45.6311926152, float longitude_init = 9.2947495255, float h0 = 231.506675163)
      : msgPath(msgPath), name(name), latitude_init(latitude_init), longitude_init(longitude_init), h0(h0)
  {
    sub = n.subscribe(msgPath, 1000, &Lla2tfOdomPublisher::callback, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    current_time = ros::Time::now();
    last_time = ros::Time::now();
  }

  void callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
  {

    lla2neu(msg);
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    //TF
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(xEast, yNorth, zUp));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));

    //Odom
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = name;

    //set the position
    odom.pose.pose.position.x = xEast;
    odom.pose.pose.position.y = yNorth;
    odom.pose.pose.position.z = zUp;

    odom.twist.twist.linear.x = (xEast - lastx) / dt;
    odom.twist.twist.linear.y = (yNorth - lasty) / dt;
    odom.twist.twist.linear.z = (zUp - lastz) / dt;
    ROS_INFO("velocity: [%f,%f, %f]", odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
    //publish the message
    odom_pub.publish(odom);
    lastx = xEast;
    lasty = yNorth;
    lastz = zUp;
    last_time = current_time;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Lla2odomPublisher");
  Lla2tfOdomPublisher lla2tfOdomPublisher;
  ros::spin();
  return 0;
}
