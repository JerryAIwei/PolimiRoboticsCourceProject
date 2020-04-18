#include "ros/ros.h"
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include "custom_messages/Status.h"
#include "distance_service/ComputeDistance.h"

#include <sstream>
#include <memory>

class DistancePublisher{
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
    nav_msgs::Odometry> MySyncPolicy;
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    message_filters::Subscriber<nav_msgs::Odometry> car_sub,obs_sub;
    std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>>
            syncPtr;
    distance_service::ComputeDistance srv;
    ros::ServiceClient client;
    double safeDistance;
    double crashDistance;

    void callback(const nav_msgs::OdometryConstPtr& msg1, const nav_msgs::OdometryConstPtr& msg2)
    {
        ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)",
                msg1->pose.pose.position.x,msg1->pose.pose.position.y,msg1->pose.pose.position.z,
                msg2->pose.pose.position.x,msg2->pose.pose.position.y,msg1->pose.pose.position.z);
        srv.request.carX = msg1->pose.pose.position.x;
        srv.request.carY = msg1->pose.pose.position.y;
        srv.request.carZ = msg1->pose.pose.position.z;
        srv.request.obsX = msg2->pose.pose.position.x;
        srv.request.obsY = msg2->pose.pose.position.y;
        srv.request.obsZ = msg2->pose.pose.position.z;
        if(client.call(srv)){
            double distance = srv.response.distance;
            custom_messages::Status msg;
            msg.distance = distance;
            if(distance>safeDistance) msg.status = "Safe";
            else if(distance>safeDistance<crashDistance)
                msg.status = "Crash";
            else msg.status = "Unsafe";
            pub.publish(msg);
        }
    }
public:
    DistancePublisher(double safeDistance = 5.0, double crashDistance = 1.0)
    :safeDistance(safeDistance),crashDistance(crashDistance)
    {
        pub = n.advertise<custom_messages::Status>("status", 1000);
        client = n.serviceClient<distance_service::ComputeDistance>("compute_distance");
        car_sub.subscribe(n, "topic1",5);
        obs_sub.subscribe(n, "topic2",5);
        syncPtr = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), car_sub, obs_sub);
        syncPtr->registerCallback(boost::bind(&DistancePublisher::callback,this, _1, _2));
    }
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "check status");
    DistancePublisher distancePublisher;

  	ros::spin();

  	return 0;
}
