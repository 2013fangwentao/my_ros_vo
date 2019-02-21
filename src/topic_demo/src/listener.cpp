#include <ros/ros.h>
#include <topic_demo/gps.h>
#include <std_msgs/Float32.h>

void gpsCallback(const topic_demo::gps::ConstPtr &msg)
{
    std_msgs::Float32 distance;
    distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
    ROS_INFO("Listener: Distance to origin =%f,state = %s",distance.data,msg->state.c_str());
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub =n.subscribe("gps_info",1,gpsCallback);
    ros::spin();
    return 0;
}