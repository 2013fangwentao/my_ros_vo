#include <ros/ros.h>
#include <service_demo/Greeting.h>
 int main(int argc, char *argv[])
 {
     ros::init(argc, argv, "greeting_client");
     ros::NodeHandle nh;
     ros::ServiceClient client = nh.serviceClient<service_demo::Greeting>("greetings");

     service_demo::Greeting srv;
     srv.request.name = "HAN";
     srv.request.age = 20;

     if (client.call(srv))
     {
         ROS_INFO("FeedBack from server %s\n",srv.response.feedback.c_str());
     }
     else
     {
         ROS_ERROR("Failed to call service greetings\n");
         return 1;
     }

     return 0;
 }
