#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    
    cv_bridge::CvImagePtr cvimage = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::imshow("Image window",cvimage->image);
    std::string time = std::to_string(msg->header.stamp.sec + (double)(msg->header.stamp.nsec) * 1e-9);
    std::string savepath ="/home/fwt/Pictures/" + time;
    savepath.append(".jpg");
    cv::imwrite(savepath, cvimage->image);
    cv::waitKey(1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "my_image_node");
    ros::NodeHandle n;
    ros::Subscriber sub =n.subscribe("/mynteye/left/image_raw",1,imageCallback);
    ros::spin();
    return 0;
}