#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
using namespace  std;

int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "fwt_publish_image");
	ros::NodeHandle nh;
	ros::Rate loop_rate(15);
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub = it.advertise("local/camera/image_raw", 100);
	image_transport::Publisher depth_pub = it.advertise("local/camera/depth_raw",100);
	if (argc!=2)
	{
		ROS_INFO("argv error\n");
		return 0;
	}
	string dirname = argv[1];
	ifstream fin((dirname + "/associate.txt").c_str());
    if (!fin)
    {
        ROS_INFO("please generate the associate file called associate.txt!\n");
        return 0;
    }

	/*---------------------------------------------------------------------*/
    vector<string> rgb_files, depth_files;
	std::vector<double> rgb_times,depth_times;	
    while (!fin.eof())
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
		rgb_times.push_back(atof(rgb_time.c_str()));
		depth_times.push_back(atof(depth_time.c_str()));
        rgb_files.push_back(dirname + "/" + rgb_file);
        depth_files.push_back(dirname + "/" + depth_file);
        if (fin.good() == false)
            break;
    }
	/*---------------------------------------------------------------------*/
	int count = 0;
	int size = rgb_files.size();
	while (ros::ok()&&count < size)
	{
		cv::Mat image = cv::imread(rgb_files.at(count), CV_LOAD_IMAGE_COLOR);
		cv::Mat depth = cv::imread(depth_files.at(count), -1);
		


		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
		sensor_msgs::ImagePtr msgdepth = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth).toImageMsg();

		image_pub.publish(msg);
		depth_pub.publish(msgdepth);
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}

	return 0;
}