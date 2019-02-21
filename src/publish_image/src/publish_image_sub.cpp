#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <memory.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "image_transport/image_transport.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "Eigen/Geometry"
#include "publish_image/fConfig.h"
#include "publish_image/fVisualOdometry.h"
#include "publish_image/fCamera.h"



using namespace std;
using namespace myslam;


deque<cv::Mat> all_image;
deque<double>  images_time;
deque<cv::Mat> all_depth;
deque<double>  depths_time;
mutex          mgetimage;
mutex          mgetdepth;
const chrono::milliseconds microsunit(1);
ros::Publisher pub_path;
fVisualOdometry::Ptr vo=nullptr;
Camera::Ptr camera=nullptr;
nav_msgs::Path path_;
std::atomic<int> run(1);
// void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
// {
// 	cv_bridge::CvImageConstPtr cvimage = cv_bridge::toCvShare(msg, msg->encoding);

// 	myslam::fFrame::Ptr pFrame = myslam::fFrame::createFrame();
// 	pFrame->camera_ = camera;
// 	pFrame->color_ = cvimage->image;
// 	pFrame->time_stamp_ = (msg->header.stamp.sec + (double)(msg->header.stamp.nsec) * 1e-9);
// LEAP:	
// 	int size_of_depth = all_image.size();
// 	if(size_of_depth==0)
// 	{
// 		ROS_INFO("No depth picture");
// 		this_thread::sleep_for(microsunit*20);
// 		goto LEAP;
// 	}
// 	m_getimage.lock();
// Leap2:
// 	double times_depth = images_time.front();
// 	if(times_depth > (pFrame->time_stamp_+0.033))
// 	{
// 		m_getimage.unlock();
// 		return ;
// 	}
// 	else if(times_depth < (pFrame->time_stamp_-0.033))
// 	{
// 		if(images_time.size()>2)
// 		{
// 			images_time.pop_front();
// 			all_image.pop_front();
// 			goto Leap2;
// 		}
// 		else
// 		{
// 			m_getimage.unlock();
// 			return ;
// 		}
// 	}
// 	else
// 	{
// 		pFrame->depth_ = all_image.front();
// 		images_time.pop_front();
// 		all_image.pop_front();
// 	}
// 	m_getimage.unlock();
// 	vo->addFrame ( pFrame );
// 	SE3 Tcw = pFrame->T_c_w_.inverse();		
// 	Eigen::Quaternion<double> Q {Tcw.rotation_matrix()};
// 	Eigen::Vector3d P = Tcw.translation();
// 	geometry_msgs::PoseStamped pose_stamped;
// 	pose_stamped.header.stamp= msg->header.stamp;
// 	pose_stamped.header.frame_id = "world";
// 	pose_stamped.pose.position.x = P.x();
// 	pose_stamped.pose.position.y = P.y();
// 	pose_stamped.pose.position.z = P.z();
// 	pose_stamped.pose.orientation.x = Q.x();
// 	pose_stamped.pose.orientation.y = Q.y();
// 	pose_stamped.pose.orientation.z = Q.z();
// 	pose_stamped.pose.orientation.w = Q.w();
// 	path_.header = pose_stamped.header;
// 	path_.poses.push_back(pose_stamped);
// 	pub_path.publish(path_);
// }

void chatterCallbackdepth(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cvimage = cv_bridge::toCvShare(msg, msg->encoding);
	mgetdepth.lock();
	all_depth.push_back(cvimage->image);
	depths_time.push_back((msg->header.stamp.sec + (double)(msg->header.stamp.nsec) * 1e-9));
	mgetdepth.unlock();
	ROS_INFO("Get depth Image....\n");	
}


void chatterCallbackimage(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cvimage = cv_bridge::toCvShare(msg, msg->encoding);
	mgetimage.lock();
	all_image.push_back(cvimage->image);
	images_time.push_back((msg->header.stamp.sec + (double)(msg->header.stamp.nsec) * 1e-9));
	mgetimage.unlock();
	ROS_INFO("Get color Image %d ....\n",msg->header.stamp.sec);	
}

// void chatterCallback2(const sensor_msgs::ImageConstPtr& msg)
// {
// 	cv_bridge::CvImageConstPtr cvimage = cv_bridge::toCvShare(msg, msg->encoding);
// 	ROS_INFO("Get one Image....\n");
// 	m_getimage.lock();
// 	ROS_INFO("Get one Image2....\n");
//     all_image.push_back(cvimage->image);	
//  	images_time.push_back(msg->header.stamp.sec + (double)(msg->header.stamp.nsec) * 1e-9);
// 	while (all_image.size() > 30)
// 	{
// 		all_image.pop_front();
// 		images_time.pop_front();
// 	}
// 	m_getimage.unlock();
// 	ROS_INFO("Get one Image3....\n");	
// }

// void processing(const char* config)
// {

// 	while(true)
// 	{
// 		m_getimage.lock();
// 		int images_size = all_image.size();
// 		m_getimage.unlock();
// 		if (images_size==0)
// 		{
// 			this_thread::sleep_for(microsunit*20);
// 			continue;
// 		}
// 		m_getimage.lock();
// 		cv::Mat image = all_image.front();
// 		double time_stamp = images_time.front();
// 		all_image.pop_front();
// 		images_time.pop_front();
// 		m_getimage.unlock();
// 		myslam::fFrame::Ptr pFrame = myslam::fFrame::createFrame();
//         pFrame->camera_ = camera;
//         pFrame->color_ = image;
//         pFrame->time_stamp_ = time_stamp;
// 		vo->addFrame ( pFrame );
// 		SE3 Tcw = pFrame->T_c_w_.inverse();		
// 		Eigen::Quaternion<double> Q {Tcw.rotation_matrix()};
//     	Eigen::Vector3d P = Tcw.translation();
// 		geometry_msgs::PoseStamped pose_stamped;
//     	pose_stamped.header.stamp.sec = (int)time_stamp;
//     	pose_stamped.header.stamp.nsec = (time_stamp-(int)time_stamp)*1e9;
//     	pose_stamped.header.frame_id = "world";
//     	pose_stamped.pose.position.x = P.x();
//     	pose_stamped.pose.position.y = P.y();
//     	pose_stamped.pose.position.z = P.z();
//     	pose_stamped.pose.orientation.x = Q.x();
//     	pose_stamped.pose.orientation.y = Q.y();
//     	pose_stamped.pose.orientation.z = Q.z();
//     	pose_stamped.pose.orientation.w = Q.w();
// 		path_.header = pose_stamped.header;
// 		path_.poses.push_back(pose_stamped);
// 		pub_path.publish(path_);
// 	}
// }

void processing()
{
	while(run)
	{
		int sized = all_depth.size();
		int sizec = all_image.size();
		ROS_INFO("depth size: %d color size: %d\n",sized,sizec);
		if(sized==0||sizec==0)		
		{
			this_thread::sleep_for(microsunit*100);
			continue;
		}

		mgetdepth.lock();
		cv::Mat depth = all_depth.front();
		all_depth.pop_front();
		mgetdepth.unlock();

		mgetimage.lock();
		cv::Mat color = all_image.front();
		all_image.pop_front();
		double time = images_time.front();
		images_time.pop_front();
		mgetimage.unlock();
		// cv::imshow("image",color);
		cv::imshow("depth",color);
		std::string savepath ="/home/fwt/Pictures/color/" +std::to_string(time) + ".jpg";
		cv::imwrite(savepath, color);
		cv::waitKey(1);
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "myvo_exe");
	fConfig::setParameterFile(argv[1]);
	camera = std::make_shared<Camera>();
	vo = std::make_shared<fVisualOdometry>();

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	pub_path = nh.advertise<nav_msgs::Path>("/fwt_path/path",1000);
	image_transport::Subscriber image_sub = it.subscribe("/local/camera/image_raw", 30, chatterCallbackimage);
	image_transport::Subscriber depth_sub = it.subscribe("/local/camera/depth_raw", 30, chatterCallbackdepth);
	std::thread th{processing};
	ros::spin();
	run=0;
	th.join();
	return 0;
}