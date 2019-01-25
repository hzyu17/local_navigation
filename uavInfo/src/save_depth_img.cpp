#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>
#include <fstream>


using namespace message_filters;
using namespace std;

#define MASK_DIST 6.0

ofstream outFile_depth_data;
ofstream outFile_rgb_data;


void callback(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::ImageConstPtr& rgb)
{
	// Read from sensor msg
	cv_bridge::CvImagePtr rgb_ptr;
	try
	{
		rgb_ptr = cv_bridge::toCvCopy(rgb); //, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e1)
	{
		ROS_ERROR("cv_bridge 1 exception: %s", e1.what());
		return;
	}

	cv_bridge::CvImagePtr depth_ptr;
	try
	{
		depth_ptr = cv_bridge::toCvCopy(depth);
	}
	catch (cv_bridge::Exception& e2)
	{
		ROS_ERROR("cv_bridge 2 exception: %s", e2.what());
		return;
	}

	cv::Mat rgb_img = rgb_ptr->image;
	cv::Mat depth_img = depth_ptr->image;

	// Merge by iteration
	int nr=depth_img.rows;
	int nc=depth_img.cols;

	int channel = rgb_img.channels();

	for(int i=0; i<nr ;i++)
	{
		const float* inDepth = depth_img.ptr<float>(i); // float
		uchar* inRgb = rgb_img.ptr<uchar>(i);

		for(int j=0; j<nc; j++)
		{
			outFile_depth_data << inDepth[j] << ",";
			outFile_rgb_data << inRgb[channel*j] << "," << inRgb[channel*j + 1] << "," << inRgb[channel*j + 2] << ",";
			// if(inDepth[j] != inDepth[j] || inDepth[j] > MASK_DIST)
			// {
			// 	inRgb[channel*j] = 0;
			// 	inRgb[channel*j + 1] = 0;
			// 	inRgb[channel*j + 2] = 0;
			// }			
		}
	}
	outFile_depth_data << std::endl;
	outFile_rgb_data << std::endl;

	cv::imshow("merged", rgb_img);
	cv::waitKey(5);
}

int main(int argc, char** argv)
{

	char tmp[64];
    strftime( tmp, sizeof(tmp), "%Y_%m_%d_%X",localtime(&t) );

    char c_depth_data[100];
    sprintf(c_depth_data,"depth_data_%s.csv", tmp);
    outFile_depth_data.open(c_depth_data, ios::out);

    char c_rgb_data[100];
    sprintf(c_rgb_data,"rgb_data_%s.csv", tmp);
	outFile_rgb_data.open(c_rgb_data, tmp);

	ros::init(argc, argv, "lantern");
	ros::NodeHandle nh;

	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, rgb_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

	return 0;
}
