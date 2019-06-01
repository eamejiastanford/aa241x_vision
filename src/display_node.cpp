/**
 * NOTE: this node should only be used for testing, this should never actually
 * be run in flight!!!
 */


#include <iostream>
#include <string>
#include <vector>
#include <chrono>	// std:c:chrono::system_clock
#include <ctime>	// std::time_t, struct std::tm, std::localtime
#include <iomanip>  // std::put_time

// system includes
#include <sys/stat.h>	// directory handling
#include <sys/types.h>	// directory handling
#include <sys/time.h>	// time functions


// specific library stuff
#include <opencv2/opencv.hpp>	// opencv
#include <ros/ros.h> 			// ros

// image handling
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;
using namespace cv;


class DisplayNode {

public:

	DisplayNode(int frame_width, int frame_height);

	int run();

private:

	// node handler
	ros::NodeHandle _nh;
	image_transport::ImageTransport _it;

	// settings
	int _frame_width;
	int _frame_height;

	// subscriber
	image_transport::Subscriber _image_sub;

	// callbacks
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);


};


DisplayNode::DisplayNode(int frame_width, int frame_height) :
_frame_width(frame_width),
_frame_height(frame_height),
_it(_nh)
{
	// subscribe to the image
	_image_sub = _it.subscribe("image", 1, &DisplayNode::imageCallback, this);
}


void DisplayNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	// wrap the decoding in a try catch to handle the error in the encoding
	try {
		// get the image
		cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;
		cv::imshow("debug", image);
		//cv::waitKey(10);

	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
}


int DisplayNode::run() {

	// create the window
	// TODO: set this window size to the known size of the image!!!
	cv::namedWindow("debug", cv::WINDOW_NORMAL);
	cv::resizeWindow("debug", _frame_width, _frame_height);
	cv::startWindowThread();

	// spin listening for the callbacks
	ros::spin();

	// destroy the window when the node is stopped
	cv::destroyWindow("debug");
}



int main(int argc, char **argv) {

	// define the node
	ros::init(argc, argv, "display_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings
	int frame_width, frame_height;

	private_nh.param("frame_width", frame_width, 640);
	private_nh.param("frame_height", frame_height, 512);

	// create the node
	DisplayNode node(frame_width, frame_height);

	// run the node
	return node.run();

}
