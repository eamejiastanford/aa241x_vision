/**
 * this file will contain the skeleton / example node for reading from the picam
 * using the C++ library
 *
 * maybe I'll also have it demonstrate the ability to find the AprilTag in the
 * image
 *
 * TODO: figure out how much the latency is and see if it will really cause
 * problems.  I think if they have an estimator running for the position of the
 * truck bed, it will help with any latency??? -> actually probably not...
 */

#include <iostream>

#include <ros/ros.h>

#include <raspicam/raspicam_cv.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag36h11.h>
}

using namespace std;



// TODO: remove aa241x from the node name
class VisionNode {

public:

	VisionNode(int frame_width, int frame_height, bool publish_image);


	// main loop
	int run();


private:

	// node handler
	ros::NodeHandle _nh;
	image_transport::ImageTransport _it;


	// settings, etc
	int _frame_width;		// image width to use in [px]
	int _frame_height;		// image height to use in [px]
	bool _publish_image;	// true if the image data should be published

	// camera stuff
	raspicam::RaspiCam_Cv _camera;	// TODO: use the correct class name here

	// april tag stuff
	//APDetector _detector;	// TODO: use the correct class name here

	// subscribers
	// TODO: figure out what subscriber may be desired


	// publishers
	ros::Publisher _tag_relative_position_pub;	// the relative position vector to the truck
	ros::Publisher _tag_details_pub;			// the raw tag details (for debugging)
	image_transport::Publisher _image_pub;

	// TODO: add a publisher of the image frame -> for the display node

	// callbacks
	// TODO: figure out what might be needed


	// helper functions
	// TODO: any helper functions here

    //Low Pass Filter variables
    float x_raw=0.0;
    float y_raw=0.0;
    float z_raw=0.0;
    float x_est=0.0;
    float y_est=0.0;
    float z_est=0.0;

};


VisionNode::VisionNode(int frame_width, int frame_height, bool publish_image) :
_frame_width(frame_width),
_frame_height(frame_height),
_it(_nh)
{
	_image_pub = _it.advertise("image", 1);

    // configure the camera
    _camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    _camera.set(cv::CAP_PROP_FRAME_WIDTH, _frame_width);
	_camera.set(cv::CAP_PROP_FRAME_HEIGHT, _frame_height);
	_camera.set(cv::CAP_PROP_FORMAT, CV_8UC1);

}


int VisionNode::run() {

	// TODO: set up the tag detector
	// TODO: determine if the setup is best in the constructor or here
	// TODO: try another one of the apriltag libraries???

    // open the camera
    ROS_INFO("opening camera");
	if (!_camera.open()) {
        ROS_ERROR("Error opening the camera");
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }

    // apriltag handling setup
	apriltag_family_t *tf = tag16h5_create();
    //apriltag_family_t *tf = tag36h11_create();
	
	apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 3.0;
    td->quad_sigma = 0.0;
    td->refine_edges = 0;
    //td->decode_sharpening = 0.25;

	ros::Time image_time; 	// timestamp of when the image was grabbed
	cv::Mat frame_gray;		// the image in grayscale


	// loop while the node should be running
	while (ros::ok()) {

		// TODO: probably want to have a mission state machine running so that
		// this processing is not occuring throughout the entire flight!!!

		// TODO: grab the frame from the camera
        _camera.grab();
		_camera.retrieve(frame_gray);
		image_time = ros::Time::now();


        // Make an image_u8_t header for the Mat data to pass over to the april
        // tag detector
        image_u8_t im = { .width = frame_gray.cols,
            .height = frame_gray.rows,
            .stride = frame_gray.cols,
            .buf = frame_gray.data
        };
		
		zarray_t *detections = apriltag_detector_detect(td, &im);
        ROS_INFO("%d tags detected", zarray_size(detections));


		// TODO: compute the relative vector between me and the AprilTag
		// NOTE: this will be left entirely to the students I think
		// NOTE: I feel like it would be too easy to just give them that code

		// TODO: publish the detection information


		if (_publish_image) {
            // Draw detection outlines
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                // Define camera parameters struct
                apriltag_detection_info_t info;
                info.det = det;
                //info.tagsize = 0.16;
                info.tagsize = 0.09;
                info.fx = 1.0007824174077226e+03;
                info.fy = 1.0007824174077226e+03;
                info.cx = 640.;
                info.cy = 360.;

                // Estimate pose
                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info, &pose);
                matd_t* t = pose.t;
                double *tData = t->data;
                x_raw = tData[0];
                y_raw = tData[1];
                z_raw = tData[2];
                //Low Pass Filter Parameters
                float alpha = 0.2;
                float beta = 0.05;
                //Low Pass Filter
                x_est = alpha*x_raw +(1-alpha)*x_est;
                y_est = alpha*y_raw +(1-alpha)*y_est;
                z_est = beta*z_raw +(1-beta)*z_est;
                double distance = sqrt(x_est*x_est + y_est*y_est + z_est*z_est);
                cout << "this is the x: " << x_est << endl;
                cout << "this is the y: " << y_est << endl;
                cout << "this is the z: " << z_est << endl;
                cout << "this is the distance: " << distance << endl;

                line(frame_gray, cv::Point(det->p[0][0], det->p[0][1]),
                         cv::Point(det->p[1][0], det->p[1][1]),
                         cv::Scalar(0, 0xff, 0), 2);
                line(frame_gray, cv::Point(det->p[0][0], det->p[0][1]),
                         cv::Point(det->p[3][0], det->p[3][1]),
                         cv::Scalar(0, 0, 0xff), 2);
                line(frame_gray, cv::Point(det->p[1][0], det->p[1][1]),
                         cv::Point(det->p[2][0], det->p[2][1]),
                         cv::Scalar(0xff, 0, 0), 2);
                line(frame_gray, cv::Point(det->p[2][0], det->p[2][1]),
                         cv::Point(det->p[3][0], det->p[3][1]),
                         cv::Scalar(0xff, 0, 0), 2);

                std::stringstream ss;
                ss << det->id;
                cv::String text = ss.str();
                int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
                double fontscale = 1.0;
                int baseline;
                cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                                &baseline);
                cv::putText(frame_gray, text, cv::Point(det->c[0]-textsize.width/2,
                                           det->c[1]+textsize.height/2),
                        fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
            }

			// publish the annotated image
			std_msgs::Header header;
			header.stamp = image_time;
			sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", frame_gray).toImageMsg();
			_image_pub.publish(img_msg);
		}

        // clean up the detections
        zarray_destroy(detections);


		// TODO: if there are no subscribers, I'm not sure this is needed...
		// though I wonder if it is needed from a time sync point of view
		ros::spinOnce();
	}

    // need to stop the camera
    ROS_INFO("stopping camera");
	_camera.release();

    // remove apriltag stuff
	apriltag_detector_destroy(td);
	tag16h5_destroy(tf);
    //tag36h11_destroy(tf);

}




int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "vision_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings
	int frame_width, frame_height;
	bool publish_image;
	private_nh.param("frame_width", frame_width, 640);
	private_nh.param("frame_height", frame_height, 512);
	private_nh.param("publish_image", publish_image, false);

	// create the node
	VisionNode node(frame_width, frame_height, publish_image);

	// run the node
	return node.run();
}
