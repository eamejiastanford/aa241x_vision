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
#include <string>

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

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

const std::string MINISEARCH = "MINISEARCH";
const std::string Hover_Search = "Hover_Search";
const std::string CAMERA_TEST = "CAMERA_TEST";


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

	// settings, etc
	int _frame_width;		// image width to use in [px]
	int _frame_height;		// image height to use in [px]

	// camera stuff
	raspicam::RaspiCam_Cv _camera;	// TODO: use the correct class name here

        // Drone state
        std::string _STATE;

        // Tag relative position (camera frame) from the drone
        float _xr = 0.0;
        float _yr = 0.0;
        float _zr = 0.0;

        // Tag orientation
        float _R33 = 0.0;

        // Tag id
        int id = -1;

        // Subscribers
        ros::Subscriber _droneState_sub;

        // Messages
        std_msgs::Float64 _tag_relative_x_msg;
        std_msgs::Float64 _tag_relative_y_msg;
        std_msgs::Float64 _tag_relative_z_msg;
        std_msgs::Bool _tag_found_msg;
        std_msgs::String _droneState_msg;
        std_msgs::Float64 _R33_msg;
        std_msgs::Int64 _detected_tag_msg;

	// publishers
        ros::Publisher _tag_relative_x_pub;	// the relative position vector to the truck
        ros::Publisher _tag_relative_y_pub;	// the relative position vector to the truck
        ros::Publisher _tag_relative_z_pub;	// the relative position vector to the truck
        ros::Publisher _tag_found_pub;
        ros::Publisher _tag_details_pub;	// the raw tag details (for debugging)
        ros::Publisher _R33_pub;
        ros::Publisher _detected_tag_pub;

        // Callbacks
        void droneStateCallback(const std_msgs::String::ConstPtr& msg);

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
_frame_height(frame_height)
{
    // Publishers
    _tag_relative_x_pub = _nh.advertise<std_msgs::Float64>("tag_rel_x", 10);
    _tag_relative_y_pub = _nh.advertise<std_msgs::Float64>("tag_rel_y", 10);
    _tag_relative_z_pub = _nh.advertise<std_msgs::Float64>("tag_rel_z", 10);
    _tag_found_pub      = _nh.advertise<std_msgs::Bool>("tagFound",10);
    _R33_pub = _nh.advertise<std_msgs::Float64>("R33",10);
    _detected_tag_pub = _nh.advertise<std_msgs::Int64>("detected_tag", 10);

    // Subscribers
    _droneState_sub = _nh.subscribe<std_msgs::String>("drone_state", 10, &VisionNode::droneStateCallback, this);

    // configure the camera
    _camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    _camera.set(cv::CAP_PROP_FRAME_WIDTH, _frame_width);
    _camera.set(cv::CAP_PROP_FRAME_HEIGHT, _frame_height);
    _camera.set(cv::CAP_PROP_FORMAT, CV_8UC1);

}

void VisionNode::droneStateCallback(const std_msgs::String::ConstPtr& msg) {
        // save the state locally to be used in the main loop
        _STATE = msg->data;
}


int VisionNode::run() {

    // open the camera
    ROS_INFO("opening camera");
    if (!_camera.open()) {
        ROS_ERROR("Error opening the camera");
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }

    // apriltag handling setup
    apriltag_family_t *tf_16 = tag16h5_create();
    apriltag_family_t *tf_36 = tag36h11_create();

    apriltag_detector_t *td_16 = apriltag_detector_create();
    apriltag_detector_t *td_36 = apriltag_detector_create();
    apriltag_detector_add_family(td_16, tf_16);
    apriltag_detector_add_family(td_36, tf_36);
    td_16->quad_decimate = 1.0;//3.0;
    td_16->quad_sigma = 0.25;//0.0;
    td_16->refine_edges = 0;
    td_36->quad_decimate = 1.0;//3.0;
    td_36->quad_sigma = 0.25;//0.0;
    td_36->refine_edges = 0;
    //td->decode_sharpening = 0.25;

    cv::Mat frame_gray;		// the image in grayscale

    // loop while the node should be running
    while (ros::ok()) {

        // Check if we should use the camera..
        //if (_STATE == Hover_Search || _STATE == CAMERA_TEST || _STATE == MINISEARCH) {
        if(true) {

            // Take a picture
            _camera.grab();
            _camera.retrieve(frame_gray);

            // Create an image to do detection
            image_u8_t im = { .width = frame_gray.cols,
                        .height = frame_gray.rows,
                        .stride = frame_gray.cols,
                        .buf = frame_gray.data
            };

            // Detect apriltags
            zarray_t *detections_16 = apriltag_detector_detect(td_16, &im);
            zarray_t *detections_36 = apriltag_detector_detect(td_36, &im);
            ROS_INFO("%d tags detected", zarray_size(detections_16));
            ROS_INFO("%d tags detected", zarray_size(detections_36));

            // Check if we found april tags
            if (zarray_size(detections_16) > 0){
                _tag_found_msg.data = true;
            }
            // Check if we found april tags
            else if (zarray_size(detections_36) > 0){
                _tag_found_msg.data = true;
            }
            else {
                _tag_found_msg.data = false;
            }

            // Loop through list of detections
            for (int i = 0; i < zarray_size(detections_16); i++) {
                apriltag_detection_t *det;
                zarray_get(detections_16, i, &det);

                // Define camera parameters struct
                apriltag_detection_info_t info;
                info.det = det;
                // ROSBAG DET:

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
                matd_t* R = pose.R;
                double *tData = t->data;
                double *RData = R->data;
                x_raw = tData[0];
                y_raw = tData[1];
                z_raw = tData[2];
                _R33 = RData[8];
                id = det->id;

                // Check if this is really an april tag based on detected z-axis orientation
                if (_R33 >= 0.95 && (id == 0 || id == 1 || id == 2 || id == 3) ) {
                    //Low Pass Filter Parameters
                    float alpha = 0.2;
                    float beta = 0.05;
                    //Low Pass Filter
                    x_est = alpha*x_raw +(1-alpha)*x_est;
                    y_est = alpha*y_raw +(1-alpha)*y_est;
                    z_est = beta*z_raw +(1-beta)*z_est;
                    double distance = sqrt(x_est*x_est + y_est*y_est + z_est*z_est);

                    // Add offsets from relative location of camera from drone center
                    _xr = x_est + 0.2;
                    _yr = y_est + 0.0;
                    _zr = z_est + 0.0;

                    // Publish the vector from the drone to the april tag (camera frame)
                    _tag_relative_x_msg.data = _xr;
                    _tag_relative_y_msg.data = _yr;
                    _tag_relative_z_msg.data = _zr;
                    _R33_msg.data = _R33;
                    _detected_tag_msg.data = id;

                    _tag_relative_x_pub.publish(_tag_relative_x_msg);
                    _tag_relative_y_pub.publish(_tag_relative_y_msg);
                    _tag_relative_z_pub.publish(_tag_relative_z_msg);
                    _tag_found_pub.publish(_tag_found_msg);
                    _R33_pub.publish(_R33_msg);
                    _detected_tag_pub.publish(_detected_tag_msg);

    //                cout << "this is the x: " << x_est << endl;
    //                cout << "this is the y: " << y_est << endl;
    //                cout << "this is the z: " << z_est << endl;
    //                cout << "this is the distance: " << distance << endl;
                }

            }

            // Loop through list of detections
            for (int i = 0; i < zarray_size(detections_36); i++) {
                apriltag_detection_t *det;
                zarray_get(detections_36, i, &det);

                // Define camera parameters struct
                apriltag_detection_info_t info;
                info.det = det;
                // ROSBAG DET:

                //info.tagsize = 0.16;
                info.tagsize = 0.20;
                info.fx = 1.0007824174077226e+03;
                info.fy = 1.0007824174077226e+03;
                info.cx = 640.;
                info.cy = 360.;

                // Estimate pose
                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info, &pose);
                matd_t* t = pose.t;
                matd_t* R = pose.R;
                double *tData = t->data;
                double *RData = R->data;
                x_raw = tData[0];
                y_raw = tData[1];
                z_raw = tData[2];
                _R33 = RData[8];
                id = det->id;

                // Check if this is really an april tag based on detected z-axis orientation
                if (_R33 >= 0.95 && (id == 9) ) {
                    //Low Pass Filter Parameters
                    float alpha = 0.2;
                    float beta = 0.05;
                    //Low Pass Filter
                    x_est = alpha*x_raw +(1-alpha)*x_est;
                    y_est = alpha*y_raw +(1-alpha)*y_est;
                    z_est = beta*z_raw +(1-beta)*z_est;
                    double distance = sqrt(x_est*x_est + y_est*y_est + z_est*z_est);

                    // Add offsets from relative location of camera from drone center
                    _xr = x_est + 0.2;
                    _yr = y_est + 0.0;
                    _zr = z_est + 0.0;

                    // Publish the vector from the drone to the april tag (camera frame)
                    _tag_relative_x_msg.data = _xr;
                    _tag_relative_y_msg.data = _yr;
                    _tag_relative_z_msg.data = _zr;
                    _R33_msg.data = _R33;
                    _detected_tag_msg.data = id;

                    _tag_relative_x_pub.publish(_tag_relative_x_msg);
                    _tag_relative_y_pub.publish(_tag_relative_y_msg);
                    _tag_relative_z_pub.publish(_tag_relative_z_msg);
                    _tag_found_pub.publish(_tag_found_msg);
                    _R33_pub.publish(_R33_msg);
                    _detected_tag_pub.publish(_detected_tag_msg);

    //                cout << "this is the x: " << x_est << endl;
    //                cout << "this is the y: " << y_est << endl;
    //                cout << "this is the z: " << z_est << endl;
    //                cout << "this is the distance: " << distance << endl;
                }

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

            for (int i = 0; i < zarray_size(detections_36); i++) {
                apriltag_detection_t *det;
                zarray_get(detections_36, i, &det);

                // Define camera parameters struct
                apriltag_detection_info_t info;
                info.det = det;
                // ROSBAG DET:

                //info.tagsize = 0.16;
                info.tagsize = 0.40;
                info.fx = 1.0007824174077226e+03;
                info.fy = 1.0007824174077226e+03;
                info.cx = 640.;
                info.cy = 360.;

                // Estimate pose
                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info, &pose);
                matd_t* t = pose.t;
                matd_t* R = pose.R;
                double *tData = t->data;
                double *RData = R->data;
                x_raw = tData[0];
                y_raw = tData[1];
                z_raw = tData[2];
                _R33 = RData[8];
                id = det->id;

                // Check if this is really an april tag based on detected z-axis orientation
                if (_R33 >= 0.95 && (id == 5) ) {
                    //Low Pass Filter Parameters
                    float alpha = 0.2;
                    float beta = 0.05;
                    //Low Pass Filter
                    x_est = alpha*x_raw +(1-alpha)*x_est;
                    y_est = alpha*y_raw +(1-alpha)*y_est;
                    z_est = beta*z_raw +(1-beta)*z_est;
                    double distance = sqrt(x_est*x_est + y_est*y_est + z_est*z_est);

                    // Add offsets from relative location of camera from drone center
                    _xr = x_est + 0.2;
                    _yr = y_est + 0.0;
                    _zr = z_est + 0.0;

                    // Publish the vector from the drone to the april tag (camera frame)
                    _tag_relative_x_msg.data = _xr;
                    _tag_relative_y_msg.data = _yr;
                    _tag_relative_z_msg.data = _zr;
                    _R33_msg.data = _R33;
                    _detected_tag_msg.data = id;

                    _tag_relative_x_pub.publish(_tag_relative_x_msg);
                    _tag_relative_y_pub.publish(_tag_relative_y_msg);
                    _tag_relative_z_pub.publish(_tag_relative_z_msg);
                    _tag_found_pub.publish(_tag_found_msg);
                    _R33_pub.publish(_R33_msg);
                    _detected_tag_pub.publish(_detected_tag_msg);

    //                cout << "this is the x: " << x_est << endl;
    //                cout << "this is the y: " << y_est << endl;
    //                cout << "this is the z: " << z_est << endl;
    //                cout << "this is the distance: " << distance << endl;
                }

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
            // clean up the detections
            zarray_destroy(detections_16);
            zarray_destroy(detections_36);

        }

        ros::spinOnce();

    }

    // need to stop the camera
    ROS_INFO("stopping camera");
    _camera.release();

    // remove apriltag stuff
	apriltag_detector_destroy(td_16);
	apriltag_detector_destroy(td_36);
	tag16h5_destroy(tf_16);
	tag36h11_destroy(tf_36);
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
