/**
 * this node will contain a skeleton for students for the control node.
 *
 * NOTE: may want to rename this node... there will be 2 control nodes I think
 * 	- landing control node
 * 	- mission control node
 *
 * Though, they don't necessarily need to be in separate nodes, they could all
 * be in the same node with a state machine running and helping to state what
 * elements of the node should be running.
 *
 * I don't necessarily want to dictate how they define their nodes....
 * but I do want to give them an example.
 *
 * Maybe give the example but caution/note that this may not be the best way
 * to do it, there are choices to be made by the teams
 */


#include <ros/ros.h>



class AA241xControlNode {

public:

	AA241xControlNode();

	int run();


private:


	// node handler
	ros::NodeHandler _nh;

	// TODO: add any settings, etc, here


	// subscribers
	ros::Subscriber _beacon_meas_sub;	// measurement to the beacons (or whatever the meas will be)
	ros::Subscriber _ap_range_sub;		// measurement from the imaging node


	// publishers (?)
	// TODO: set up publishers for data you want to log with rosbag

	// callbacks
	// TODO: add callbacks here
	void beaconMeasCallback(const aa241x_mission::Measurement::ConstPtr& msg);  // NOTE: this might not even be wanted here by students
	void apRangeCallback(const aa241x_student::APRange::ConstPtr& msg);


	// helper functions
	// TODO: define helper functions here



};


AA241xControlNode::AA241xControlNode() {


	// subscribe to the desired topics
	_beacon_meas_sub = _nh.subscribe<aa241x_mission::Measurement>("measurement", 10, &AA241xControlNode::beaconMeasCallback, this);
	_ap_range_sub = _nh.subscribe<aa241x_student::APRange>("ap_range", 10, &AA241xControlNode::apRangeCallback, this);

}

// TODO: I think I am going to remove all beacon measurement related content from this node
// TODO: create another node for the path planning that takes in this information
void AA241xControlNode::beaconMeasCallback(const aa241x_mission::Measurement::ConstPtr& msg) {
	// TODO: decide what to do with the beacon measurement here
}

void AA241xControlNode::apRangeCallback(const aa241x_student::APRange::ConstPtr& msg) {
	// TODO: decide what to do with the range measurement here
	// TODO: basically writing some of the controller in this callback
}



void AA241xControlNode::run() {

	// TODO: copy the code from my research here
	// TODO: should send a 0 motion command until in offboard mode and then
	// should start sending commands as desired to go where desired

	// TODO: maybe path planning should be its own node, but again I'm thinking
	// that is an exercise left to the students
	// this skeleton code should be enough to get them to understand what is
	// going on and how to handle other element (such as path planning)


}

int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "aa241x_control_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings

	// create the node
	AA241xControlNode node();

	// run the node
	return node.run();
}