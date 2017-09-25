/**
 * Node Talker_Node
 * File auto-generated on 25/09/2017 15:30:58
 */
#include "ros_base/ROSNode.h"
#include "std/string.h"
#include "std_msgs/String.h"
#define NODE_NAME "Talker_Node"
class Talker_Node : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	void publisher_callback(const ros::TimerEvent& );
	void publisher_1_callback(const ros::TimerEvent& );
	struct params {
		std::string node_name;
		int frequency_publisher;
		int frequency_publisher_1;
	} params;
	struct vars {
		double starting_time_publisher;
		double starting_time_publisher_1;
	} vars;
	ros::Publisher pub_publisher;
	ros::Timer timer_publisher;
	ros::Publisher pub_publisher_1;
	ros::Timer timer_publisher_1;
public:
	 Talker_Node();
};

/**
 * Method nodeSigintHandler auto-generated
 */
void nodeSigintHandler(int sig) {
	g_request_shutdown = 1;
}

/**
 * Method main auto-generated
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_NAME, ros::init_options::NoSigintHandler);
	while(!ros::master::check())
		usleep(1000);
	signal(SIGINT, nodeSigintHandler);
	Talker_Node node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Talker_Node::prepare() {
	params.node_name = NODE_NAME;
	params.frequency_publisher = 100.0;
	params.frequency_publisher_1 = 10000.0;
	handle.getParam("node_name", params.node_name);
	handle.getParam("frequency_publisher", params.frequency_publisher);
	pub_publisher = handle.advertise < std_msgs::String > ("/chatter", 10);
	timer_publisher = handle.createTimer(ros::Duration(1/params.frequency_publisher), &Talker_Node::publisher_callback, this);
	vars.starting_time_publisher = ros::Time::now().toSec();
	handle.getParam("frequency_publisher_1", params.frequency_publisher_1);
	pub_publisher_1 = handle.advertise < std_msgs::String > ("/chatter", 10);
	timer_publisher_1 = handle.createTimer(ros::Duration(1/params.frequency_publisher_1), &Talker_Node::publisher_1_callback, this);
	vars.starting_time_publisher_1 = ros::Time::now().toSec();
	return true;
}

/**
 * Method tearDown auto-generated
 */
void Talker_Node::tearDown() {
	ROS_INFO("Node is shutting down");
	return;
}

/**
 * Method errorHandling auto-generated
 */
void Talker_Node::errorHandling() {
	ROSNode::errorHandling();
}

/**
 * Method publisher_callback auto-generated
 */
void Talker_Node::publisher_callback(const ros::TimerEvent& ) {
	std_msgs::String msg;
	std::stringstream ss;
	ss << "current time: " << (ros::Time::now().toSec() - vars.starting_time_publisher);
	msg.data = ss.str().c_str();
	pub_publisher.publish(msg);
	/**
	 * Source text: talker.cpp
	 */
	
}

/**
 * Method publisher_1_callback auto-generated
 */
void Talker_Node::publisher_1_callback(const ros::TimerEvent& ) {
	std_msgs::String msg;
	std::stringstream ss;
	ss << "current time: " << (ros::Time::now().toSec() - vars.starting_time_publisher_1);
	msg.data = ss.str().c_str();
	pub_publisher_1.publish(msg);
	/**
	 * Source text: talker_2.cpp
	 */
	
}

/**
 * Method Talker_Node auto-generated
 */
 Talker_Node::Talker_Node() {
	setName(NODE_NAME);
}

