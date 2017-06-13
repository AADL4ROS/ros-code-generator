/**
 * Node Talker_Node
 * File auto-generated on 13/06/2017 17:36:08
 */
#include "ros_base/ROSNode.h"
#include "std_msgs/String.h"
#include "std_msgs/String.h"
#include "std_msgs/String.h"
#define NODE_NAME "Talker_Node"
class Talker_Node : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	struct params {
		std::string node_name;
		int frequency;
	} params;
	struct vars {
		double starting_time;
	} vars;
	ros::Publisher pub_publisher_one;
	ros::Timer timer_publisher_one;
public:
	 Talker_Node();
};

/**
 * Method nodeSigintHandler auto-generated
 */
void nodeSigintHandler(int sig;) {
	g_request_shutdown = 1;
}

/**
 * Method main auto-generated
 */
int main(int argc;, char **argv;) {
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
	params.frequency = 100.0;
	handle.getParam("node_name", params.node_name);
	handle.getParam("frequency", params.frequency);
	pub_publisher_one = handle.advertise < std::string > ("/chatter", 10);
	timer_publisher_one = handle.createTimer(ros::Duration(1/params.frequency), &Talker_Node::publisher_one_callback, this);
	vars.starting_time = ros::Time::now().toSec();
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
 * Method Talker_Node auto-generated
 */
 Talker_Node::Talker_Node() {
	setName(NODE_NAME);
}

