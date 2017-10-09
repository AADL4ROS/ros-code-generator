/**
 * Node Listener_Node_Cus
 * File auto-generated on 09/10/2017 22:27:09
 */
#include "ros_base/ROSNode.h"
#define NODE_NAME "Listener_Node_Cus"

class Listener_Node_Cus : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
public:
	 Listener_Node_Cus();
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
int main(int argc, char** argv) {
	ros::init(argc, argv, NODE_NAME, ros::init_options::NoSigintHandler);
	while(!ros::master::check())
		usleep(1000);
	signal(SIGINT, nodeSigintHandler);
	Listener_Node_Cus node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Listener_Node_Cus::prepare() {
	return true;
}

/**
 * Method tearDown auto-generated
 */
void Listener_Node_Cus::tearDown() {
	ROS_INFO("Node is shutting down");
	return;
}

/**
 * Method errorHandling auto-generated
 */
void Listener_Node_Cus::errorHandling() {
	ROSNode::errorHandling();
}

/**
 * Method Listener_Node_Cus auto-generated
 */
 Listener_Node_Cus::Listener_Node_Cus() {
	setName(NODE_NAME);
}

