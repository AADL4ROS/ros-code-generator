/**
 * Node Talker_Node_C
 * File auto-generated on 09/10/2017 22:27:48
 */
#include "ros_base/ROSNode.h"
#define NODE_NAME "Talker_Node_C"

class Talker_Node_C : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
public:
	 Talker_Node_C();
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
	Talker_Node_C node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Talker_Node_C::prepare() {
	return true;
}

/**
 * Method tearDown auto-generated
 */
void Talker_Node_C::tearDown() {
	ROS_INFO("Node is shutting down");
	return;
}

/**
 * Method errorHandling auto-generated
 */
void Talker_Node_C::errorHandling() {
	ROSNode::errorHandling();
}

/**
 * Method Talker_Node_C auto-generated
 */
 Talker_Node_C::Talker_Node_C() {
	setName(NODE_NAME);
}

