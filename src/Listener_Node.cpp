/**
 * Node Listener_Node
 * File auto-generated on 14/06/2017 16:01:40
 */
#include "ros_base/ROSNode.h"
#include "std_msgs/String.h"
#define NODE_NAME "Listener_Node"
class Listener_Node : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	void subscriber_callback(const std::string:ConstPtr& msg);
	struct params {
		std::string node_name;
	} params;
	ros::Subscriber sub_subscriber;
public:
	 Listener_Node();
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
	Listener_Node node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Listener_Node::prepare() {
	params.node_name = NODE_NAME;
	handle.getParam("node_name", params.node_name);
	sub_subscriber = handle.subscribe("/chatter", 10, &Listener_Node::subscriber_callback, this);
	return true;
}

/**
 * Method tearDown auto-generated
 */
void Listener_Node::tearDown() {
	ROS_INFO("Node is shutting down");
	return;
}

/**
 * Method errorHandling auto-generated
 */
void Listener_Node::errorHandling() {
	ROSNode::errorHandling();
}

/**
 * Method subscriber_callback auto-generated
 */
void Listener_Node::subscriber_callback(const std::string:ConstPtr& msg) {
	ROS_INFO("%s", msg->data.c_str());
	/**
	 * Source text: listener.cpp
	 */
	
}

/**
 * Method Listener_Node auto-generated
 */
 Listener_Node::Listener_Node() {
	setName(NODE_NAME);
}
