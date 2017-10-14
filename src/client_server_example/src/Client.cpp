/**
 * Node Client
 * File auto-generated on 14/10/2017 18:50:29
 */
#include "ros_base/ROSNode.h"
#include "client_server_example/Custom_Service.h"

#define NODE_NAME "Client"

class Client : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	ros::ServiceClient service_client_caller;
public:
	 Client();
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
	Client node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Client::prepare() {
	service_client_caller = handle.serviceClient<client_server_example::Custom_Service>("service");
	return true;
}

/**
 * Method tearDown auto-generated
 */
void Client::tearDown() {
	ROS_INFO("Node is shutting down");
	return;
}

/**
 * Method errorHandling auto-generated
 */
void Client::errorHandling() {
	ROSNode::errorHandling();
}

/**
 * Method Client auto-generated
 */
 Client::Client() {
	setName(NODE_NAME);
}

