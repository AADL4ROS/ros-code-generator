/**
 * Node Server_Node
 * File auto-generated on 14/10/2017 18:50:29
 */
#include "ros_base/ROSNode.h"
#include "client_server_example/Custom_Service.h"

#define NODE_NAME "Server_Node"

class Server_Node : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	bool receiver_service_callback(client_server_example::Custom_Service::Request &req, client_server_example::Custom_Service::Request &res);
	ros::ServiceServer service_server_receiver;
public:
	 Server_Node();
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
	Server_Node node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Server_Node::prepare() {
	service_server_receiver = handle.advertiseService("service", receiver_service_callback);
	return true;
}

/**
 * Method tearDown auto-generated
 */
void Server_Node::tearDown() {
	ROS_INFO("Node is shutting down");
	return;
}

/**
 * Method errorHandling auto-generated
 */
void Server_Node::errorHandling() {
	ROSNode::errorHandling();
}

/**
 * Method receiver_service_callback auto-generated
 */
bool Server_Node::receiver_service_callback(client_server_example::Custom_Service::Request &req, client_server_example::Custom_Service::Request &res) {
	/**
	 * Source text: service_source.cpp
	 */
	
}

/**
 * Method Server_Node auto-generated
 */
 Server_Node::Server_Node() {
	setName(NODE_NAME);
}

