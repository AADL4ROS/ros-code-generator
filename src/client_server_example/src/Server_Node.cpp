/**
 * Node Server_Node
 * File auto-generated on 19/10/2017 18:13:28
 */
#include "ros_base/ROSNode.h"
#include "client_server_example/Server_Node_configuration.h"
#include "custom_srvs/ServiceA.h"
#include "custom_msgs/Complex.h"
#include "client_server_example/subscriber.h"


class Server_Node : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	bool receiver_service_callback(custom_srvs::ServiceA::Request &req, custom_srvs::ServiceA::Response &res);
	void subscriber_callback(const custom_msgs::Complex::ConstPtr& msg);
	InternalState is;
	ros::ServiceServer service_server_receiver;
	ros::Subscriber sub_subscriber;
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
	ros::init(argc, argv, "Server_Node", ros::init_options::NoSigintHandler);
	signal(SIGINT, nodeSigintHandler);
	Server_Node node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Server_Node::prepare() {
	Parameters p;
	handle.param<std::string>("stringName", p.stringName, "ciao");
	handle.param<double>("testReal", p.testReal, 0);
	handle.getParam("testNoDefault", p.testNoDefault);
	is.initialize(&p);
	service_server_receiver = handle.advertiseService("service", &Server_Node::receiver_service_callback, this);
	sub_subscriber = handle.subscribe("/out_topic", 1, &Server_Node::subscriber_callback, this);
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
bool Server_Node::receiver_service_callback(custom_srvs::ServiceA::Request &req, custom_srvs::ServiceA::Response &res) {
	/**
	 * Source text: service_source.h
	 */
	
}

/**
 * Method subscriber_callback auto-generated
 */
void Server_Node::subscriber_callback(const custom_msgs::Complex::ConstPtr& msg) {
	funzione_subscriber( is.vars(), is.params(), msg);
}

/**
 * Method Server_Node auto-generated
 */
 Server_Node::Server_Node() {
	setName(ros::this_node::getName());
}

