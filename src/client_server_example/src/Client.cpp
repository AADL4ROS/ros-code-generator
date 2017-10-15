/**
 * Node Client
 * File auto-generated on 15/10/2017 15:29:18
 */
#include "ros_base/ROSNode.h"
#include "client_server_example/Custom_Service.h"

#define NODE_NAME "Client"

class Client : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	void publisher_callback(const ros::TimerEvent& );
	struct params {
		int frequency_publisher;
	} params;
	struct vars {
		double starting_time_publisher;
	} vars;
	ros::ServiceClient service_client_caller;
	ros::Publisher pub_publisher;
	ros::Timer timer_publisher;
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
	params.frequency_publisher = 100.0;
	service_client_caller = handle.serviceClient<client_server_example::Custom_Service>("service");
	handle.getParam("frequency_publisher", params.frequency_publisher);
	pub_publisher = handle.advertise < std_msgs::String > ("/out_topic", 10);
	timer_publisher = handle.createTimer(ros::Duration(1/params.frequency_publisher), &Client::publisher_callback, this);
	vars.starting_time_publisher = ros::Time::now().toSec();
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
 * Method publisher_callback auto-generated
 */
void Client::publisher_callback(const ros::TimerEvent& ) {
	std_msgs::String msg;
	std::stringstream ss;
	ss << "current time: " << (ros::Time::now().toSec() - vars.starting_time_publisher);
	msg.data = ss.str().c_str();
	pub_publisher.publish(msg);
	/**
	 * Source text: publisher.cpp
	 */
	
}

/**
 * Method Client auto-generated
 */
 Client::Client() {
	setName(NODE_NAME);
}

