/**
 * Node Publisher
 * File auto-generated on 23/10/2017 14:52:38
 */
#include "ros_base/ROSNode.h"
#include "client_server_example/prepare_pub.h"
#include "client_server_example/Publisher_configuration.h"
#include "std_msgs/String.h"
#include "client_server_example/publisher.h"


class Publisher : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	void publisher_callback(const ros::TimerEvent& );
	InternalState is;
	ros::Publisher pub_publisher;
	ros::Timer timer_publisher;
public:
	 Publisher();
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
	ros::init(argc, argv, "Publisher", ros::init_options::NoSigintHandler);
	signal(SIGINT, nodeSigintHandler);
	Publisher node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Publisher::prepare() {
	Parameters p;
	handle.param<std::string>("stringName", p.stringName, "ciao");
	handle.param<double>("testReal", p.testReal, 0);
	is.initialize(&p);
	pub_publisher = handle.advertise < std_msgs::String > ("/out_topic", 10);
	timer_publisher = handle.createTimer(ros::Duration(0.01), &Publisher::publisher_callback, this);
	custom_prepare( is.vars(), is.params());
	return true;
}

/**
 * Method tearDown auto-generated
 */
void Publisher::tearDown() {
	ROS_INFO("Node is shutting down");
	return;
}

/**
 * Method errorHandling auto-generated
 */
void Publisher::errorHandling() {
	ROSNode::errorHandling();
}

/**
 * Method publisher_callback auto-generated
 */
void Publisher::publisher_callback(const ros::TimerEvent& ) {
	pub_publisher.publish(funzione_publisher( is.vars(), is.params()));
}

/**
 * Method Publisher auto-generated
 */
 Publisher::Publisher() {
	setName(ros::this_node::getName());
}

