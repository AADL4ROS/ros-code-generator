/**
 * Node Publisher
 * File auto-generated on 18/10/2017 16:16:41
 */
#include "ros_base/ROSNode.h"
#include "std_msgs/String.h"

#define NODE_NAME "Publisher"

class Publisher : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	void publisher_callback(const ros::TimerEvent& );
	struct vars {
		double starting_time_publisher;
	} vars;
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
	ros::init(argc, argv, Publisher, ros::init_options::NoSigintHandler);
	signal(SIGINT, nodeSigintHandler);
	Publisher node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Publisher::prepare() {
	pub_publisher = handle.advertise < std_msgs::String > ("/out_topic", 10);
	timer_publisher = handle.createTimer(ros::Duration(0.01), &Publisher::publisher_callback, this);
	vars.starting_time_publisher = ros::Time::now().toSec();
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
 * Method Publisher auto-generated
 */
 Publisher::Publisher() {
	setName(ros::this_node::getName());
}

