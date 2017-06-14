/**
 * Node Talker_Node
 * File auto-generated on 14/06/2017 16:46:06
 */
#include "ros_base/ROSNode.h"
#include "std_msgs/String.h"
#define NODE_NAME "Talker_Node"
class Talker_Node : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	void publisher_one_callback(const ros::TimerEvent& );
	void publisher_two_callback(const ros::TimerEvent& );
	struct params {
		std::string node_name;
		int frequency_publisher_one;
		int frequency_publisher_two;
	} params;
	struct vars {
		double starting_time_publisher_one;
		double starting_time_publisher_two;
	} vars;
	ros::Publisher pub_publisher_one;
	ros::Timer timer_publisher_one;
	ros::Publisher pub_publisher_two;
	ros::Timer timer_publisher_two;
public:
	 Talker_Node();
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
	Talker_Node node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Talker_Node::prepare() {
	params.node_name = NODE_NAME;
	params.<variables.Variable.Variable object at 0x10b409e48> = 100.0;
	params.<variables.Variable.Variable object at 0x10b37b4e0> = 20000.0;
	handle.getParam("node_name", params.node_name);
	handle.getParam("frequency_publisher_one", params.frequency_publisher_one);
	pub_publisher_one = handle.advertise < std::string > ("/chatter", 10);
	timer_publisher_one = handle.createTimer(ros::Duration(1/params.frequency_publisher_one), &Talker_Node::publisher_one_callback, this);
	vars.starting_time_publisher_one = ros::Time::now().toSec();
	handle.getParam("frequency_publisher_two", params.frequency_publisher_two);
	pub_publisher_two = handle.advertise < std::string > ("/chatter", 10);
	timer_publisher_two = handle.createTimer(ros::Duration(1/params.frequency_publisher_two), &Talker_Node::publisher_two_callback, this);
	vars.starting_time_publisher_two = ros::Time::now().toSec();
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
 * Method publisher_one_callback auto-generated
 */
void Talker_Node::publisher_one_callback(const ros::TimerEvent& ) {
	std_msgs::String msg;
	std::stringstream ss;
	ss << "current time: " << (ros::Time::now().toSec() - vars.starting_time_publisher_one);
	msg.data = ss.str().c_str();
	pub_publisher_one.publish(msg);
	/**
	 * Source text: talker_one.cpp
	 */
	
}

/**
 * Method publisher_two_callback auto-generated
 */
void Talker_Node::publisher_two_callback(const ros::TimerEvent& ) {
	std_msgs::String msg;
	std::stringstream ss;
	ss << "current time: " << (ros::Time::now().toSec() - vars.starting_time_publisher_two);
	msg.data = ss.str().c_str();
	pub_publisher_two.publish(msg);
	/**
	 * Source text: talker_two.cpp
	 */
	
}

/**
 * Method Talker_Node auto-generated
 */
 Talker_Node::Talker_Node() {
	setName(NODE_NAME);
}

