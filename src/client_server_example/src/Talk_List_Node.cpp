/**
 * Node Talk_List_Node
 * File auto-generated on 10/11/2017 14:16:09
 */
#include "ros_base/ROSNode.h"
#include "client_server_example/Talk_List_Node_configuration.h"
#include "std_msgs/String.h"
#include "custom_msgs/Complex.h"
#include "client_server_example/talk_list.h"
#include "ros_base/tf_interface.h"


class Talk_List_Node : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	void call_pub_callback(const std_msgs::String::ConstPtr& msg);
	InternalState is;
	ros::Subscriber sub_call_pub;
	ros::Publisher pub_call_pub;
	ros_base::TransformationFrames * tf;
public:
	 Talk_List_Node();
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
	ros::init(argc, argv, "Talk_List_Node", ros::init_options::NoSigintHandler);
	signal(SIGINT, nodeSigintHandler);
	Talk_List_Node node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Talk_List_Node::prepare() {
	Parameters p;
	handle.param<std::string>("stringName", p.stringName, "ciao");
	handle.param<double>("testReal", p.testReal, 0);
	handle.param<double>("traj/test1", p.traj.test1, 1.0);
	handle.param<int>("traj/test2", p.traj.test2, 2);
	is.initialize(&p);
	sub_call_pub = handle.subscribe("/in_topic", 1, &Talk_List_Node::call_pub_callback, this);
	pub_call_pub = handle.advertise<custom_msgs::Complex>("/out_topic", 10);
	return true;
}

/**
 * Method tearDown auto-generated
 */
void Talk_List_Node::tearDown() {
	ROS_INFO("Node is shutting down");
	return;
}

/**
 * Method errorHandling auto-generated
 */
void Talk_List_Node::errorHandling() {
	ROSNode::errorHandling();
}

/**
 * Method call_pub_callback auto-generated
 */
void Talk_List_Node::call_pub_callback(const std_msgs::String::ConstPtr& msg) {
	pub_call_pub.publish(funzione_talk_list( is.vars(), is.params(), msg, tf));
}

/**
 * Method Talk_List_Node auto-generated
 */
 Talk_List_Node::Talk_List_Node() {
	setName(ros::this_node::getName());
	tf = new ros_base::TransformationFrames();
}

