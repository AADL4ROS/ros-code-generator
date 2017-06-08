/**
 * Node listener_node
 * File auto-generated on 08/06/2017 10:58:06
 */

#include "ros_base/ROSNode.h"

/**
 * Automatically imported from message datatype
 */
#include "std_msgs/String.h"

#define NODE_NAME "listener_node"

class Listener_Node : public ros_base::ROSNode {
private:
    bool prepare();
    void errorHandling();
    void tearDown();
    
    struct params {
        std::string node_name;
    } params;
    
    ros::Subscriber sub;
    void subCallback(const std_msgs::String::ConstPtr& msg);
public:
    Listener_Node();
};

Listener_Node::Listener_Node() {
    setName(NODE_NAME);
}

void Listener_Node::subCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("%s", msg->data.c_str());
}

bool Listener_Node::prepare() {
    params.node_name = NODE_NAME;
    
    handle.getParam("node_name", params.node_name);
    
    sub = handle.subscribe("/chatter", 10, &Listener_Node::subCallback, this);
    
    return true;
}

void Listener_Node::errorHandling() {
    ROSNode::errorHandling();
}

void Listener_Node::tearDown() {
    ROS_INFO("Node is shutting down");
    return;
}

void nodeSigintHandler(int sig) {
    g_request_shutdown = 1;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME, ros::init_options::NoSigintHandler);
    while(!ros::master::check())
        usleep(1000);
    signal(SIGINT, nodeSigintHandler);
    Listener_Node node;
    node.start();
    return 0;
}
