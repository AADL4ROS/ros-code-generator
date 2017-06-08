/**
 * Node talker_node
 * File auto-generated on 08/06/2017 10:58:06
 */

#include "ros_base/ROSNode.h"

/**
 * Automatically imported from message datatype
 */
#include "std_msgs/String.h"

#define NODE_NAME "talker_node"

class Talker_Node : public ros_base::ROSNode {
private:
    bool prepare();
    void errorHandling();
    void tearDown();
    
    struct params {
        std::string node_name;
        int frequency;
    } params;
    
    struct vars {
        double starting_time;
    } vars;
    
    ros::Publisher pub;
    ros::Timer timer;
    void pubCallback(const ros::TimerEvent&);
public:
    Talker_Node();
};

Talker_Node::Talker_Node() {
    setName(NODE_NAME);
}

void Talker_Node::pubCallback(const ros::TimerEvent&) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "current time: " << (ros::Time::now().toSec() - vars.starting_time);
    msg.data = ss.str().c_str();
    pub.publish(msg);
}

bool Talker_Node::prepare() {
    params.node_name = NODE_NAME;
    params.frequency = 100;
    
    handle.getParam("node_name", params.node_name);
    handle.getParam("frequency", params.frequency);
    
    pub = handle.advertise<std_msgs::String>("/chatter", 10);
    timer = handle.createTimer(ros::Duration(1/params.frequency), &Talker_Node::pubCallback, this);
    
    vars.starting_time = ros::Time::now().toSec();
    return true;
}

void Talker_Node::errorHandling() {
    ROSNode::errorHandling();
}

void Talker_Node::tearDown() {
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
    Talker_Node node;
    node.start();
    return 0;
}

