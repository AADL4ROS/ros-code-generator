{{__DISCLAIMER__}}
#include "ros_base/ROSNode.h"
#include "std_msgs/String.h"

#define NODE_NAME {{__NODE_NAME__}}

class {{__CLASS_NAME__}} : public ros_base::ROSNode {
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
    {{__CLASS_NAME__}}();
};

{{__CLASS_NAME__}}::{{__CLASS_NAME__}}() {
    setName(NODE_NAME);
}

void {{__CLASS_NAME__}}::pubCallback(const ros::TimerEvent&) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "current time: " << (ros::Time::now().toSec() - vars.starting_time);
    msg.data = ss.str().c_str();
    pub.publish(msg);
}

bool {{__CLASS_NAME__}}::prepare() {
    params.node_name = NODE_NAME;
    params.frequency = {{__FREQUENCY__}};
    
    handle.getParam("node_name", params.node_name);
    handle.getParam("frequency", params.frequency);
    
    pub = handle.advertise<std_msgs::String>("/chatter", 10);
    timer = handle.createTimer(ros::Duration(1/params.frequency), &{{__CLASS_NAME__}}::pubCallback, this);
    
    vars.starting_time = ros::Time::now().toSec();
    return true;
}

void {{__CLASS_NAME__}}::errorHandling() {
    ROSNode::errorHandling();
}

void {{__CLASS_NAME__}}::tearDown() {
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
    {{__CLASS_NAME__}} node;
    node.start();
    return 0;
}

