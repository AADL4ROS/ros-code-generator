{{__DISCLAIMER__}}
#include "ros_base/ROSNode.h"

{{__DATATYPE_INCLUDES__}}

#define NODE_NAME "{{__NODE_NAME__}}"

class {{__CLASS_NAME__}} : public ros_base::ROSNode {
private:
    bool prepare();
    void errorHandling();
    void tearDown();
    
    struct params {
        std::string node_name;
    } params;
    
    ros::Subscriber sub;
    void subCallback(const {{__DT_NAMESPACE__}}::{{__DATATYPE__}}::ConstPtr& msg);
public:
    {{__CLASS_NAME__}}();
};

{{__CLASS_NAME__}}::{{__CLASS_NAME__}}() {
    setName(NODE_NAME);
}

void {{__CLASS_NAME__}}::subCallback(const {{__DT_NAMESPACE__}}::{{__DATATYPE__}}::ConstPtr& msg) {
    ROS_INFO("%s", msg->data.c_str());
}

bool {{__CLASS_NAME__}}::prepare() {
    params.node_name = NODE_NAME;
    
    handle.getParam("node_name", params.node_name);
    
    sub = handle.subscribe("/chatter", 10, &{{__CLASS_NAME__}}::subCallback, this);
    
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
