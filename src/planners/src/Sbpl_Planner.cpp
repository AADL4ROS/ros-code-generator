/**
 * Node Sbpl_Planner
 * File auto-generated on 10/11/2017 16:17:07
 */
#include "ros_base/ROSNode.h"
#include "planners/sbpl_planner.h"
#include "planners/Sbpl_Planner_configuration.h"
#include "geometry_msgs/PoseStamped.h"
#include "planners/global_planner_utils.h"
#include "nav_msgs/OccupancyGrid.h"
#include "planners/costmap_utils.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/Path.h"


class Sbpl_Planner : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	void goal_cb_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void costmap_cb_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void costmap_up_cb_callback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
	void path_out_pub_callback(const ros::TimerEvent& );
	void replan_cb_callback(const ros::TimerEvent& );
	InternalState is;
	ros::Subscriber sub_goal_cb;
	ros::Subscriber sub_costmap_cb;
	ros::Subscriber sub_costmap_up_cb;
	ros::Publisher pub_path_out_pub;
	ros::Timer timer_path_out_pub;
	ros::Timer timer_replan_cb;
public:
	 Sbpl_Planner();
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
	ros::init(argc, argv, "Sbpl_Planner", ros::init_options::NoSigintHandler);
	signal(SIGINT, nodeSigintHandler);
	Sbpl_Planner node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Sbpl_Planner::prepare() {
	Parameters p;
	handle.param<bool>("replan", p.replan, false);
	handle.getParam("starting_epsilon", p.starting_epsilon);
	handle.getParam("ending_epsilon", p.ending_epsilon);
	handle.param<int>("dimensions", p.dimensions, 3);
	handle.param<std::string>("motPrimString", p.motPrimString, "generated_primitives.txt");
	handle.param<std::string>("vehicleParams/robotShape", p.vehicleParams.robotShape, "circular");
	handle.param<double>("vehicleParams/robot_width", p.vehicleParams.robot_width, 1.5);
	handle.param<double>("vehicleParams/robot_length", p.vehicleParams.robot_length, 3.5);
	handle.param<double>("vehicleParams/v_max", p.vehicleParams.v_max, 2.0);
	handle.param<double>("vehicleParams/omega_max", p.vehicleParams.omega_max, 1.5708);
	handle.param<std::string>("planningSetup/planner_type", p.planningSetup.planner_type, "adstar");
	handle.param<double>("planningSetup/allocated_time", p.planningSetup.allocated_time, 10.0);
	handle.param<double>("planningSetup/initial_epsilon", p.planningSetup.initial_epsilon, 20.0);
	handle.param<double>("planningSetup/epsilon_decrease_step", p.planningSetup.epsilon_decrease_step, 0.5);
	handle.param<double>("mapParameters/resolutionTolerance", p.mapParameters.resolutionTolerance, 0.0001);
	handle.param<int>("mapParameters/obsthresh", p.mapParameters.obsthresh, 254);
	handle.param<int>("mapParameters/cost_inscribed_thresh", p.mapParameters.cost_inscribed_thresh, 253);
	handle.param<int>("mapParameters/cost_possibly_circumscribed_thresh", p.mapParameters.cost_possibly_circumscribed_thresh, 128);
	is.initialize(&p);
	sub_goal_cb = handle.subscribe("/goal", 1, &Sbpl_Planner::goal_cb_callback, this);
	sub_costmap_cb = handle.subscribe("/global/grid", 1, &Sbpl_Planner::costmap_cb_callback, this);
	sub_costmap_up_cb = handle.subscribe("/global/grid_updates", 1, &Sbpl_Planner::costmap_up_cb_callback, this);
	pub_path_out_pub = handle.advertise < nav_msgs::Path > ("/path", 10);
	timer_path_out_pub = handle.createTimer(ros::Duration(0.2), &Sbpl_Planner::path_out_pub_callback, this);
	timer_replan_cb = handle.createTimer(ros::Duration(1), &Sbpl_Planner::replan_cb_callback, this);
	return custom_prepare( is.vars(), is.params());
}

/**
 * Method tearDown auto-generated
 */
void Sbpl_Planner::tearDown() {
	ROS_INFO("Node is shutting down");
	return;
}

/**
 * Method errorHandling auto-generated
 */
void Sbpl_Planner::errorHandling() {
	ROSNode::errorHandling();
}

/**
 * Method goal_cb_callback auto-generated
 */
void Sbpl_Planner::goal_cb_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	saveGoal( is.vars(), is.params(), msg);
}

/**
 * Method costmap_cb_callback auto-generated
 */
void Sbpl_Planner::costmap_cb_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	saveCostmap( is.vars(), is.params(), msg);
}

/**
 * Method costmap_up_cb_callback auto-generated
 */
void Sbpl_Planner::costmap_up_cb_callback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
	updateCostmap( is.vars(), is.params(), msg);
}

/**
 * Method path_out_pub_callback auto-generated
 */
void Sbpl_Planner::path_out_pub_callback(const ros::TimerEvent& ) {
	pub_path_out_pub.publish(sbplPlan( is.vars(), is.params()));
}

/**
 * Method replan_cb_callback auto-generated
 */
void Sbpl_Planner::replan_cb_callback(const ros::TimerEvent& ) {
	setReplan( is.vars(), is.params());
}

/**
 * Method Sbpl_Planner auto-generated
 */
 Sbpl_Planner::Sbpl_Planner() {
	setName(ros::this_node::getName());
}

