/**
 * Node Dwa_Planner
 * File auto-generated on 10/11/2017 15:42:15
 */
#include "ros_base/ROSNode.h"
#include "planners/dwa_planner.h"
#include "planners/Dwa_Planner_configuration.h"
#include "nav_msgs/Path0.h"
#include "planners/save_path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "planners/costmap_utils.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "geometry_msgs/Twist.h"


class Dwa_Planner : public ros_base::ROSNode {
private:
	bool prepare();
	void tearDown();
	void errorHandling();
	void path_cb_callback(const nav_msgs::Path0::ConstPtr& msg);
	void costmap_cb_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void costmap_up_cb_callback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
	void cmd_vel_pub_callback(const ros::TimerEvent& );
	InternalState is;
	ros::Subscriber sub_path_cb;
	ros::Subscriber sub_costmap_cb;
	ros::Subscriber sub_costmap_up_cb;
	ros::Publisher pub_cmd_vel_pub;
	ros::Timer timer_cmd_vel_pub;
public:
	 Dwa_Planner();
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
	ros::init(argc, argv, "Dwa_Planner", ros::init_options::NoSigintHandler);
	signal(SIGINT, nodeSigintHandler);
	Dwa_Planner node;
	node.start();
	return 0;
}

/**
 * Method prepare auto-generated
 */
bool Dwa_Planner::prepare() {
	Parameters p;
	handle.param<double>("robotConfiguration/acc_lim_x", p.robotConfiguration.acc_lim_x, 2.5);
	handle.param<double>("robotConfiguration/acc_lim_y", p.robotConfiguration.acc_lim_y, 2.5);
	handle.param<double>("robotConfiguration/acc_lim_th", p.robotConfiguration.acc_lim_th, 3.2);
	handle.param<double>("robotConfiguration/max_trans_vel", p.robotConfiguration.max_trans_vel, 0.55);
	handle.param<double>("robotConfiguration/min_trans_vel", p.robotConfiguration.min_trans_vel, 0.1);
	handle.param<double>("robotConfiguration/max_vel_x", p.robotConfiguration.max_vel_x, 0.55);
	handle.param<double>("robotConfiguration/min_vel_x", p.robotConfiguration.min_vel_x, 0.0);
	handle.param<double>("robotConfiguration/max_vel_y", p.robotConfiguration.max_vel_y, 0.1);
	handle.param<double>("robotConfiguration/min_vel_y", p.robotConfiguration.min_vel_y, -0.1);
	handle.param<double>("robotConfiguration/max_rot_vel", p.robotConfiguration.max_rot_vel, 1.0);
	handle.param<double>("robotConfiguration/min_rot_vel", p.robotConfiguration.min_rot_vel, 0.4);
	handle.param<double>("goalTolerance/yaw_goal_tolerance", p.goalTolerance.yaw_goal_tolerance, 0.05);
	handle.param<double>("goalTolerance/xy_goal_tolerance", p.goalTolerance.xy_goal_tolerance, 0.10);
	handle.param<bool>("goalTolerance/latch_xy_goal_tolerance", p.goalTolerance.latch_xy_goal_tolerance, false);
	handle.param<double>("forwardSimulation/sim_time", p.forwardSimulation.sim_time, 1.7);
	handle.param<double>("forwardSimulation/sim_granularity", p.forwardSimulation.sim_granularity, 0.025);
	handle.param<int>("forwardSimulation/vx_samples", p.forwardSimulation.vx_samples, 3);
	handle.param<int>("forwardSimulation/vy_samples", p.forwardSimulation.vy_samples, 10);
	handle.param<int>("forwardSimulation/vth_samples", p.forwardSimulation.vth_samples, 20);
	handle.param<double>("forwardSimulation/controller_frequency", p.forwardSimulation.controller_frequency, 20.0);
	handle.param<double>("trajectoryScoring/path_distance_bias", p.trajectoryScoring.path_distance_bias, 32.0);
	handle.param<double>("trajectoryScoring/goal_distance_bias", p.trajectoryScoring.goal_distance_bias, 24.0);
	handle.param<double>("trajectoryScoring/occdist_scale", p.trajectoryScoring.occdist_scale, 0.01);
	handle.param<double>("trajectoryScoring/forward_point_distance", p.trajectoryScoring.forward_point_distance, 0.325);
	handle.param<double>("trajectoryScoring/stop_time_buffer", p.trajectoryScoring.stop_time_buffer, 0.2);
	handle.param<double>("trajectoryScoring/scaling_speed", p.trajectoryScoring.scaling_speed, 0.25);
	handle.param<double>("trajectoryScoring/max_scaling_factor", p.trajectoryScoring.max_scaling_factor, 0.2);
	handle.param<double>("oscillation_reset_dist", p.oscillation_reset_dist, 0.05);
	handle.param<bool>("prune_plan", p.prune_plan, true);
	is.initialize(&p);
	sub_path_cb = handle.subscribe("/path", 1, &Dwa_Planner::path_cb_callback, this);
	sub_costmap_cb = handle.subscribe("/local/grid", 1, &Dwa_Planner::costmap_cb_callback, this);
	sub_costmap_up_cb = handle.subscribe("/local/grid_updates", 1, &Dwa_Planner::costmap_up_cb_callback, this);
	pub_cmd_vel_pub = handle.advertise < geometry_msgs::Twist > ("/cmd_vel", 10);
	timer_cmd_vel_pub = handle.createTimer(ros::Duration(0.05), &Dwa_Planner::cmd_vel_pub_callback, this);
	return custom_prepare( is.vars(), is.params());
}

/**
 * Method tearDown auto-generated
 */
void Dwa_Planner::tearDown() {
	ROS_INFO("Node is shutting down");
	return;
}

/**
 * Method errorHandling auto-generated
 */
void Dwa_Planner::errorHandling() {
	ROSNode::errorHandling();
}

/**
 * Method path_cb_callback auto-generated
 */
void Dwa_Planner::path_cb_callback(const nav_msgs::Path0::ConstPtr& msg) {
	savePath( is.vars(), is.params(), msg);
}

/**
 * Method costmap_cb_callback auto-generated
 */
void Dwa_Planner::costmap_cb_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	saveCostmap( is.vars(), is.params(), msg);
}

/**
 * Method costmap_up_cb_callback auto-generated
 */
void Dwa_Planner::costmap_up_cb_callback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
	updateCostmap( is.vars(), is.params(), msg);
}

/**
 * Method cmd_vel_pub_callback auto-generated
 */
void Dwa_Planner::cmd_vel_pub_callback(const ros::TimerEvent& ) {
	pub_cmd_vel_pub.publish(dwaPlan( is.vars(), is.params()));
}

/**
 * Method Dwa_Planner auto-generated
 */
 Dwa_Planner::Dwa_Planner() {
	setName(ros::this_node::getName());
}

