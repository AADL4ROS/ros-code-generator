#ifndef _DWA_PLANNER_CONFIGURATION_H
#define _DWA_PLANNER_CONFIGURATION_H
/**
 * Auto-generated Internal State
 */
#include "ros_base/Configuration.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"

struct Variables: ros_base::VariablesBase { 
	Costmap2D costmap;
	std::string mapFrame;
	nav_msgs::Path current_path;
	geometry_msgs::Pose current_goal;
	Variables () {
		mapFrame = "map";
	};
};
struct Parameters: ros_base::ParametersBase { 
	struct robotConfiguration_t { 
		double acc_lim_x;
		double acc_lim_y;
		double acc_lim_th;
		double max_trans_vel;
		double min_trans_vel;
		double max_vel_x;
		double min_vel_x;
		double max_vel_y;
		double min_vel_y;
		double max_rot_vel;
		double min_rot_vel;
	} robotConfiguration;
	struct goalTolerance_t { 
		double yaw_goal_tolerance;
		double xy_goal_tolerance;
		bool latch_xy_goal_tolerance;
	} goalTolerance;
	struct forwardSimulation_t { 
		double sim_time;
		double sim_granularity;
		int vx_samples;
		int vy_samples;
		int vth_samples;
		double controller_frequency;
	} forwardSimulation;
	struct trajectoryScoring_t { 
		double path_distance_bias;
		double goal_distance_bias;
		double occdist_scale;
		double forward_point_distance;
		double stop_time_buffer;
		double scaling_speed;
		double max_scaling_factor;
	} trajectoryScoring;
	double oscillation_reset_dist;
	bool prune_plan;
};
typedef std::shared_ptr < const Parameters > Parameters_ptr;
typedef std::shared_ptr < Variables > Variables_ptr;
class InternalState: ros_base::InternalStateBase {
public:
	Variables_ptr vars() {
		return std::static_pointer_cast < Variables > (_vars);
	};

	Parameters_ptr params() const {
		return std::static_pointer_cast < const Parameters > (_params);
	};

	void initialize (ros_base::ParametersBase * p = NULL) {
		_params = std::make_shared < const Parameters > (*static_cast < Parameters * > (p));
		_vars = std::make_shared < Variables > ();
	}
};
#endif