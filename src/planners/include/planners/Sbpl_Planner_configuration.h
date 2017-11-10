#ifndef _SBPL_PLANNER_CONFIGURATION_H
#define _SBPL_PLANNER_CONFIGURATION_H
/**
 * Auto-generated Internal State
 */
#include "ros_base/Configuration.h"
struct Variables: ros_base::VariablesBase { 
	Costmap2D costmap;
	std::string mapFrame;
	double epsilon;
	struct flags_t { 
		bool new_goal;
		bool new_plan;
		bool new_costmap;
	} flags;
	Variables () {
		mapFrame = "map";
		flags.new_goal = false;
		flags.new_plan = false;
		flags.new_costmap = false;
	};
};
struct Parameters: ros_base::ParametersBase { 
	bool replan;
	double starting_epsilon;
	double ending_epsilon;
	int dimensions;
	std::string motPrimString;
	struct vehicleParams_t { 
		std::string robotShape;
		double robot_width;
		double robot_length;
		double v_max;
		double omega_max;
	} vehicleParams;
	struct planningSetup_t { 
		std::string planner_type;
		double allocated_time;
		double initial_epsilon;
		double epsilon_decrease_step;
	} planningSetup;
	struct mapParameters_t { 
		double resolutionTolerance;
		int obsthresh;
		int cost_inscribed_thresh;
		int cost_possibly_circumscribed_thresh;
	} mapParameters;
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