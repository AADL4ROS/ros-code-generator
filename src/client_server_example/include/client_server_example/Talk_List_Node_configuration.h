#ifndef _TALK_LIST_NODE_CONFIGURATION_H
#define _TALK_LIST_NODE_CONFIGURATION_H
/**
 * Auto-generated Internal State
 */
#include "ros_base/Configuration.h"
#include "ros_base/tf_interface.h"
struct Variables: ros_base::VariablesBase { 
	double publisherFrequency;
	Variables () {
		publisherFrequency = 0;
	};
};
struct Parameters: ros_base::ParametersBase { 
	std::string stringName;
	double testReal;
	struct traj_t { 
		double test1;
		int test2;
	} traj;
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