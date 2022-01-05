#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"


/**
 * This file defines the MoveActionInterface class.
 * MoveActionInterface is used to simulate synthetic actions (non physics based simulator)
 * 
 */



namespace KCL_rosplan {

	class PerceiveHintsActionInterface: public RPActionInterface
	{

	private:
                //ros::ServiceClient client;
	public:

		/* constructor */
		PerceiveHintsActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

