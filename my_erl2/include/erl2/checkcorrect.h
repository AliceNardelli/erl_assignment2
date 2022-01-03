#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"


/**
 * This file defines the MoveActionInterface class.
 * MoveActionInterface is used to simulate synthetic actions (non physics based simulator)
 * 
 */

#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"


namespace KCL_rosplan {

	class CheckCorrectActionInterface: public RPActionInterface
	{

	private:
                ros::ServiceClient client;
             
	public:

		/* constructor */
		CheckCorrectActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
