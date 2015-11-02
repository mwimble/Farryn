#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <stdint.h>
#include "farrynSkidSteerDrive.h"
#include "farryn_controller/FarrynConfig.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "farryn_controller");
	FarrynSkidSteerDrive*  farrynSkidSteerDrive = new FarrynSkidSteerDrive();

	dynamic_reconfigure::Server<farryn_controller::FarrynConfig> server;
 	dynamic_reconfigure::Server<farryn_controller::FarrynConfig>::CallbackType f;
	f = boost::bind(&FarrynSkidSteerDrive::configCallback, farrynSkidSteerDrive, _1, _2);
	server.setCallback(f);

	ROS_INFO("Starting to spin...");

	ros::Rate r(1);
	float old_batt = 0.0;
	int32_t old_m1_enc = 0;
	int32_t old_m2_enc = 0;

	while (ros::ok()) {
		try {
			if ((old_batt != farrynSkidSteerDrive->getLogicBatteryLevel()) ||
				(old_m1_enc != farrynSkidSteerDrive->getM1Encoder()) ||
				(old_m2_enc != farrynSkidSteerDrive->getM2Encoder())
				) {
				ROS_INFO("[farrynSkidSteerDrive] m1 enc: %d, m2 enc: %d, batt: %f", 
						 farrynSkidSteerDrive->getM1Encoder(),
						 farrynSkidSteerDrive->getM2Encoder(),
						 farrynSkidSteerDrive->getLogicBatteryLevel());
				old_batt = farrynSkidSteerDrive->getLogicBatteryLevel();
				old_m1_enc = farrynSkidSteerDrive->getM1Encoder();
				old_m2_enc = farrynSkidSteerDrive->getM2Encoder();
			}

			ros::spinOnce();
			r.sleep();
		} catch(FarrynSkidSteerDrive::TRoboClawException* e) {
			ROS_ERROR("[MAIN] Exception: %s", e->what());
		} catch(...) {
		    ROS_ERROR("[MAIN] Unhangled exception");
        }
	}

	return 0;
}