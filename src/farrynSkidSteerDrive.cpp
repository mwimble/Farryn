// On exception, clear queue by waiting 10ms.
// Capture speed, update last speed.

#include <fcntl.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <poll.h>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <tf/transform_broadcaster.h>
#include <unistd.h>
#include <vector>

#include <linux/usbdevice_fs.h>

#include "farrynSkidSteerDrive.h"

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg

// 7.5 inches in meters.
#define axle_width_ 0.1905
#define quad_pulse_per_meter_ 9230.0

FarrynSkidSteerDrive::FarrynSkidSteerDrive() :
    lastM1Position(0),
    lastM2Position(0),
    lastXPosition(0),
    lastYPosition(0),
    lastXVelocity(0.0),
    lastYVelocity(0.0)
    {
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] A ROS Node for FarrynSkidSteerDrive has not been initialized.");
		throw new TRoboClawException("A ROS Node for FarrynSkidSteerDrive has not been initialized.");
	}

	// if (pthread_mutex_init(&roboClawLock, NULL) != 0) {
	// 	ROS_FATAL_STREAM("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] pthread_mutex_init failed");
	// 	throw new TRoboClawException("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] pthread_mutex_init failed");
	// }

	alive = true;
	updateRate_ = 100.0;
	updatePeriod_ = 1.0 / updateRate_;
	M1_MAX_METERS_PER_SEC = 0.333;
	M2_MAX_METERS_PER_SEC = 0.333;
	MAX_SECONDS_TRAVEL = 0.5;
	portAddress = 0x80;
	MAX_COMMAND_RETRIES = 5;
	DEBUG = true;

	M1_P = 226.3538;
	M2_P = 267.1718;
	M1_I = 13.35421;
	M2_I = 14.51053;
	M1_QPPS = 2810;
	M2_QPPS = 2512;
	AXLE_WIDTH = 0;

	rosNode = new ros::NodeHandle(); //### namespace
	
	lastTime = ros::Time::now();

	rosNode->param<std::string>("cmd_vel_topic", cmdVelTopic, "/cmd_vel");
	rosNode->param<std::string>("motor_usb_port", motorUSBPort, "/dev/ttyACM0");

	clawPort = open(motorUSBPort.c_str(), O_RDWR | O_NOCTTY);
	if (clawPort == -1) {
		ROS_ERROR("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] Unable to open USB port, errno: (%d) %s", errno, strerror(errno));
		throw new TRoboClawException("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] Unable to open USB port");
	}

	// if (ioctl(clawPort, USBDEVFS_RESET, 0) == -1) {
	// 	ROS_ERROR("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] Unable to reset USB port, error (%d) %s", errno, strerror(errno));
	// 	throw new TRoboClawException("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] Unable to reset USB port");
 //  	}

  	lock.l_type = F_WRLCK;
	lock.l_whence = SEEK_SET;
	lock.l_start = 0;
	lock.l_len = 0;
	lock.l_pid = getpid();
	if (fcntl(clawPort, F_SETLK, &lock) != 0) {
		ROS_ERROR("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] Device is already locked");
		throw new TRoboClawException("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] Device is already locked");
	}

    // Fetch the current port settings.
	struct termios portOptions;
	tcgetattr(clawPort, &portOptions);
	memset(&portOptions.c_cc, 0, sizeof(portOptions.c_cc));

    // Flush the port's buffers (in and out) before we start using it.
    tcflush(clawPort, TCIOFLUSH);

    // Set the input and output baud rates.
    //cfsetispeed(&portOptions, B115200);
    //cfsetospeed(&portOptions, B115200);

    // c_cflag contains a few important things- CLOCAL and CREAD, to prevent
    //   this program from "owning" the port and to enable receipt of data.
    //   Also, it holds the settings for number of data bits, parity, stop bits,
    //   and hardware flow control. 
    portOptions.c_cflag = CS8 | CLOCAL | CREAD;
    portOptions.c_iflag = IGNPAR;
    portOptions.c_oflag = 0;
    portOptions.c_lflag = 0;

    // Now that we've populated our options structure, let's push it back to the system.
    if (tcsetattr(clawPort, TCSANOW, &portOptions) < 0) {
		ROS_ERROR("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] Unable to set terminal options (tcsetattr)");
		throw new TRoboClawException("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] Unable to set terminal options (tcsetattr)");
    }

    // Flush the buffer one more time.
    tcflush(clawPort, TCIOFLUSH);
    flush();
    usleep(15000);
	stop();
	
	roboClawStatusReaderThread = boost::thread(boost::bind(&FarrynSkidSteerDrive::roboClawStatusReader, this));
	roboClawMotorControllerThread = boost::thread(boost::bind(&FarrynSkidSteerDrive::robotMotorController, this));

	setM1PID(M1_P, M1_I, 0, M1_QPPS);
	setM2PID(M2_P, M2_I, 0, M2_QPPS);

	ros::SubscribeOptions so = 
		ros::SubscribeOptions::create<geometry_msgs::Twist>(
				cmdVelTopic,
				1,
				boost::bind(&FarrynSkidSteerDrive::cmdVelCallback, this, _1),
				ros::VoidPtr(),
				&queue
			);
	cmdVelSubscriber = rosNode->subscribe(so);
	callbackQueueThread = boost::thread(boost::bind(&FarrynSkidSteerDrive::queueThread, this));

	ROS_INFO("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] Starting");
	ROS_INFO("[FarrynSkidSteerDrive::FarrynSkidSteerDrive] getLogicBatteryLevel %f", getLogicBatteryLevel());
}

FarrynSkidSteerDrive::~FarrynSkidSteerDrive() {
	roboClawLock.lock();
	if (clawPort) {
		flush();
		close(clawPort);
	}

	roboClawLock.unlock();
}

void FarrynSkidSteerDrive::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
	ROS_INFO("[FarrynSkidSteerDrive::cmdVelCallback] cmd_msg.linear.x: %f, cmd_msg.angular.z: %f", cmd_msg->linear.x, cmd_msg->angular.z);
	geometry_msgs::Twist copy;
	copy.linear.x = cmd_msg->linear.x;
	copy.angular.z = cmd_msg->angular.z;
	twistQueue.Produce(copy);
	// float velocity = cmd_msg->linear.x;
	// float angle = cmd_msg->angular.z;
	// drive(velocity, angle);
}

void FarrynSkidSteerDrive::configCallback(farryn_controller::FarrynConfig &config, uint32_t level) {
	ROS_INFO("[FarrynSkidSteerDrive::configCallback]\n\tlevel: %d\n\tcmd_vel_topic: %s\n\tKB_velocity:  %f\n\tmotorUSBPort: %s\n\t*updatePeriod: %f", 
			 level,
			 config.cmd_vel_topic.c_str(),
			 config.KP_velocity,
			 config.motor_usb_port.c_str(),
			 updatePeriod_);
	if (M1_P != config.M1_P) {
		M1_P = config.M1_P;
		ROS_INFO("[FarrynSkidSteerDrive::configCallback] setting new M1_P value: %f", M1_P);
		setM1PID(M1_P, M1_I, 0, M1_QPPS);
	}
	
	if (M1_I != (float) config.M1_I) {
		M1_I = (float) config.M1_I;
		ROS_INFO("[FarrynSkidSteerDrive::configCallback] setting new M1_I value: %f", M1_I);
		setM1PID(M1_I, M1_I, 0, M1_QPPS);
	}
	
	if (M1_QPPS != (float) config.M1_QPPS) {
		M1_QPPS = (float) config.M1_QPPS;
		ROS_INFO("[FarrynSkidSteerDrive::configCallback] setting new M1_QPPS value: %d", M1_QPPS);
		setM1PID(M1_P, M1_I, 0, M1_QPPS);
	}

	if (M2_P != (float) config.M2_P) {
		M2_P = (float) config.M2_P;
		ROS_INFO("[FarrynSkidSteerDrive::configCallback] setting new M2_P value: %f", M2_P);
		setM1PID(M2_P, M2_I, 0, M2_QPPS);
	}
	
	if (M2_I != (float) config.M2_I) {
		M2_I = (float) config.M2_I;
		ROS_INFO("[FarrynSkidSteerDrive::configCallback] setting new M2_I value: %f", M2_I);
		setM1PID(M2_P, M2_I, 0, M2_QPPS);
	}
	
	if (M2_QPPS != config.M2_QPPS) {
		M2_QPPS = config.M2_QPPS;
		ROS_INFO("[FarrynSkidSteerDrive::configCallback] setting new M2_QPPS value: %d", M2_QPPS);
		setM2PID(M2_P, M2_I, 0, M2_QPPS);
	}

	if (AXLE_WIDTH != config.AXLE_WIDTH) {
		AXLE_WIDTH = config.AXLE_WIDTH;
		ROS_INFO("[FarrynSkidSteerDrive::configCallback] setting new AXLE_WIDTH value: %d", AXLE_WIDTH);
	}
}

void FarrynSkidSteerDrive::drive(float velocity, float angle) {
	boost::mutex::scoped_lock scoped_lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::drive] velocity: %f, angle: %f", velocity, angle);
	int32_t m1_speed;
	int32_t m2_speed;
	setVelocities(velocity, angle, &m1_speed, &m2_speed);
	lastXVelocity = velocity - (AXLE_WIDTH / 2.0) * angle;
	lastYVelocity = velocity + (AXLE_WIDTH / 2.0) * angle;
	/*
	if (left_velocity > M1_MAX_METERS_PER_SEC) left_velocity = M1_MAX_METERS_PER_SEC;
	if (right_velocity > M2_MAX_METERS_PER_SEC) right_velocity = M2_MAX_METERS_PER_SEC;

	unsigned long m1_speed = left_velocity / M1_MAX_METERS_PER_SEC * M1_QPPS;
	unsigned long m2_speed = right_velocity / M2_MAX_METERS_PER_SEC * M2_QPPS;
	*/

	int32_t m1_max_distance = M1_MAX_METERS_PER_SEC * m1_speed * MAX_SECONDS_TRAVEL; // Limit travel.
	int32_t m2_max_distance = M2_MAX_METERS_PER_SEC * m2_speed * MAX_SECONDS_TRAVEL; // Limit travel.
	ROS_INFO_STREAM("[FarrynSkidSteerDrive::drive] ---- command: " << MIXEDSPEEDDIST
		 << ", drive velocity: " << velocity
		 << ", angle: " << angle
		 << ", m1_speed: " << m1_speed
		 << ", m1_max_distance: " << m1_max_distance
		 << ", m2_speed: " << m2_speed
		 << ", m2_max_distance: " << m2_max_distance);
	int retry;
	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			writeN(true, 19, portAddress, MIXEDSPEEDDIST,
				   SetDWORDval(m1_speed),
				   SetDWORDval(m1_max_distance),
				   SetDWORDval(m2_speed),
				   SetDWORDval(m2_max_distance),
				   1 /* Cancel any previous command */
				   );
            ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::drive] return");
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::drive] Exception: %s, retry number %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::drive] RETRY COUNT EXCEEDED");
    ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::drive] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::drive] RETRY COUNT EXCEEDED");
}

void FarrynSkidSteerDrive::flush() {
	int retval = tcflush(clawPort, TCIOFLUSH);
	if (retval != 0) {
		ROS_ERROR("[FarrynSkidSteerDrive::flush] Unable to flush device");
		throw new TRoboClawException("[FarrynSkidSteerDrive::flush] Unable to flush device");
	}
}

unsigned short FarrynSkidSteerDrive::get2ByteCommandResult(uint8_t command) {
	ROS_DEBUG_COND(DEBUG, "[FarrynSkidSteerDrive::get2ByteCommandResult] command: 0x%X", command);
	uint8_t checkSum = portAddress + command;

	writeN(false, 2, portAddress, command);
	unsigned short result = 0;
	uint8_t datum = readByteWithTimeout();
	checkSum += datum;
	result |= datum << 8;
	datum = readByteWithTimeout();
	checkSum += datum;
	result |= datum;

	uint8_t responseChecksum = readByteWithTimeout();
	if ((checkSum & 0x7F) != (responseChecksum & 0x7F)) {
		if (DEBUG) ROS_ERROR("[FarrynSkidSteerDrive::get2ByteCommandResult] Expected checkSum of: 0x%X, but got: 0x%X", int(checkSum), int(responseChecksum));
		throw new TRoboClawException("[FarrynSkidSteerDrive::get2ByteCommandResult] INVALID CHECKSUM");
	}

	return result;
}

FarrynSkidSteerDrive::EncodeResult FarrynSkidSteerDrive::getEncoderCommandResult(uint8_t command) {
	ROS_DEBUG_COND(DEBUG, "[FarrynSkidSteerDrive::getEncoderCommandResult] command: 0x%X", command);
	uint8_t checkSum = portAddress + command;

	writeN(false, 2, portAddress, command);
	EncodeResult result = {0, 0};
	uint8_t datum = readByteWithTimeout();
	checkSum += datum;
	result.value |= datum << 24;
	datum = readByteWithTimeout();
	checkSum += datum;
	result.value |= datum << 16;
	datum = readByteWithTimeout();
	checkSum += datum;
	result.value |= datum << 8;
	datum = readByteWithTimeout();
	checkSum += datum;
	result.value |= datum;
	datum = readByteWithTimeout();
	checkSum += datum;
	result.status = datum;

	ROS_DEBUG_COND(DEBUG, "[FarrynSkidSteerDrive::::EncodeResult] command: %d, value: %d, status: 0x%x", command, result.value, result.status);

	uint8_t responseChecksum = readByteWithTimeout();
	if ((checkSum & 0x7F) != (responseChecksum & 0x7F)) {
		ROS_ERROR("[FarrynSkidSteerDrive::EncodeResult] Expected checkSum of: 0x%X, but got: 0x%X", int(checkSum), int(responseChecksum));
		throw new TRoboClawException("[FarrynSkidSteerDrive::::EncodeResult] INVALID CHECKSUM");
	}

	return result;
}

uint16_t FarrynSkidSteerDrive::getErrorStatus() {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::getErrorStatus]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint8_t checkSum = portAddress + GETERROR;

			writeN(false, 2, portAddress, GETERROR);
			uint8_t datum = readByteWithTimeout() << 8;
			checkSum += datum;
			datum += readByteWithTimeout();
			checkSum += datum;
			uint8_t responseChecksum = readByteWithTimeout();
			// if ((checkSum & 0x7F) != (responseChecksum & 0x7F)) {
			// 	ROS_ERROR("[FarrynSkidSteerDrive::getErrorStatus] Expected checkSum of: 0x%X, but got: 0x%X", int(checkSum), int(responseChecksum));
			// 	throw new TRoboClawException("[FarrynSkidSteerDrive::getErrorStatus] INVALID CHECKSUM");
			// }

        	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getErrorStatus] return");
			return datum;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::getErrorStatus] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::getErrorStatus] RETRY COUNT EXCEEDED");
    ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getErrorStatus] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::getErrorStatus] RETRY COUNT EXCEEDED");
}

float FarrynSkidSteerDrive::getLogicBatteryLevel() {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::getLogicBatteryLevel]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
        	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getLogicBatteryLevel] return");
			return ((float) get2ByteCommandResult(GETLBATT)) / 10.0;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::getLogicBatteryLevel] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::getErrorStatus] RETRY COUNT EXCEEDED");
    ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getLogicBatteryLevel] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::getErrorStatus] RETRY COUNT EXCEEDED");
}

uint32_t FarrynSkidSteerDrive::getLongCont(uint8_t& checksum) {
	ROS_DEBUG_COND(DEBUG, "[FarrynSkidSteerDrive::getLongCont] checksum: 0x%X", checksum);
	uint32_t result = 0;
	uint8_t datum = readByteWithTimeout();
	checksum += datum;
	result |= datum << 24;
	datum = readByteWithTimeout();
	checksum += datum;
	result |= datum << 16;
	datum = readByteWithTimeout();
	checksum += datum;
	result |= datum << 8;
	datum = readByteWithTimeout();
	checksum += datum;
	result |= datum;
	return result;
}

int32_t FarrynSkidSteerDrive::getM1Encoder() {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::getM1Encoder]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			EncodeResult result = getEncoderCommandResult(GETM1ENC);
            ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getM1Encoder] return");
			return result.value;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::getM1Encoder] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::getM1Encoder] RETRY COUNT EXCEEDED");
    ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getM1Encoder] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::getM1Encoder] RETRY COUNT EXCEEDED");
}

int32_t FarrynSkidSteerDrive::getM2Encoder() {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::getM2Encoder]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			EncodeResult result = getEncoderCommandResult(GETM2ENC);
            ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getM2Encoder] return");
			return result.value;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::getM2Encoder] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::getM2Encoder] RETRY COUNT EXCEEDED");
    ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getM2Encoder] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::getM2Encoder] RETRY COUNT EXCEEDED");
}

float FarrynSkidSteerDrive::getMainBatteryLevel() {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::getMainBatteryLevel]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
        	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getMainBatteryLevel] return");
			return ((float) get2ByteCommandResult(GETMBATT)) / 10.0;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::getMainBatteryLevel] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::getMainBatteryLevel] RETRY COUNT EXCEEDED");
    ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getMainBatteryLevel] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::getMainBatteryLevel] RETRY COUNT EXCEEDED");
}

FarrynSkidSteerDrive::TMotorCurrents FarrynSkidSteerDrive::getMotorCurrents() {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::getMotorCurrents]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			TMotorCurrents result;
			unsigned long currentPair = getUlongCommandResult(GETCURRENTS);
			result.m1Current = ((int16_t) (currentPair >> 16)) * 0.010;
			result.m2Current = ((int16_t) (currentPair & 0xFFFF)) * 0.010;
        	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getMotorCurrents] return");
			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::getMotorCurrents] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::getMotorCurrents] RETRY COUNT EXCEEDED");
    ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getMotorCurrents] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::getMotorCurrents] RETRY COUNT EXCEEDED");
}

unsigned long FarrynSkidSteerDrive::getUlongCommandResult(uint8_t command) {
	ROS_DEBUG_COND(DEBUG, "[FarrynSkidSteerDrive::getUlongCommandResult command: 0x%X", command);
	uint8_t checkSum = portAddress + command;

	writeN(false, 2, portAddress, command);
	unsigned long result = 0;
	uint8_t datum = readByteWithTimeout();
	checkSum += datum;
	result |= datum << 24;
	datum = readByteWithTimeout();
	checkSum += datum;
	result |= datum << 16;
	datum = readByteWithTimeout();
	checkSum += datum;
	result |= datum << 8;
	datum = readByteWithTimeout();
	checkSum += datum;
	result |= datum;

	uint8_t responseChecksum = readByteWithTimeout();
	if ((checkSum & 0x7F) != (responseChecksum & 0x7F)) {
		ROS_ERROR("[FarrynSkidSteerDrive::getUlongCommandResult] Expected checkSum of: 0x%X, but got: 0x%X", int(checkSum), int(responseChecksum));
		throw new TRoboClawException("[FarrynSkidSteerDrive::getUlongCommandResult] INVALID CHECKSUM");
	}

	return result;
}

FarrynSkidSteerDrive::ULongPair FarrynSkidSteerDrive::getULongPairCommandResult(uint8_t command) {
	ROS_DEBUG_COND(DEBUG, "[FarrynSkidSteerDrive::getULongPairCommandResult] command: 0x%X", command);
	uint8_t checksum = portAddress + command;

	writeN(false, 2, portAddress, command);
	uint32_t result1 = getLongCont(checksum);
	uint32_t result2 = getLongCont(checksum);

	uint8_t responseChecksum = readByteWithTimeout();
	if ((checksum & 0x7F) != (responseChecksum & 0x7F)) {
		ROS_ERROR("[FarrynSkidSteerDrive::getULongPairCommandResult] Expected checkSum of: 0x%X, but got: 0x%X", int(checksum), int(responseChecksum));
		throw new TRoboClawException("[FarrynSkidSteerDrive::getULongPairCommandResult] INVALID CHECKSUM");
	}

	ULongPair result;
	result.p1 = result1;
	result.p2 = result2;
	return result;
}

FarrynSkidSteerDrive::TPIDQ FarrynSkidSteerDrive::getM1PIDQ() {
	TPIDQ result;
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::getM1PIDQ]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint8_t checksum = portAddress + GETM1PID;

			writeN(false, 2, portAddress, GETM1PID);
			result.p = (int32_t) getLongCont(checksum);
			result.i = (int32_t) getLongCont(checksum);
			result.d = (int32_t) getLongCont(checksum);
			result.q = (int32_t) getLongCont(checksum);
			uint8_t responseChecksum = readByteWithTimeout();
			if ((checksum & 0x7F) != (responseChecksum & 0x7F)) {
				ROS_ERROR("[FarrynSkidSteerDrive::getM1PIDQ] Expected checksum of: 0x%X, but got: 0x%X", int(checksum), int(responseChecksum));
				throw new TRoboClawException("[FarrynSkidSteerDrive::getM1PIDQ] INVALID CHECKSUM");
			}

        	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getM1PIDQ] return");
			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::getM1PIDQ] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::getM1PIDQ] RETRY COUNT EXCEEDED");
    ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getM1PIDQ] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::getM1PIDQ] RETRY COUNT EXCEEDED");
}

FarrynSkidSteerDrive::TPIDQ FarrynSkidSteerDrive::getM2PIDQ() {
	TPIDQ result;
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::getM2PIDQ]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint8_t checksum = portAddress + GETM2PID;

			writeN(false, 2, portAddress, GETM2PID);
			result.p = (int32_t) getLongCont(checksum);
			result.i = (int32_t) getLongCont(checksum);
			result.d = (int32_t) getLongCont(checksum);
			result.q = (int32_t) getLongCont(checksum);
			uint8_t responseChecksum = readByteWithTimeout();
			if ((checksum & 0x7F) != (responseChecksum & 0x7F)) {
				ROS_ERROR("[FarrynSkidSteerDrive::getM2PIDQ] Expected checksum of: 0x%X, but got: 0x%X", int(checksum), int(responseChecksum));
				throw new TRoboClawException("[FarrynSkidSteerDrive::getM2PIDQ] INVALID CHECKSUM");
			}

        	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getM2PIDQ] return");
			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::getM2PIDQ] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::getM2PIDQ] RETRY COUNT EXCEEDED");
	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getM2PIDQ] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::getM2PIDQ] RETRY COUNT EXCEEDED");
}

string FarrynSkidSteerDrive::getVersion() {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::getVersion]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint8_t checksum = portAddress + GETVERSION;

			writeN(false, 2, portAddress, GETVERSION);

			uint8_t i;
			uint8_t data;
			stringstream version;

			for (i = 0; i < 32; i++) {
				if (data != -1) {
					data = readByteWithTimeout();
					version << (char) data;
					checksum += data;
					if (data == 0) {
						uint8_t responseChecksum = readByteWithTimeout();
						if ((checksum & 0x7F) != (responseChecksum & 0x7F)) {
							ROS_ERROR("[FarrynSkidSteerDrive::getVersion] Expected checkSum of: 0x%X, but got: 0x%X", int(checksum), int(responseChecksum));
							throw new TRoboClawException("[FarrynSkidSteerDrive::getVersion] INVALID CHECKSUM");
						}

                    	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getVersion] return");
						return version.str();
					}
				}
			}

			ROS_ERROR("[FarrynSkidSteerDrive::getVersion] unexpected long string");
			throw new TRoboClawException("[FarrynSkidSteerDrive::getVersion] unexpected long string");
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::getVersion] Exception: %s, retry number: %d", e->what(), retry);
		}
	}


	ROS_ERROR("[FarrynSkidSteerDrive::getVersion] RETRY COUNT EXCEEDED");
	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::getVersion] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::getVersion] RETRY COUNT EXCEEDED");
}

void FarrynSkidSteerDrive::queueThread() {
	static const double timeout = 0.01;

	while (alive && rosNode->ok()) {
		queue.callAvailable(ros::WallDuration(timeout));
	}
}

void FarrynSkidSteerDrive::robotMotorController() {
    while (1) {
        geometry_msgs::Twist cmd_msg;
        twistQueue.Consume(cmd_msg);
        float velocity = cmd_msg.linear.x;
        float angle = cmd_msg.angular.z;
        drive(velocity, angle);
        usleep(1000);
    }
}

void FarrynSkidSteerDrive::roboClawStatusReader() {
	ROS_INFO("FarrynSkidSteerDrive::roboClawStatusReader start");
	ros::Publisher statusPublisher = rosNode->advertise<farryn_controller::RoboClawStatus>("/RoboClawStatus", 1);
	ros::Rate rate(1);
	uint32_t counter = 0;
	roboClawStatus.firmwareVersion = getVersion();
	while (rosNode->ok()) {
		try {
			uint8_t errorStatus = getErrorStatus();
			roboClawStatus.errorStatus = errorStatus;
			roboClawStatus.stickyErrorStatus |= errorStatus;
			if (errorStatus == 0) roboClawStatus.errorString = "normal";
			else {
				stringstream errorMessage;
				if (errorStatus & 0x80) {
					errorMessage << "[Logic Battery Low] ";
				}

				if (errorStatus & 0x40) {
					errorMessage << "[Logic Battery High] ";
				}

				if (errorStatus & 0x20) {
					errorMessage << "[Main Battery Low] ";
				}

				if (errorStatus & 0x10) {
					errorMessage << "[Main Battery High] ";
				}

				if (errorStatus & 0x08) {
					errorMessage << "[Temperature] ";
				}

				if (errorStatus & 0x04) {
					errorMessage << "[E-Stop] ";
				}

				if (errorStatus & 0x02) {
					errorMessage << "[M2 OverCurrent] ";
				}

				if (errorStatus & 0x01) {
					errorMessage << "[M1 OverCurrent] ";
				}

				if (errorStatus & 0xFF00) {
					errorMessage << "[INVALID EXTRA STATUS BITS]";
				}

				roboClawStatus.errorString = errorMessage.str();
			}

			roboClawStatus.logicBatteryVoltage = getLogicBatteryLevel();
			roboClawStatus.mainBatteryVoltage = getMainBatteryLevel();
			TMotorCurrents motorCurrents = getMotorCurrents();
			roboClawStatus.m1MotorCurrent = motorCurrents.m1Current;
			roboClawStatus.m2MotorCurrent = motorCurrents.m2Current;
			
			TPIDQ pidq = getM1PIDQ();
			roboClawStatus.m1P = pidq.p / 65536.0;
			roboClawStatus.m1I = pidq.i / 65536.0;
			roboClawStatus.m1D = pidq.d / 65536.0;
			roboClawStatus.m1Qpps = pidq.q;
			
			pidq = getM2PIDQ();
			roboClawStatus.m2P = pidq.p / 65536.0;
			roboClawStatus.m2I = pidq.i / 65536.0;
			roboClawStatus.m2D = pidq.d / 65536.0;
			roboClawStatus.m2Qpps = pidq.q;

			EncodeResult encoder = getEncoderCommandResult(GETM1ENC);
			roboClawStatus.encoderM1value = encoder.value;
			roboClawStatus.encoderM1Status = encoder.status;
			lastM1Position = encoder.value;

			encoder = getEncoderCommandResult(GETM2ENC);
			roboClawStatus.encoderM2value = encoder.value;
			roboClawStatus.encoderM2Status = encoder.status;
			lastM2Position = encoder.value;
			
			lastTime = ros::Time::now();

			// stringstream ss;
			// ss << "No error, counter: " << counter++;
			// roboClawStatus.firmwareVersion = ss.str();
			statusPublisher.publish(roboClawStatus);
			rate.sleep();
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::roboClawStatusReader] Exception: %s", e->what());
		}
	}
}

uint8_t FarrynSkidSteerDrive::readByteWithTimeout() {
	ROS_DEBUG_COND(DEBUG, "[FarrynSkidSteerDrive::readByteWithTimeout]");

	struct pollfd ufd[1];
	ufd[0].fd = clawPort;
	ufd[0].events = POLLIN;

	int retval = poll(ufd, 1, 100);
	if (retval < 0) {
		ROS_ERROR("[FarrynSkidSteerDrive::readByteWithTimeout] Poll failed (%d) %s", errno, strerror(errno));
		throw new TRoboClawException("[FarrynSkidSteerDrive::readByteWithTimeout] Read error");
	} else if (retval == 0) {
		stringstream ev;
		ev << "[FarrynSkidSteerDrive::readByteWithTimeout] TIMEOUT revents: " << hex << ufd[0].revents;
		ROS_ERROR_STREAM(ev.str());
		throw new TRoboClawException("[FarrynSkidSteerDrive::readByteWithTimeout] TIMEOUT");
	} else if (ufd[0].revents & POLLERR) {
		ROS_ERROR("[FarrynSkidSteerDrive::readByteWithTimeout] Error on socket");
		throw new TRoboClawException("[FarrynSkidSteerDrive::readByteWithTimeout] Error on socket");
	} else if (ufd[0].revents & POLLIN) {
		char buffer[1];
		int bytesRead = read(clawPort, buffer, sizeof(buffer));
		if (bytesRead != 1) {
			usleep(15000); // ### Timeout/flush RoboClaw.
			ROS_ERROR("[FarrynSkidSteerDrive::readByteWithTimeout] Failed to read 1 byte, read: %d", bytesRead);
			throw TRoboClawException("[FarrynSkidSteerDrive::readByteWithTimeout] Failed to read 1 byte");
		}

		ROS_DEBUG_COND(DEBUG, "[FarrynSkidSteerDrive::readByteWithTimeout] ...byte: 0x%X", int(buffer[0]));
		return buffer[0];
	} else {
		ROS_ERROR("[FarrynSkidSteerDrive::readByteWithTimeout] Unhandled case");
		throw new TRoboClawException("[FarrynSkidSteerDrive::readByteWithTimeout] Unhandled case");
	}
}

void FarrynSkidSteerDrive::setM1PID(float p, float i, float d, uint32_t qpps) {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::setM1PID(] p: %f, i: %f, d: %f, qpps: %d", p, i, d, qpps);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint32_t kp = int(p * 65536.0); // 14834322.6368 = E25A93
			uint32_t ki = int(i * 65536.0);
			uint32_t kd = int(d * 65536.0);
			writeN(true, 18, portAddress, SETM1PID, 
				   SetDWORDval(kd),
				   SetDWORDval(kp),
				   SetDWORDval(ki),
				   SetDWORDval(qpps));
        	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::setM1PID(] return");
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::setM1PID] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::setM1PID] RETRY COUNT EXCEEDED");
	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::setM1PID(] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::setM1PID] RETRY COUNT EXCEEDED");
}

void FarrynSkidSteerDrive::setM2PID(float p, float i, float d, uint32_t qpps) {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_DEBUG_COND(DEBUG, "-----> [FarrynSkidSteerDrive::setM2PID(] p: %f, i: %f, d: %f, qpps: %d", p, i, d, qpps);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint32_t kp = int(p * 65536.0); // 14834322.6368 = E25A93
			uint32_t ki = int(i * 65536.0);
			uint32_t kd = int(d * 65536.0);
			writeN(true, 18, portAddress, SETM2PID, 
				   SetDWORDval(kd),
				   SetDWORDval(kp),
				   SetDWORDval(ki),
				   SetDWORDval(qpps));
        	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::setM2PID(] return");
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::setM2PID] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::setM2PID] RETRY COUNT EXCEEDED");
	ROS_DEBUG_COND(DEBUG, "<----- [FarrynSkidSteerDrive::setM2PID(] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::setM2PID] RETRY COUNT EXCEEDED");
}

// Command motors to a given linear and angular velocity
void FarrynSkidSteerDrive::setVelocities(double v, double w, int32_t* left_qpps, int32_t* right_qpps) {
	double wmag = fabs(w);
	double vmag = fabs(v);

    /* Limit slow turning when you have no forward velocity
    if (vmag < 0.1) {
      if (wmag < 0.15) {
        w = 0.0;
      } else if (wmag < 0.5) {
        w = copysign(0.5, w);
      }
    }*/

    // Reset error terms if large change in velocities or stopping.
    // if (fabs(state_.v_sp - v) > pid_error_reset_v_step_threshold_ || 
    //     fabs(state_.w_sp - w) > pid_error_reset_w_step_threshold_ ||
    //     (vmag < pid_error_reset_min_v_ && wmag < pid_error_reset_min_w_)) {
    //   pid_left_.reset();
    //   pid_right_.reset();
    // }

    // state_.v_sp = v;
    // state_.w_sp = w;

	double left_sp;
	double right_sp;
    vwToWheelSpeed(v, w, &left_sp, &right_sp);
    // Convert speeds to quad pulses per second
    *left_qpps = static_cast<int32_t>(round(left_sp * quad_pulse_per_meter_));
	*right_qpps = static_cast<int32_t>(round(right_sp * quad_pulse_per_meter_));

    // Compute kinematic feedforward control input
    // state_.left_duty_sp = state_.left_qpps_sp * duty_per_qpps_;
    // state_.right_duty_sp = state_.right_qpps_sp * duty_per_qpps_;
}

void FarrynSkidSteerDrive::stop() {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_INFO_STREAM("-----> [FarrynSkidSteerDrive::stop] command: " << MIXEDSPEED);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
		    lastXVelocity = 0.0;
		    lastYVelocity = 0.0;
			writeN(false/*#####true*/, 11, portAddress, MIXEDSPEED,
				SetDWORDval(0),
				SetDWORDval(0),
				1 /* Cancel any previous command */
				);
            ROS_INFO_STREAM("<----- [FarrynSkidSteerDrive::stop] return");
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[FarrynSkidSteerDrive::stop] Exception: %s, retry number: %d", e->what(), retry);
		}
	}

	ROS_ERROR("[FarrynSkidSteerDrive::stop] RETRY COUNT EXCEEDED");
	ROS_INFO_STREAM("<----- [FarrynSkidSteerDrive::stop] ERROR");
	throw new TRoboClawException("[FarrynSkidSteerDrive::stop] RETRY COUNT EXCEEDED");
}

void FarrynSkidSteerDrive::updateOdometry() {
// 	ROS_INFO("FarrynSkidSteerDrive::updateOdometry start");
// 	ros::Publisher statusPublisher = rosNode->advertise<farryn_controller::RoboClawStatus>("odom", 10);
// 	tf::TransformBroadcaster odomBroadcaster;
// 	
// 	double x = 0.0;
// 	double y = 0.0;
// 	double th = 0.0;
// 	ros::Time currentTime = ros::Time::now();
// 	
// 	ros::Rate rate(1/*#####*/);
// 	uint32_t counter = 0;
// 	roboClawStatus.firmwareVersion = getVersion();
// 	while (rosNode->ok()) {
// 		try {
// 		    ros::spinOnce();
// 		    currentTime = ros::Time::now();
// 		    double dt = (currentTime - lastTime).toSec();
// 		    //double deltaX = (
// 		} catch (TRoboClawException* e) {
// 			ROS_ERROR("[FarrynSkidSteerDrive::roboClawStatusReader] Exception: %s", e->what());
// 		}
// 	}
}

// Convert linear / angular velocity to left / right motor speeds in meters /
// second
void FarrynSkidSteerDrive::vwToWheelSpeed(double v, double w, double *left_mps, double *right_mps) {
    // Compute the differential drive speeds from the input
    *left_mps = v - (axle_width_ / 2.0) * w;
    *right_mps = v + (axle_width_ / 2.0) * w;

    /*
    // Scale the speeds to respect the wheel speed limit
    double limitk = 1.0;
    if (fabs(*left_mps) > max_wheel_vel_) {
      limitk = max_wheel_vel_ / fabs(*left_mps);
    }

    if (fabs(*right_mps) > max_wheel_vel_) {
      double rlimitk = max_wheel_vel_ / fabs(*right_mps);
      if (rlimitk < limitk) {
        limitk = rlimitk;
      }
    }

    if (limitk != 1.0) {
      *left_mps *= limitk;
      *right_mps *= limitk;
    }

    // Deal with min limits
    if (fabs(*left_mps) < min_wheel_vel_) {
      *left_mps = 0.0;
    } if (fabs(*right_mps) < min_wheel_vel_) {
      *right_mps = 0.0;
    }
    */
}

void FarrynSkidSteerDrive::writeByte(uint8_t byte) {
	ROS_DEBUG_COND(DEBUG, "[FarrynSkidSteerDrive::writeByte] byte: 0x%X", byte);
	ssize_t result = write(clawPort, &byte, 1);
	if (result != 1) {
		ROS_ERROR("[FarrynSkidSteerDrive::writeByte] Unable to write one byte, result: %d, errno: %d)", result,  errno);
		throw new TRoboClawException("[FarrynSkidSteerDrive::writeByte] Unable to write one byte");
	}
}

void FarrynSkidSteerDrive::writeN(bool sendChecksum, uint8_t cnt, ...) {
	va_list marker;
	va_start(marker, cnt);

	uint8_t data[128];
	uint16_t dataIndex = 0;

	int origFlags = fcntl(clawPort, F_GETFL, 0);
	//	fcntl(clawPort, F_SETFL, origFlags & ~O_NONBLOCK);

	uint8_t checksum = 0;
	for (uint8_t i = 0; i < cnt; i++) {
		uint8_t byte = va_arg(marker, int);
		writeByte(byte);
		//data[dataIndex++] = byte;
		checksum += byte;
	}

	va_end(marker);

	if (sendChecksum) {
		//data[dataIndex++] = (checksum & 0x7F) | 0x80;
		writeByte((checksum & 0x7F) | 0x80);
	}
	if (sendChecksum) {
		uint8_t response = readByteWithTimeout();
		if (response != 0xFF) {
			ROS_ERROR("[FarrynSkidSteerDrive::writeByte] Invalid ACK response");
			throw new TRoboClawException("[FarrynSkidSteerDrive::writeByte] Invalid ACK response");
		}
	}
}

boost::mutex FarrynSkidSteerDrive::roboClawLock;
