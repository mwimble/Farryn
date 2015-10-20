#ifndef __FARRYN_SKID_STEER_DRIVE_H
#define __FARRYN_SKID_STEER_DRIVE_H

#include <fcntl.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <LockFreeQueue.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include "farryn_controller/FarrynConfig.h"
#include "farryn_controller/RoboClawStatus.h"

using namespace std;

class FarrynSkidSteerDrive {
public:
	struct TRoboClawException : public std::exception {
		std::string s;
		TRoboClawException(std::string ss) : s(ss) {}
		~TRoboClawException() throw() {}
		const char* what() const throw() { return s.c_str(); }
	};


	FarrynSkidSteerDrive();
	~FarrynSkidSteerDrive();

	void configCallback(farryn_controller::FarrynConfig &config, uint32_t level);

	uint16_t getErrorStatus();

	float getLogicBatteryLevel();

	float getMainBatteryLevel();

	typedef struct {
		int32_t		value;
		uint8_t		status;
	} EncodeResult;

	int32_t getM1Encoder();

	int32_t getM2Encoder();

	typedef struct {
		float m1Current;
		float m2Current;
	} TMotorCurrents;

	TMotorCurrents getMotorCurrents();

	typedef struct {
		int32_t p;
		int32_t i;
		int32_t d;
		int32_t q;
	} TPIDQ;

	TPIDQ getM1PIDQ();

	TPIDQ getM2PIDQ();

	string getVersion();

	void stop();

private:
	double	updateRate_;
	double	updatePeriod_;

	static boost::mutex roboClawLock; // Mutex for access to roboclaw via USB.
	boost::thread roboClawStatusReaderThread; // Periodically read and publish RoboClaw status.
	boost::thread roboClawMotorControllerThread; // Periodically dequeue and execute motor commands.
	LockFreeQueue<geometry_msgs::Twist> twistQueue;

	typedef struct {
		unsigned long p1;
		unsigned long p2;
	} ULongPair;

	enum {M1FORWARD = 0,
	    M1BACKWARD = 1,
	    SETMINMB = 2,
	    SETMAXMB = 3,
	    M2FORWARD = 4,
	    M2BACKWARD = 5,
	    M17BIT = 6,
	    M27BIT = 7,
	    MIXEDFORWARD = 8,
	    MIXEDBACKWARD = 9,
	    MIXEDRIGHT = 10,
	    MIXEDLEFT = 11,
	    MIXEDFB = 12,
	    MIXEDLR = 13,
	    GETM1ENC = 16,
	    GETM2ENC = 17,
	    GETM1SPEED = 18,
	    GETM2SPEED = 19,
	    RESETENC = 20,
	    GETVERSION = 21,
	    GETMBATT = 24,
	    GETLBATT = 25,
	    SETMINLB = 26,
	    SETMAXLB = 27,
	    SETM1PID = 28,
	    SETM2PID = 29,
	    GETM1ISPEED = 30,
	    GETM2ISPEED = 31,
	    M1DUTY = 32,
	    M2DUTY = 33,
	    MIXEDDUTY = 34,
	    M1SPEED = 35,
	    M2SPEED = 36,
	    MIXEDSPEED = 37,
	    M1SPEEDACCEL = 38,
	    M2SPEEDACCEL = 39,
	    MIXEDSPEEDACCEL = 40,
	    M1SPEEDDIST = 41,
	    M2SPEEDDIST = 42,
	    MIXEDSPEEDDIST = 43,
	    M1SPEEDACCELDIST = 44,
	    M2SPEEDACCELDIST = 45,
	    MIXEDSPEEDACCELDIST = 46,
	    GETBUFFERS = 47,
	    SETPWM = 48,
	    GETCURRENTS = 49,
	    MIXEDSPEED2ACCEL = 50,
	    MIXEDSPEED2ACCELDIST = 51,
	    M1DUTYACCEL = 52,
	    M2DUTYACCEL = 53,
	    MIXEDDUTYACCEL = 54,
	    GETM1PID = 55,
	    GETM2PID = 56,
	    GETERROR = 90,
	    WRITENVM = 94,
		GETM1MAXCURRENT = 135};

	int clawPort;
	char portAddress;
	int MAX_COMMAND_RETRIES;
	bool DEBUG;

	farryn_controller::RoboClawStatus 	roboClawStatus;

	float M1_P;
	float M2_P;
	float M1_I;
	float M2_I;
	unsigned long M1_QPPS;
	unsigned long M2_QPPS;
	float AXLE_WIDTH;

	float MAX_SECONDS_TRAVEL;

	float M1_MAX_METERS_PER_SEC;
	float M2_MAX_METERS_PER_SEC;

	bool alive;
	boost::thread callbackQueueThread;
	string cmdVelTopic;
	struct flock lock;
	string motorUSBPort;
	ros::CallbackQueue queue;

	ros::NodeHandle *rosNode;
	ros::Subscriber cmdVelSubscriber;

	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
	
	void drive(float velocity, float angle);

	void flush();

	unsigned short get2ByteCommandResult(uint8_t command);

	EncodeResult getEncoderCommandResult(uint8_t command);

	uint32_t getLongCont(uint8_t& checksum);

	unsigned long getUlongCommandResult(uint8_t command);

	ULongPair getULongPairCommandResult(uint8_t command);

	void queueThread();

	uint8_t readByteWithTimeout();

	void roboClawStatusReader();

    void robotMotorController();
    
	void setM1PID(float p, float i, float d, uint32_t qpps);

	void setM2PID(float p, float i, float d, uint32_t qpps);

	void setVelocities(double v, double w, uint32_t* left_qpps, uint32_t* right_qpps);

	void vwToWheelSpeed(double v, double w, double *left_mps, double *right_mps);

	void writeByte(uint8_t byte);

	void writeN(bool sendChecksum, uint8_t cnt, ...);

};

#endif
