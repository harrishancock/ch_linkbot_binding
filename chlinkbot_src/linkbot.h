/*Ch version of the header file*/

#ifndef LINKBOT_WRAPPER_HPP_
#define LINKBOT_WRAPPER_HPP_

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#define sleep(x) Sleep((x)*1000)
#endif

typedef enum robotJoints_e {
  ROBOT_ZERO,
  JOINT1,
  JOINT2,
  JOINT3,
  JOINT4,
  ROBOT_NUM_JOINTS = 4
} robotJointId_t;

typedef enum robotJointState_e
{
    ROBOT_NEUTRAL = 0,
    ROBOT_FORWARD,
    ROBOT_BACKWARD,
    ROBOT_HOLD,
    ROBOT_POSITIVE,
    ROBOT_NEGATIVE,
    ROBOT_ACCEL,
} robotJointState_t;

struct LinkbotImpl;
/*Linkbot I*/
class CLinkbotI {
    public:
        CLinkbotI();
        ~CLinkbotI();
        int connect();
		int connectWithSerialID(const char* id);
		void disconnect();

        /* GETTERS */

        void getAccelerometerData(double &x, double &y, double &z); 
		void getDistance(double &distance, double radius);
        void getJointAngle(robotJointId_t id, double &angle);
        void getJointAngles(double &angle1, double &angle2, double &angle3);
		void getJointAngleInstant(robotJointId_t id, double &angle);
        void getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
        void getJointSpeed(robotJointId_t id, double &speed);
        void getJointSpeedRatio(robotJointId_t id, double &ratio);
        void getJointSpeeds(double &speed1, double &speed2, double &speed3);
        void getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		void getLEDColorRGB(int &r, int &g, int &b);
		void getLEDColor(string_t &color);

        /* SETTERS */
        void setBuzzerFrequency(int frequency, double time);
		void setBuzzerFrequencyOn(int frequency);
		void setBuzzerFrequencyOff();
		void setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
        void setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
        void setJointSpeed(robotJointId_t id, double speed);
        void setJointSpeeds(double speed1, double speed2, double speed3);
        void setJointSpeedRatio(robotJointId_t id, double ratio);
        void setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
        void setJointPower(robotJointId_t id, int power);
		void setLEDColorRGB(int r, int g, int b);
		void setLEDColor(char *color);
        void setMotorPowers(double p1, double p2, double p3);
        void setMovementStateNB( robotJointState_t dir1,
                robotJointState_t dir2,
                robotJointState_t dir3);
        void setMovementStateTime( robotJointState_t dir1,
                robotJointState_t dir2,
                robotJointState_t dir3,
                double seconds);
        void setMovementStateTimeNB( robotJointState_t dir1,
                robotJointState_t dir2,
                robotJointState_t dir3,
                double seconds);
		void setSpeed(double speed, double radius);

        /* MOVEMENT */

		void closeGripper();
		void closeGripperNB();
		void driveBackward(double angle);
        void driveBackwardNB(double angle);
	    void driveDistance(double distance, double radius);
        void driveDistanceNB(double distance, double radius);
	    void driveForeverNB();
	    void driveForward(double angle);
        void driveForwardNB(double angle);
		void driveTime(double seconds);
		void holdJoint(robotJointId_t id);
		void holdJoints();
		int isMoving();
		void moveForeverNB();
		void moveJoint(robotJointId_t id, double angle);
	    void moveJointNB(robotJointId_t id, double angle);
	    void moveJointForeverNB(robotJointId_t id);
		void moveJointTime(robotJointId_t id, double time);
		void moveJointWait(robotJointId_t id);
        void move(double j1, double j2, double j3);
        void moveNB(double j1, double j2, double j3);
        void moveWait();
		void moveJointTo(robotJointId_t id, double angle);
		void moveJointToNB(robotJointId_t id, double angle);
		void moveJointToByTrackPos(robotJointId_t id, double angle);
        void moveJointToByTrackPosNB(robotJointId_t id, double angle);
		void moveTime(double time);
		void moveTo(double angle1, double angle2, double angle3);
		void moveToNB(double angle1, double angle2, double angle3);
		void moveToByTrackPos(double angle1, double angle2, double angle3);
        void moveToByTrackPosNB(double angle1, double angle2, double angle3);
		void moveToZero();
		void moveToZeroNB();
		void openGripper(double angle);
		void openGripperNB(double angle);
		void relaxJoint(robotJointId_t id);
	    void relaxJoints();
		void resetToZero();
        void resetToZeroNB();
        void stop();
        void stopOneJoint(robotJointId_t id);
		void turnLeft(double angle, double radius, double tracklength);
        void turnLeftNB(double angle, double radius, double tracklength);
        void turnRight(double angle, double radius, double tracklength);
        void turnRightNB(double angle, double radius, double tracklength);

        /* MISC */
        void enableButtonCallback(void* userdata, void (*buttonCallback)(void* data, int button, int buttonDown));
        void disableButtonCallback();
		void delaySeconds(int seconds);
		void systemTime(double &time);

        LinkbotImpl *m;

        static void *g_chlinkbot_dlhandle;
        static int g_chlinkbot_dlcount;

        private:
        double mMaxSpeed;
        
};

/*Linkbot L*/
class CLinkbotL {
    public:
        CLinkbotL();
        ~CLinkbotL();
        int connect();
		int connectWithSerialID(const char* id);
		void disconnect();

        /* GETTERS */

        void getAccelerometerData(double &x, double &y, double &z); 
        void getJointAngle(robotJointId_t id, double &angle);
        void getJointAngles(double &angle1, double &angle2, double &angle3);
		void getJointAngleInstant(robotJointId_t id, double &angle);
        void getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
        void getJointSpeed(robotJointId_t id, double &speed);
        void getJointSpeedRatio(robotJointId_t id, double &ratio);
        void getJointSpeeds(double &speed1, double &speed2, double &speed3);
        void getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		void getLEDColorRGB(int &r, int &g, int &b);
		void getLEDColor(string_t &color);

        /* SETTERS */
        void setBuzzerFrequency(int frequency, double time);
		void setBuzzerFrequencyOn(int frequency);
		void setBuzzerFrequencyOff();
		void setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
        void setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
        void setJointSpeed(robotJointId_t id, double speed);
        void setJointSpeeds(double speed1, double speed2, double speed3);
        void setJointSpeedRatio(robotJointId_t id, double ratio);
        void setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
        void setJointPower(robotJointId_t id, int power);
		void setLEDColorRGB(int r, int g, int b);
		void setLEDColor(char *color);
        void setMotorPowers(double p1, double p2, double p3);
        void setMovementStateNB( robotJointState_t dir1,
                robotJointState_t dir2,
                robotJointState_t dir3);
        void setMovementStateTime( robotJointState_t dir1,
                robotJointState_t dir2,
                robotJointState_t dir3,
                double seconds);
        void setMovementStateTimeNB( robotJointState_t dir1,
                robotJointState_t dir2,
                robotJointState_t dir3,
                double seconds);

        /* MOVEMENT */

		void holdJoint(robotJointId_t id);
		void holdJoints();
		int isMoving();
		void moveForeverNB();
		void moveJoint(robotJointId_t id, double angle);
	    void moveJointNB(robotJointId_t id, double angle);
	    void moveJointForeverNB(robotJointId_t id);
		void moveJointTime(robotJointId_t id, double time);
        void move(double j1, double j2, double j3);
        void moveNB(double j1, double j2, double j3);
        void moveWait();
		void moveJointTo(robotJointId_t id, double angle);
		void moveJointToNB(robotJointId_t id, double angle);
		void moveJointToByTrackPos(robotJointId_t id, double angle);
        void moveJointToByTrackPosNB(robotJointId_t id, double angle);
		void moveJointWait(robotJointId_t id);
		void moveTime(double time);
		void moveTo(double angle1, double angle2, double angle3);
		void moveToNB(double angle1, double angle2, double angle3);
		void moveToByTrackPos(double angle1, double angle2, double angle3);
        void moveToByTrackPosNB(double angle1, double angle2, double angle3);
		void moveToZero();
		void moveToZeroNB();
		void relaxJoint(robotJointId_t id);
	    void relaxJoints();
		void resetToZero();
        void resetToZeroNB();
        void stop();
        void stopOneJoint(robotJointId_t id);

        /* MISC */
        void enableButtonCallback(void* userdata, void (*buttonCallback)(void* data, int button, int buttonDown));
        void disableButtonCallback();
		void delaySeconds(int seconds);
		void systemTime(double &time);

        LinkbotImpl *m;

        static void *g_chlinkbot_dlhandle;
        static int g_chlinkbot_dlcount;

        private:
        double mMaxSpeed;
        
};

/*GroupI*/
class CLinkbotIGroup {

public:
	CLinkbotIGroup();
	~CLinkbotIGroup();
	void addRobot(char* serialID);
	void connect();
	
	/* MOVEMENT FUNCTIONS */
	void closeGripper();
	void closeGripperNB();
	void driveBackward(double angle);
    void driveBackwardNB(double angle);
	void driveDistance(double distance, double radius);
    void driveDistanceNB(double distance, double radius);
	void driveForeverNB();
	void driveForward(double angle);
    void driveForwardNB(double angle);
	void driveTime(double time);
	void driveTimeNB(double time);
	void holdJoint(robotJointId_t id);
    void holdJoints();
	int isMoving();
	void move(double j1, double j2, double j3);
	void moveForeverNB();
	void moveJoint(robotJointId_t id, double angle);
	void moveJointNB(robotJointId_t id, double angle);
	void moveJointForeverNB(robotJointId_t id);
	void moveJointTo(robotJointId_t id, double angle);
	void moveJointToNB(robotJointId_t id, double angle);
	void moveJointToByTrackPos(robotJointId_t id, double angle);
    void moveJointToByTrackPosNB(robotJointId_t id, double angle);
	void moveJointWait(robotJointId_t id);
    void moveNB(double j1, double j2, double j3);
	void moveTo(double angle1, double angle2, double angle3);
	void moveToNB(double angle1, double angle2, double angle3);
	void moveToByTrackPos(double angle1, double angle2, double angle3);
    void moveToByTrackPosNB(double angle1, double angle2, double angle3);
    void moveToZero();
    void moveToZeroNB();
	void moveWait();
	void openGripper(double angle);
    void openGripperNB(double angle);
	void relaxJoint(robotJointId_t id);
	void relaxJoints();
	void resetToZero();
    void resetToZeroNB();
    void stop();
	void turnLeft(double angle, double radius, double tracklength);
    void turnLeftNB(double angle, double radius, double tracklength);
    void turnRight(double angle, double radius, double tracklength);
    void turnRightNB(double angle, double radius, double tracklength);

	/* SET FUNCTIONS */

	void setJointSpeed(robotJointId_t id, double speed);
    void setJointSpeeds(double speed1, double speed2, double speed3);
    void setJointSpeedRatio(robotJointId_t id, double ratio);
    void setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
	void setSpeed(double speed, double radius);

private:
	CLinkbotI **_robots;
	int _numRobots;
    int argInt;
    double argDouble;
    int _numAllocated;
	int _motionInProgress;
	void *_thread;
	char **_ID; //Array that stores the Id of the robots to add to the group
};


/*GroupL*/
class CLinkbotLGroup {

public:
	CLinkbotLGroup();
	~CLinkbotLGroup();
	void addRobot(char* serialID);
	void connect();
	
	/* MOVEMENT FUNCTIONS */
	void holdJoint(robotJointId_t id);
    void holdJoints();
	int isMoving();
	void move(double j1, double j2, double j3);
	void moveForeverNB();
	void moveJoint(robotJointId_t id, double angle);
	void moveJointNB(robotJointId_t id, double angle);
	void moveJointForeverNB(robotJointId_t id);
	void moveJointTo(robotJointId_t id, double angle);
	void moveJointToNB(robotJointId_t id, double angle);
	void moveJointToByTrackPos(robotJointId_t id, double angle);
    void moveJointToByTrackPosNB(robotJointId_t id, double angle);
	void moveJointWait(robotJointId_t id);
    void moveNB(double j1, double j2, double j3);
	void moveTo(double angle1, double angle2, double angle3);
	void moveToNB(double angle1, double angle2, double angle3);
	void moveToByTrackPos(double angle1, double angle2, double angle3);
    void moveToByTrackPosNB(double angle1, double angle2, double angle3);
    void moveToZero();
    void moveToZeroNB();
	void moveWait();
	void relaxJoint(robotJointId_t id);
	void relaxJoints();
	void resetToZero();
    void resetToZeroNB();
    void stop();

	/* SET FUNCTIONS */

	void setJointSpeed(robotJointId_t id, double speed);
    void setJointSpeeds(double speed1, double speed2, double speed3);
    void setJointSpeedRatio(robotJointId_t id, double ratio);
    void setJointSpeedRatios(double ratios1, double ratios2, double ratios3);

private:
	CLinkbotL **_robots;
	int _numRobots;
    int argInt;
    double argDouble;
    int _numAllocated;
	int _motionInProgress;
	void *_thread;
	char **_ID; //Array that stores the Id of the robots to add to the group
};



void *CLinkbotI::g_chlinkbot_dlhandle=NULL;
int CLinkbotI::g_chlinkbot_dlcount=0;
void *CLinkbotL::g_chlinkbot_dlhandle=NULL;
int CLinkbotL::g_chlinkbot_dlcount=0;

#pragma importf "chlinkboti.chf"
#pragma importf "chlinkbotl.chf"
#endif
