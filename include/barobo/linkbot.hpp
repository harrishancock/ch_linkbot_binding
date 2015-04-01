#ifndef LINKBOT_WRAPPER_HPP_
#define LINKBOT_WRAPPER_HPP_

typedef double* robotRecordData_t;

typedef enum robotJoints_e {
  ROBOT_ZERO,
  ROBOT_JOINT1,
  ROBOT_JOINT2,
  ROBOT_JOINT3,
  ROBOT_JOINT4,
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

class Linkbot {
    public:
        Linkbot();
        ~Linkbot();
        int connectWithSerialID(const char* serialID);
		int connect();
		int disconnect();

        /* GETTERS */

        void getAccelerometerData(double &x, double &y, double &z);
		void getBatteryVoltage(double &voltage);
		void getDistance(double &distance, double radius);
        void getFormFactor(int &type);
        void getJointAngle(robotJointId_t id, double &angle);
        void getJointAngles(double &angle1, double &angle2, double &angle3);
		void getJointAngleInstant(robotJointId_t id, double &angle);
        void getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
		void getJointSafetyAngle(double &angle);
		void getJointSafetyAngleTimeout(double &timeout);
        void getJointSpeed(robotJointId_t id, double &speed);
        void getJointSpeedRatio(robotJointId_t id, double &ratio);
        void getJointSpeeds(double &speed1, double &speed2, double &speed3);
        void getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		void getLEDColorRGB(int &r, int &g, int &b);
		void getLEDColor(char color[]);

        /* SETTERS */
        void setBuzzerFrequency(int frequency, double time);
		void setBuzzerFrequencyOn(int frequency);
		void setBuzzerFrequencyOff();
		void setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
        void setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
        void setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds);
		void setJointSafetyAngle(double angle);
		void setJointSafetyAngleTimeout(double timeout);
        void setJointSpeed(robotJointId_t id, double speed);
        void setJointSpeeds(double speed1, double speed2, double speed3);
        void setJointSpeedRatio(robotJointId_t id, double ratio);
        void setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
        void setJointPower(robotJointId_t id, double power);
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
		void driveAngle(double angle);
		void driveAngleNB(double angle);
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
		void holdJointsAtExit();
		int isMoving(int mask=0x07);
		int isConnected();
        void move(double j1, double j2, double j3);
        void moveNB(double j1, double j2, double j3);
        void moveWait(int mask=0x07);
		void moveForeverNB();
		void moveJoint(robotJointId_t id, double angle);
	    void moveJointNB(robotJointId_t id, double angle);
	    void moveJointForeverNB(robotJointId_t id);
		void moveJointTime(robotJointId_t id, double time);
		void moveJointTimeNB(robotJointId_t id, double time);
		void moveJointTo(robotJointId_t id, double angle);
		void moveJointToNB(robotJointId_t id, double angle);
		void moveJointToByTrackPos(robotJointId_t id, double angle);
        void moveJointToByTrackPosNB(robotJointId_t id, double angle);
		void moveJointWait(robotJointId_t id);
		void moveTime(double time);
		void moveTimeNB(double time);
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
        /*
        void enableButtonCallback(void* userdata, void (*buttonCallback)(void* data, int button, int buttonDown));
        void disableButtonCallback();
        */
        void recordAnglesBegin(
            robotRecordData_t &time,
            robotRecordData_t &angle1,
            robotRecordData_t &angle2,
            robotRecordData_t &angle3,
            double timeInterval = 0.1,
            int mask = 0x07,
			int shiftData = 1);
        void recordAnglesEnd(int &num);
        void recordDistanceBegin(
            robotJointId_t id,
            robotRecordData_t &time,
            robotRecordData_t &distance,
            double radius,
            double timeInterval = 0.1,
            int shiftData = 1);
        void recordDistanceEnd(robotJointId_t id, int &num);

        void recordAnglesBegin2(
            robotRecordData_t &time,
            robotRecordData_t &angle1,
            robotRecordData_t &angle2,
            robotRecordData_t &angle3,
            int shiftData = 1);
        void recordAnglesEnd2(int &num);
        void recordDistanceBegin2(
            robotJointId_t id,
            robotRecordData_t &time,
            robotRecordData_t &distance,
            double radius,
            int shiftData = 1);
        void recordDistanceEnd2(robotJointId_t id, int &num);
		void recordDistanceOffset(double distance);
		void recordNoDataShift();
		void enableRecordDataShift();
		void disableRecordDataShift();
		void delaySeconds(int seconds);
		void systemTime(double &time);

        LinkbotImpl *m;

        private:
        double mMaxSpeed;
};

struct LinkbotGroupImpl;

class LinkbotGroup {

public:
	LinkbotGroup();
	~LinkbotGroup();
	void addRobot(Linkbot& robot);
	void addRobots(Linkbot robot[], int numRobots);
	void connect();
	int checkFormFactor(int type);

	/* MOVEMENT FUNCTIONS */
	void closeGripper();
	void closeGripperNB();
	void driveAngle(double angle);
	void driveAngleNB(double angle);
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
	void holdJointsAtExit();
	int isMoving(int mask=0x07);
	int isConnected();
	void move(double j1, double j2, double j3);
	void moveForeverNB();
	void moveJoint(robotJointId_t id, double angle);
	void moveJointNB(robotJointId_t id, double angle);
	void moveJointForeverNB(robotJointId_t id);
	void moveJointTime(robotJointId_t id, double time);
	void moveJointTimeNB(robotJointId_t id, double time);
	void moveJointTo(robotJointId_t id, double angle);
	void moveJointToNB(robotJointId_t id, double angle);
	void moveJointToByTrackPos(robotJointId_t id, double angle);
    void moveJointToByTrackPosNB(robotJointId_t id, double angle);
	void moveJointWait(robotJointId_t id);
    void moveNB(double j1, double j2, double j3);
	void moveTime(double time);
	void moveTimeNB(double time);
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
    LinkbotGroupImpl *m;
};


#endif
