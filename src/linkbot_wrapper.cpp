#include <iostream>
#include <math.h>
namespace c_impl {
#include "baromesh/linkbot.h"
}
#include "barobo/linkbot.hpp"
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#elif defined __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#include <unistd.h>
#else
#include <unistd.h>
#endif

#include "../include/rgbhashtable.h"

#define M_PI            3.14159265358979323846

#define CALL_C_IMPL(func, ...) do { \
    if (isConnected()) { \
        if (auto rc = c_impl::func(m->linkbot, __VA_ARGS__)) { \
            std::cerr << #func << " returned " << rc << std::endl; \
        } \
    } \
} while (0);

#define ABS(x) ( (x<0) ? (-(x)) : (x) )

#define DEPRECATED(from, to) \
  fprintf(stdout, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

/* sgn(num) returns -1, 0, +1 */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

struct LinkbotImpl
{
    LinkbotImpl() 
    : jointsRecordingActive(false), 
      userShiftDataThreshold(2.0),
      exitState(ROBOT_HOLD),
      jointsRecordingMask(0),
      userShiftData(false),
      userInitTime(0),
      userTimeInterval(0.1),
      userRadius(3.5),
      distanceOffset(0)
    {
        for (auto& x : jointSpeed) {
            x = 45;
        }
    }
    ~LinkbotImpl() {
    }
    friend void _jointEventCB(int, c_impl::barobo::JointState::Type, int, void*);
    void jointEventCB(int jointNo, c_impl::barobo::JointState::Type state);
    std::mutex jointStateMutex;
    std::condition_variable jointStateCond;
    c_impl::barobo::JointState::Type jointStates[3];
    c_impl::baromesh::Linkbot *linkbot;
    uint8_t motorMask;
	robotJointState_t exitState;

    void setJointsMovingFlag(int mask) {
        for(int i = 0; i < 3; i++) {
            if(motorMask&mask&(1<<i)) {
                jointStates[i] = c_impl::barobo::JointState::MOVING;
            }
        }
    }

    void refreshJointStates() {
        int time;
        c_impl::linkbotGetJointStates(
            linkbot,
            &time,
            &jointStates[0],
            &jointStates[1],
            &jointStates[2]);
    }

    bool isMoving(int mask) {
        bool moving = false;
        for(int i = 0; i < 3; i++) {
            if(!((1<<i)&mask&motorMask)) continue;
            if(jointStates[i] == c_impl::barobo::JointState::MOVING) {
                moving = true;
                break;
            }
        }
        return moving;
    }

    /* For recording angles */
    /* tuple format is <timestamp, jointNo, angle> */
    std::vector<std::tuple<int, int, double>> recordedJointAngles;
    std::vector<std::tuple<int, double, double, double>> threadJointAngles;
    bool jointsRecordingActive;
    uint8_t jointsRecordingMask;
    void encoderEventCB(int jointNo, double angle, int timestamp);
    std::mutex recordAnglesMutex;
    std::condition_variable recordAnglesCond;
    std::thread recordAnglesThread;
    bool userShiftData;
	bool setShiftData = 1;
    double userShiftDataThreshold;
    double userInitTime;
    double userTimeInterval;
    double userRadius;
    double userInitAngles[3];
    double** userRecordedTimes;
    double** userRecordedAngles[3];
	double distanceOffset;
    double jointSpeed[3];
};

void _encoderEventCB(int joint, double angle, int timestamp, void* data)
{
    LinkbotImpl *l = static_cast<LinkbotImpl*>(data);
    l->encoderEventCB(joint, angle, timestamp);
}

void _jointEventCB(int joint, c_impl::barobo::JointState::Type state, int timestamp, void *data)
{
    LinkbotImpl *l = static_cast<LinkbotImpl*>(data);
    l->jointEventCB(joint, state);
}

Linkbot::Linkbot()
    : m(new LinkbotImpl)
{
	if (m->linkbot = c_impl::linkbotFromTcpEndpoint("127.0.0.1", "42010")){
	}
	else{
		std::cerr << "Could not connect to robot.\n";
		return;
	}

	for (int i = 0; i < 3; i++) {
		m->jointStates[i] = c_impl::barobo::JointState::HOLD;
	}
	mMaxSpeed = 200;

	/* Enable joint callbacks */
	c_impl::linkbotSetJointEventCallback(m->linkbot, _jointEventCB, m);
	/* Get the form factor */
	c_impl::barobo::FormFactor::Type formFactor;
	c_impl::linkbotGetFormFactor(m->linkbot, &formFactor);
	switch (formFactor) {
	case c_impl::barobo::FormFactor::I:
		m->motorMask = 0x05;
		break;
	case c_impl::barobo::FormFactor::L:
		m->motorMask = 0x03;
		break;
	case c_impl::barobo::FormFactor::T:
		m->motorMask = 0x07;
		break;
	}
    /* Set the joint speeds to a default value */
    setJointSpeeds(45, 45, 45);
}

Linkbot::Linkbot(const char* serialId)
    : m(new LinkbotImpl)
{
	if (m->linkbot = c_impl::linkbotFromSerialId(serialId)){
	}
	else{
        std::cerr << "Could not connect to " << serialId << std::endl;
		return;
	}
	for (int i = 0; i < 3; i++) {
		m->jointStates[i] = c_impl::barobo::JointState::HOLD;
	}
	mMaxSpeed = 200;

	/* Enable joint callbacks */
	c_impl::linkbotSetJointEventCallback(m->linkbot, _jointEventCB, m);
	/* Get the form factor */
	c_impl::barobo::FormFactor::Type formFactor;
	c_impl::linkbotGetFormFactor(m->linkbot, &formFactor);
	switch (formFactor) {
	case c_impl::barobo::FormFactor::I:
		m->motorMask = 0x05;
		break;
	case c_impl::barobo::FormFactor::L:
		m->motorMask = 0x03;
		break;
	case c_impl::barobo::FormFactor::T:
		m->motorMask = 0x07;
		break;
	}
    getJointSpeeds( 
        m->jointSpeed[0],
        m->jointSpeed[1],
        m->jointSpeed[2]);
}

int Linkbot::connect()
{
    /*m = new LinkbotImpl();
	
    if(m->linkbot = c_impl::linkbotFromTcpEndpoint("127.0.0.1", "42010")){
		m->connected = true;
	}
	else{
		fprintf(stderr, "Could not connect to robot. Exiting..\n");
		m->connected = false;
		exit (-1);
	}
	
    for(int i = 0; i < 3; i++) {
        m->jointStates[i] = c_impl::barobo::JointState::HOLD;
    }
    mMaxSpeed = 200;*/

    /* Enable joint callbacks */
    //c_impl::linkbotSetJointEventCallback(m->linkbot, _jointEventCB, m);
    /* Get the form factor */
    /*c_impl::barobo::FormFactor::Type formFactor;
    c_impl::linkbotGetFormFactor(m->linkbot, &formFactor);
    switch (formFactor) {
        case c_impl::barobo::FormFactor::I:
            m->motorMask = 0x05;
            break;
        case c_impl::barobo::FormFactor::L:
            m->motorMask = 0x03;
            break;
        case c_impl::barobo::FormFactor::T:
            m->motorMask = 0x07;
            break;
    }*/
    return 0;
}

int Linkbot::connectWithSerialID(const char* serialId)
{
    /*m = new LinkbotImpl();
	    
	if(m->linkbot = c_impl::linkbotFromSerialId(serialId)){
	}
	else{
		fprintf(stderr, "Could not connect to robot. Exiting..\n");
		return -1;
	}
    for(int i = 0; i < 3; i++) {
        m->jointStates[i] = c_impl::barobo::JointState::HOLD;
    }
    mMaxSpeed = 200;*/

    /* Enable joint callbacks */
    //c_impl::linkbotSetJointEventCallback(m->linkbot, _jointEventCB, m);
    /* Get the form factor */
    /*c_impl::barobo::FormFactor::Type formFactor;
    c_impl::linkbotGetFormFactor(m->linkbot, &formFactor);
    switch (formFactor) {
        case c_impl::barobo::FormFactor::I:
            m->motorMask = 0x05;
            break;
        case c_impl::barobo::FormFactor::L:
            m->motorMask = 0x03;
            break;
        case c_impl::barobo::FormFactor::T:
            m->motorMask = 0x07;
            break;
    }*/
    return 0;
}


int Linkbot::disconnect()
{
    if (m->linkbot) {
        c_impl::linkbotDelete(m->linkbot);
        m->linkbot = nullptr;
    }
    return 0;
}

Linkbot::~Linkbot()
{
	stop(); //stop motors
    setMovementStateNB(m->exitState, m->exitState, m->exitState);
    disconnect();
    delete m; m = nullptr;
}

/* GETTERS */

void Linkbot::getAccelerometerData(double &x, double &y, double &z)
{
	int timestamp;
	CALL_C_IMPL(linkbotGetAccelerometer, &timestamp, &x, &y, &z);
}

void Linkbot::getBatteryVoltage(double &voltage)
{
	CALL_C_IMPL(linkbotGetBatteryVoltage, &voltage);
}

void Linkbot::getDistance(double &distance, double radius)
{
    double angle;
    getJointAngle(robotJointId_t(1), angle);
    distance = (angle*M_PI/180.0)*radius;
}

void Linkbot::getFormFactor(int &type)
{
    /* Get the form factor */
    c_impl::barobo::FormFactor::Type formFactor;
    c_impl::linkbotGetFormFactor(m->linkbot, &formFactor);
    switch (formFactor) {
        case c_impl::barobo::FormFactor::I:
			type = 0;
            break;
        case c_impl::barobo::FormFactor::L:
			type = 1;
            break;
        case c_impl::barobo::FormFactor::T:
			type = 2;
            break;
    }
}

void Linkbot::getJointAngleInstant(robotJointId_t id, double &angle)
{
    double angles[3];
    getJointAngles(angles[0], angles[1], angles[2]);
    angle = angles[int(id)-1];
}

void Linkbot::getJointAnglesInstant(double &angle1, double &angle2, double &angle3)
{
    int timestamp;
    CALL_C_IMPL(linkbotGetJointAngles, 
        &timestamp,
        &angle1,
        &angle2,
        &angle3);
}
void Linkbot::getJointAngle(robotJointId_t id, double &angle)
{
    double angles[3];
	int i;
	static int numReadings = 10;

	angle = 0;
	for ( i=0; i<numReadings; i++)
	{
		getJointAnglesInstant(angles[0], angles[1], angles[2]);
        angle += angles[int(id)-1];
	}
	angle = angle/numReadings;
}

void Linkbot::getJointAngles(double &angle1, double &angle2, double &angle3)
{
    double angles[3];
	int i;
	static int numReadings = 10;

	angle1 = 0;
	angle2 = 0;
	angle3 = 0;
	for ( i=0; i<numReadings; i++)
	{
		getJointAnglesInstant(angles[0], angles[1], angles[2]);
        angle1 += angles[0];
		angle2 += angles[1];
		angle3 += angles[2];
	}
	angle1 = angle1/numReadings;
	angle2 = angle2/numReadings;
	angle3 = angle3/numReadings;
}

void Linkbot::getJointSafetyAngle(double &angle)
{
	CALL_C_IMPL(linkbotGetJointSafetyAngles, &angle, &angle, &angle);
}

void Linkbot::getJointSafetyAngleTimeout(double &timeout)
{
	int t;
	CALL_C_IMPL(linkbotGetJointSafetyThresholds, &t, &t, &t);
	/*Convert from motor timer cicles to seconds
	1 cycle is 10 ms*/
	t = (t * 10) / 1000.0;
	timeout = double(t);
}

void Linkbot::getJointSpeed(robotJointId_t id, double &speed)
{
    getJointSpeeds(
        m->jointSpeed[0], 
        m->jointSpeed[1], 
        m->jointSpeed[2]);
    speed = m->jointSpeed[int(id)-1];
}

void Linkbot::getJointSpeedRatio(robotJointId_t id, double &ratio)
{
    double speed = 0;
    getJointSpeed(id, speed);
    ratio = speed / mMaxSpeed;
}

void Linkbot::getJointSpeeds(double &speed1, double &speed2, double &speed3)
{
    CALL_C_IMPL(linkbotGetJointSpeeds, &speed1, &speed2, &speed3);
    m->jointSpeed[0] = speed1;
    m->jointSpeed[1] = speed2;
    m->jointSpeed[2] = speed3;
}

void Linkbot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3)
{
    double speeds[3];
    getJointSpeeds(speeds[0], speeds[1], speeds[2]);
    ratio1 = speeds[0] / mMaxSpeed;
    ratio2 = speeds[1] / mMaxSpeed;
    ratio3 = speeds[2] / mMaxSpeed;
}

void Linkbot::getLEDColorRGB(int &r, int &g, int &b)
{
	CALL_C_IMPL(linkbotGetLedColor, &r, &g, &b);
}

void Linkbot::getLEDColor(char color[])
{
	int getRGB[3];
	int retval;
	rgbHashTable * rgbTable = NULL;

	CALL_C_IMPL(linkbotGetLedColor, &getRGB[0], &getRGB[1], &getRGB[2]);

	rgbTable = HT_Create();
    retval = HT_GetKey(rgbTable, getRGB, color);
    HT_Destroy(rgbTable);
}

/* SETTERS */

void Linkbot::setJointMovementStateNB(robotJointId_t id, robotJointState_t dir)
{
    int index = int(id)-1;
    c_impl::barobo::JointState::Type state;
    bool speedNegative = false;
    switch(dir) {
        case ROBOT_NEUTRAL:
            state = c_impl::barobo::JointState::COAST;
            break;
        case ROBOT_FORWARD:
            state = c_impl::barobo::JointState::MOVING;
            if(index == 2) { speedNegative = true; } 
            break;
        case ROBOT_BACKWARD:
            state = c_impl::barobo::JointState::MOVING;
            speedNegative = true;
            if(index == 2) { speedNegative = false; } 
            break;
        case ROBOT_HOLD:
            state = c_impl::barobo::JointState::HOLD;
            break;
        case ROBOT_POSITIVE:
            state = c_impl::barobo::JointState::MOVING;
            // intentional no-break
        case ROBOT_NEGATIVE:
            speedNegative = true;
            break;
        default:
            break;
    }
    m->setJointsMovingFlag(1<<(int(id)-1));
    CALL_C_IMPL(linkbotSetJointStates, 1<<index,
            state, (speedNegative?-1:1) ,
            state, (speedNegative?-1:1) ,
            state, (speedNegative?-1:1) );
}

void Linkbot::setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds)
{
    setJointMovementStateTimeNB(id, dir, seconds);
    moveWait(1<<(int(id)-1));
}

void Linkbot::setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds)
{
    c_impl::barobo::JointState::Type jointState;
    double coef = 1;
    if(id == 3) {
        switch(dir) {
            case ROBOT_NEGATIVE:
            case ROBOT_FORWARD:
                coef *= -1;
            case ROBOT_POSITIVE:
            case ROBOT_BACKWARD:
                jointState = c_impl::barobo::JointState::MOVING;
                break;
            case ROBOT_HOLD:
                jointState = c_impl::barobo::JointState::HOLD;
                break;
            case ROBOT_NEUTRAL:
            default:
                jointState = c_impl::barobo::JointState::COAST;
                break;
        }
    } else {
        switch(dir) {
            case ROBOT_BACKWARD:
            case ROBOT_NEGATIVE:
                coef *= -1;
            case ROBOT_FORWARD:
            case ROBOT_POSITIVE:
                jointState = c_impl::barobo::JointState::MOVING;
                break;
            case ROBOT_HOLD:
                jointState = c_impl::barobo::JointState::HOLD;
                break;
            case ROBOT_NEUTRAL:
            default:
                jointState = c_impl::barobo::JointState::COAST;
                break;
        }
    }
    m->setJointsMovingFlag(1<<(int(id)-1));
    c_impl::linkbotSetJointStatesTimed(
        m->linkbot, 1<<(int(id)-1),
        jointState, coef, seconds, c_impl::barobo::JointState::HOLD,
        jointState, coef, seconds, c_impl::barobo::JointState::HOLD,
        jointState, coef, seconds, c_impl::barobo::JointState::HOLD);
}

void Linkbot::setJointSpeed(robotJointId_t id, double speed)
{
	double speeds[3] = { 0, 0, 0 };
	if (speed > 200) {
		std::cout << "Warning: cannot set joint speed to " << speed << " degrees/second." << std::endl;
		std::cout << "It is beyond the limit of 200 degrees/second. Joint speed will be set to 200 degrees/second." << std::endl;
		speed = 200.0;
	}
	else if (speed < -200){
		std::cout << "Warning: cannot set joint speed to " << speed << " degrees/second." << std::endl;
		std::cout << "It is beyond the limit of -200 degrees/second. Joint speed will be set to -200 degrees/second." << std::endl;
		speed = -200.0;
	}
	speeds[(id - 1)] = speed;
	CALL_C_IMPL(linkbotSetJointSpeeds, 1<<(int(id)-1), speeds[0], speeds[1], speeds[2]);
    m->jointSpeed[int(id)-1] = speed;
}

void Linkbot::setJointSpeeds(double speed1, double speed2, double speed3)
{
    int type;
	int mask;
	getFormFactor(type);

	switch (type) {
		case 0:
			mask = 0x05;
			break;
		case 1:
			mask = 0x03;
			break;
		case 2:
			mask = 0x07;
			break;
		default:
			mask = 0x07;
			break;
	}
	CALL_C_IMPL(linkbotSetJointSpeeds, mask, speed1, speed2, speed3);
    m->jointSpeed[0] = speed1;
    m->jointSpeed[1] = speed2;
    m->jointSpeed[2] = speed3;
}

void Linkbot::setJointSpeedRatio(robotJointId_t id, double ratio)
{
    setJointSpeed(id, ratio*mMaxSpeed);
}

void Linkbot::setJointSpeedRatios(double ratios1, double ratios2, double ratios3)
{
    setJointSpeeds(
        ratios1*mMaxSpeed,
        ratios2*mMaxSpeed,
        ratios3*mMaxSpeed);
}

void Linkbot::setJointPower(robotJointId_t id, double power)
{
    power *= 255;
    CALL_C_IMPL(linkbotMotorPower, 1<<(int(id)-1), power, power, power);
}

void Linkbot::setMotorPowers(double p1, double p2, double p3)
{
    p1 *= 255;
    p2 *= 255;
    p3 *= 255;
    CALL_C_IMPL(linkbotMotorPower, 0x07, p1, p2, p3);
}

void Linkbot::setMovementStateNB( robotJointState_t dir1,
                robotJointState_t dir2,
                robotJointState_t dir3)
{
    if (!isConnected()) {
        return;
    }
    robotJointState_t states[3];
    states[0] = dir1;
    states[1] = dir2;
    states[2] = dir3;
    
    c_impl::barobo::JointState::Type _states[3];
    int motorNegativeMask = 0;
    int motorMovementMask = 0;
    for(int i = 0; i < 3; i++) {
        switch(states[i]) {
            case ROBOT_NEUTRAL:
                _states[i] = c_impl::barobo::JointState::COAST;
                break;
            case ROBOT_FORWARD:
                _states[i] = c_impl::barobo::JointState::MOVING;
                if(i == 2) motorNegativeMask |= 1<<i;
                motorMovementMask |= 1<<i;
                break;
            case ROBOT_BACKWARD:
                _states[i] = c_impl::barobo::JointState::MOVING;
                if(i != 2) motorNegativeMask |= 1<<i;
                motorMovementMask |= 1<<i;
                break;
            case ROBOT_HOLD:
                _states[i] = c_impl::barobo::JointState::HOLD;
                break;
            case ROBOT_POSITIVE:
                _states[i] = c_impl::barobo::JointState::MOVING;
                motorMovementMask |= 1<<i;
                break;
            case ROBOT_NEGATIVE:
                _states[i] = c_impl::barobo::JointState::MOVING;
                motorNegativeMask |= 1<<i;
                motorMovementMask |= 1<<i;
                break;
            default:
                break;
        }
    }
    /* Get the joints doing their thing */
    m->setJointsMovingFlag(0x07);
    CALL_C_IMPL(linkbotSetJointStates, 0x07,
        _states[0], 1<<0&motorNegativeMask?-1:1,
        _states[1], 1<<1&motorNegativeMask?-1:1,
        _states[2], 1<<2&motorNegativeMask?-1:1);
}

void Linkbot::setMovementStateTime( robotJointState_t dir1,
                robotJointState_t dir2,
                robotJointState_t dir3,
                double seconds)
{
    setMovementStateTimeNB(dir1, dir2, dir3, seconds);
    moveWait();
}

void Linkbot::setMovementStateTimeNB( robotJointState_t dir1,
                robotJointState_t dir2,
                robotJointState_t dir3,
                double seconds)
{
    c_impl::barobo::JointState::Type jointStates[3];
    seconds = ABS(seconds);
    double coefs[3] = {1,1,1};
    int i = 0;
    for(auto d : {dir1, dir2}) {
        switch(d) {
            case ROBOT_BACKWARD:
            case ROBOT_NEGATIVE:
                coefs[i] *= -1;
            case ROBOT_FORWARD:
            case ROBOT_POSITIVE:
                jointStates[i] = c_impl::barobo::JointState::MOVING;
                break;
            case ROBOT_HOLD:
                jointStates[i] = c_impl::barobo::JointState::HOLD;
                break;
            case ROBOT_NEUTRAL:
            default:
                jointStates[i] = c_impl::barobo::JointState::COAST;
                break;
        }
        i++;
    }
    i = 2;
    switch(dir3) {
        case ROBOT_NEGATIVE:
        case ROBOT_FORWARD:
            coefs[i] *= -1;
        case ROBOT_POSITIVE:
        case ROBOT_BACKWARD:
            jointStates[i] = c_impl::barobo::JointState::MOVING;
            break;
        case ROBOT_HOLD:
            jointStates[i] = c_impl::barobo::JointState::HOLD;
            break;
        case ROBOT_NEUTRAL:
        default:
            jointStates[i] = c_impl::barobo::JointState::COAST;
            break;
    }

    m->setJointsMovingFlag(0x07);
    c_impl::linkbotSetJointStatesTimed(
        m->linkbot, 0x07,
        jointStates[0], coefs[0], seconds, c_impl::barobo::JointState::HOLD,
        jointStates[1], coefs[1], seconds, c_impl::barobo::JointState::HOLD,
        jointStates[2], coefs[2], seconds, c_impl::barobo::JointState::HOLD);
}

void Linkbot::setBuzzerFrequencyOn(int frequency)
{
	CALL_C_IMPL(linkbotSetBuzzerFrequency, frequency);
}

void Linkbot::setBuzzerFrequencyOff()
{
	CALL_C_IMPL(linkbotSetBuzzerFrequency, 0);
}

void Linkbot::setBuzzerFrequency(int frequency, double time)
{
	setBuzzerFrequencyOn(frequency);
    #ifdef _WIN32
    Sleep(time*1000);
    #else
    usleep(time*1000000);
    #endif
	setBuzzerFrequencyOff();
}

void Linkbot::setSpeed(double speed, double radius)
{
	double omega;
	omega = speed/radius; // in rad/s
	omega = (omega*180.0)/M_PI; // in deg/s
	if (omega > 200) {
		std::cout<<"Warning: cannot set joint speeds to "<<omega<<" degrees/second."<<std::endl;
		std::cout<<"It is beyond the limit of 200 degrees/second. Joints speeds will be set to 200 degrees/second."<<std::endl;
		omega = 200.0;
	}
	else if (omega < -200){
		std::cout << "Warning: cannot set joint speeds to " << omega << " degrees/second." << std::endl;
		std::cout << "It is beyond the limit of -200 degrees/second. Joints speeds will be set to -200 degrees/second." << std::endl;
		omega = -200.0;
	}
    setJointSpeeds(omega, 0, omega);

}

void Linkbot::setLEDColorRGB(int r, int g, int b)
{
	CALL_C_IMPL(linkbotSetLedColor, r, g, b);
}

void Linkbot::setLEDColor(char *color)
{
	int htRetval;
    int getRGB[3];
    rgbHashTable * rgbTable = HT_Create();

	htRetval = HT_Get(rgbTable, color, getRGB);
	HT_Destroy(rgbTable);

	CALL_C_IMPL(linkbotSetLedColor, getRGB[0], getRGB[1], getRGB[2]);
}

void Linkbot::setJointSafetyAngle(double angle)
{
	int mask = 0x07;
	CALL_C_IMPL(linkbotSetJointSafetyAngles, mask, angle, angle, angle);
}

void Linkbot::setJointSafetyAngleTimeout(double timeout)
{
	int mask = 0x07;
	/*Convert seconds to number of motor cycles*/
	timeout = timeout * 100;
	CALL_C_IMPL(linkbotSetJointSafetyAngles, mask, timeout, timeout, timeout);
}

/* MOVEMENT */

void Linkbot::accelJointTimeNB(robotJointId_t id, double acceleration, double time)
{
    int mask = 1<<(id-1);
    CALL_C_IMPL(linkbotSetAlphaI, mask, 
        acceleration, acceleration, acceleration);
    CALL_C_IMPL(linkbotMoveAccel,
        mask, 0x07, 
        acceleration, time, c_impl::barobo::JointState::HOLD,
        acceleration, time, c_impl::barobo::JointState::HOLD,
        acceleration, time, c_impl::barobo::JointState::HOLD);
}

void Linkbot::accelJointToVelocityNB(robotJointId_t id, double acceleration,
    double velocity)
{
    int mask = 1<<(id-1);
    float time = float(velocity/acceleration);
    CALL_C_IMPL(linkbotSetAlphaI, mask, 
        acceleration, acceleration, acceleration);
    CALL_C_IMPL(linkbotMoveAccel,
        mask, 0x07, 
        0, time, c_impl::barobo::JointState::MOVING,
        0, time, c_impl::barobo::JointState::MOVING,
        0, time, c_impl::barobo::JointState::MOVING);
}

void Linkbot::accelJointAngleNB(robotJointId_t id, double acceleration,
    double angle)
{
    auto timeout = sqrt(2.0*angle/acceleration);
    accelJointTimeNB(id, acceleration, timeout);
}

void Linkbot::accelJointToMaxSpeedNB(robotJointId_t id, double acceleration)
{
    accelJointTimeNB(id, acceleration, 0);
}


void Linkbot::driveAccelJointTimeNB(double radius, double acceleration,
    double time)
{
    auto alpha = acceleration/radius;
    int mask = 0x07;
    CALL_C_IMPL(linkbotSetAlphaI, mask, 
        alpha, alpha, -alpha);
    CALL_C_IMPL(linkbotMoveAccel,
        mask, 0x07, 
        acceleration, time, c_impl::barobo::JointState::HOLD,
        acceleration, time, c_impl::barobo::JointState::HOLD,
        acceleration, time, c_impl::barobo::JointState::HOLD);
}

void Linkbot::driveAccelToVelocityNB(double radius, double acceleration,
    double velocity)
{
    auto timeout = velocity/acceleration;
    auto alpha = acceleration/radius;
    int mask = 0x07;
    CALL_C_IMPL(linkbotSetAlphaI, mask, 
        alpha, alpha, -alpha);
    CALL_C_IMPL(linkbotMoveAccel,
        mask, 0x07, 
        acceleration, timeout, c_impl::barobo::JointState::MOVING,
        acceleration, timeout, c_impl::barobo::JointState::MOVING,
        acceleration, timeout, c_impl::barobo::JointState::MOVING);
}

void Linkbot::driveAccelToMaxSpeedNB(double radius, double acceleration)
{
    auto alpha = acceleration/radius;
    int mask = 0x07;
    CALL_C_IMPL(linkbotSetAlphaI, mask, 
        alpha, alpha, -alpha);
    CALL_C_IMPL(linkbotMoveAccel,
        mask, 0x07, 
        acceleration, 0, c_impl::barobo::JointState::MOVING,
        acceleration, 0, c_impl::barobo::JointState::MOVING,
        acceleration, 0, c_impl::barobo::JointState::MOVING);
}

void Linkbot::driveAccelDistanceNB(double radius, double acceleration, 
    double distance)
{
    double angle = distance/radius;
    double alpha = acceleration/radius;
    auto timeout = sqrt(2.0*angle/alpha);
    int mask = 0x07;
    CALL_C_IMPL(linkbotSetAlphaI, mask, 
        alpha, alpha, -alpha);
    CALL_C_IMPL(linkbotMoveAccel,
        mask, 0x07, 
        acceleration, timeout, c_impl::barobo::JointState::HOLD,
        acceleration, timeout, c_impl::barobo::JointState::HOLD,
        acceleration, timeout, c_impl::barobo::JointState::HOLD);
}

void Linkbot::moveForeverNB()
{
	setMovementStateNB(ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_BACKWARD);
}

void Linkbot::moveJoint(robotJointId_t id, double angle)
{
    moveJointNB(id, angle);
    moveWait();
}

void Linkbot::moveJointNB(robotJointId_t id, double angle)
{
	double angles[3] = { 0, 0, 0 };
	int mask;
	if (id == ROBOT_JOINT1) {
		mask = 0x01;
	}
	else if (id == ROBOT_JOINT2) {
		mask = 0x02;
	}
	else if (id == ROBOT_JOINT3) {
		mask = 0x04;
	}
	else {
		mask = 0x07;
	}
	m->setJointsMovingFlag(mask);
		if (m->jointSpeed[(id - 1)] < 0){
		angles[(id - 1)] = -angle;
	}
	else {
		angles[(id - 1)] = angle;
	}
	
	CALL_C_IMPL(linkbotMove, mask, angles[0], angles[1], angles[2]);
	
}

void Linkbot::moveJointTo(robotJointId_t id, double angle)
{
    moveJointToNB(id, angle);
    moveWait();
}

void Linkbot::moveJointToNB(robotJointId_t id, double angle)
{
    m->setJointsMovingFlag(1<<(int(id)-1));
    CALL_C_IMPL(linkbotMoveTo, 1<<(int(id)-1), angle, angle, angle);
}

void Linkbot::moveJointForeverNB(robotJointId_t id)
{
	if(id == ROBOT_JOINT3) {
        setJointMovementStateNB(id, ROBOT_BACKWARD);
    } else {
        setJointMovementStateNB(id, ROBOT_FORWARD);
    }
}

void Linkbot::moveJointTime(robotJointId_t id, double time)
{
	if (time < 0){
		fprintf(stdout, "Error: time cannot have value %.2lf.\nExit...\n", time);
		return;
	}
	moveJointTimeNB(id, time);
    moveWait(1<<(int(id)-1));
}

void Linkbot::moveJointTimeNB(robotJointId_t id, double time)
{
	if (time < 0){
		fprintf(stdout, "Error: time cannot have value %.2lf.\nExit...\n", time);
		return;
	}
	if(id == ROBOT_JOINT3) {
        setJointMovementStateTimeNB(id, ROBOT_BACKWARD, time);
    } else {
        setJointMovementStateTimeNB(id, ROBOT_FORWARD, time);
    }
}

void Linkbot::moveTime(double time)
{
	if (time < 0){
		fprintf(stdout, "Error: time cannot have value %.2lf.\nExit...\n", time);
		return;
	}
	moveTimeNB(time);
    moveWait();
}

void Linkbot::moveTimeNB(double time)
{
	if (time < 0){
		fprintf(stdout, "Error: time cannot have value %.2lf.\nExit...\n", time);
		return;
	}
	setMovementStateTimeNB(ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_POSITIVE, time);
}

void Linkbot::moveJointToByTrackPos(robotJointId_t id, double angle)
{
    moveJointToByTrackPosNB(id, angle);
    moveWait(1<<(int(id)-1));
}

void Linkbot::moveJointToByTrackPosNB(robotJointId_t id, double angle)
{
    m->setJointsMovingFlag(1<<(int(id)-1));
    CALL_C_IMPL(linkbotDriveTo, 1<<(int(id)-1), angle, angle, angle);
}

void Linkbot::moveToByTrackPos(double angle1, double angle2, double angle3)
{
    moveToByTrackPosNB(angle1, angle2, angle3);
    moveWait();
}

void Linkbot::moveToByTrackPosNB(double angle1, double angle2, double angle3)
{
    m->setJointsMovingFlag(0x07);
    CALL_C_IMPL(linkbotDriveTo, 0x07, angle1, angle2, angle3);
}

void Linkbot::driveBackward(double angle)
{
	fprintf(stdout, "Warning: The function \"%s()\" is deprecated. Please use \"%s\"\n",
		"driveBackward", "driveAngle(-angle)");
    driveAngle(-angle);
}

void Linkbot::driveBackwardNB(double angle)
{
	DEPRECATED("driveBackwardNB", "driveAngleNB(-angle)");
    driveAngleNB(-angle);
}

void Linkbot::driveDistance(double distance, double radius)
{
    if (!m->jointSpeed[0]) {
        fprintf(stdout, "Error: driveDistance called with zero joint one speed\n");
        return;
    }

    driveDistanceNB(distance, radius);
    moveWait();
}

void Linkbot::driveDistanceNB(double distance, double radius)
{
    if (!m->jointSpeed[0]) {
        fprintf(stdout, "Error: driveDistanceNB called with zero joint one speed\n");
        return;
    }

	double theta;
    theta = distance/radius; // in radians
	theta = (theta *180.0)/M_PI; // in degrees
    auto time = theta/m->jointSpeed[0];
    auto dir = distance < 0
               ? ROBOT_BACKWARD
               : ROBOT_FORWARD;
    setMovementStateTimeNB(
        dir,
        dir,
        dir,
        time);
}

void Linkbot::driveForeverNB()
{
	setMovementStateNB(ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_NEGATIVE);
}

void Linkbot::driveForward(double angle)
{
	fprintf(stdout, "Warning: The function \"%s()\" is deprecated. Please use \"%s\"\n",
		"driveForward", "driveAngle(angle)");
    driveAngle(angle);
}
void Linkbot::driveForwardNB(double angle)
{
    DEPRECATED("driveForwardNB", "driveAngle(angle)");
    driveAngleNB(angle);

}

void Linkbot::driveAngle(double angle)
{
    driveAngleNB(angle);
	moveWait();
}

void Linkbot::driveAngleNB(double angle)
{
	m->setJointsMovingFlag(0x07);
	double angle1, angle3;
	angle1 = angle3 = angle;
	if (m->jointSpeed[0] < 0) {
		angle1 *= -1;
	}
	if (m->jointSpeed[2] < 0) {
		angle3 *= -1;
	}
	CALL_C_IMPL(linkbotMove, 0x07, angle1, 0, -angle3);
}

void Linkbot::driveTime(double seconds)
{
	if (seconds < 0){
		fprintf(stdout, "Error: time cannot have value %.2lf.\nExit...\n", seconds);
		return;
	}
	//setMovementStateTime(ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_NEGATIVE, seconds);
	driveTimeNB(seconds);
	moveWait();
}

void Linkbot::driveTimeNB(double seconds)
{
	if (seconds < 0){
		fprintf(stdout, "Error: time cannot have value %.2lf.\nExit...\n", seconds);
		return;
	}
	setMovementStateTimeNB(ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_NEGATIVE, seconds);
}

void Linkbot::holdJoint(robotJointId_t id)
{
	setJointMovementStateNB(id, ROBOT_HOLD);
}

void Linkbot::holdJoints()
{
	setMovementStateNB(ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD);
}

void Linkbot::holdJointsAtExit()
{
	m->exitState = ROBOT_HOLD;
}

void Linkbot::move(double j1, double j2, double j3)
{
    moveNB(j1, j2, j3);
    moveWait();
}

void Linkbot::moveNB(double j1, double j2, double j3)
{
    m->setJointsMovingFlag(0x07);
    CALL_C_IMPL(linkbotMove, 0x07, j1, j2, j3);
}

void Linkbot::moveWait(int mask)
{
    if (!isConnected()) {
        return;
    }
    /* Get the current joint states */
    int time;
    std::unique_lock<std::mutex> lock(m->jointStateMutex);
    /* Check to see if we've already stopped moving first */
    m->refreshJointStates();
    if(!m->isMoving(mask)) {
        return;
    }

    while(1) {
        auto rc = m->jointStateCond.wait_for(lock, std::chrono::milliseconds(3000));
        if (rc == std::cv_status::timeout) {
            m->refreshJointStates();
        }
        if(!m->isMoving(mask)) {
            break;
        }
    }
}

void Linkbot::moveJointWait(robotJointId_t id)
{
    moveWait(1<<(int(id)-1));
}

int Linkbot::isMoving(int mask)
{
    if (!isConnected()) {
        return 0;
    }
    static std::chrono::time_point<std::chrono::system_clock> lastChecked;
    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedTime = now-lastChecked;
    std::unique_lock<std::mutex> lock(m->jointStateMutex);
    if(elapsedTime.count() > 2.0) {
        m->refreshJointStates();
        lastChecked = now;
        m->jointStateCond.notify_all();
    }
    int moving = 0;
    for(int i = 0; i < 3; i++) {
        if(!((1<<i)&mask&m->motorMask)) continue;
        if(m->jointStates[i] == c_impl::barobo::JointState::MOVING) {
            moving = 1;
            break;
        }
    }
    return moving;
}

int Linkbot::isConnected()
{
    return !!m->linkbot;
}

void Linkbot::relaxJoint(robotJointId_t id)
{
	setJointMovementStateNB(id, ROBOT_NEUTRAL);
}

void Linkbot::relaxJoints()
{
	setMovementStateNB(ROBOT_NEUTRAL, ROBOT_NEUTRAL, ROBOT_NEUTRAL);
}

void Linkbot::stop()
{
    CALL_C_IMPL(linkbotStop, 0x07);
}

void Linkbot::stopOneJoint(robotJointId_t id)
{
	CALL_C_IMPL(linkbotStop, 1<<(int(id)-1));
}


void Linkbot::turnLeft(double angle, double radius, double tracklength)
{
    turnLeftNB(angle, radius, tracklength);
    moveWait();
}

void Linkbot::turnLeftNB(double angle, double radius, double tracklength)
{
    double theta;
    theta = (angle*tracklength)/(2*radius);
    m->setJointsMovingFlag(0x07);
    CALL_C_IMPL(linkbotMove, 0x07, -theta, 0, -theta);
}

void Linkbot::turnRight(double angle, double radius, double tracklength)
{
	turnRightNB(angle, radius, tracklength);
	moveWait();
}

void Linkbot::turnRightNB(double angle, double radius, double tracklength)
{
    double theta;
    theta = (angle*tracklength)/(2*radius);
    m->setJointsMovingFlag(0x07);
    CALL_C_IMPL(linkbotMove, 0x07, theta, 0, theta);
}

void Linkbot::openGripper(double angle)
{
	move(-angle/2.0, 0, -angle/2.0);
}

void Linkbot::openGripperNB(double angle)
{
    moveNB(-angle/2.0, 0, -angle/2.0);	
}

void Linkbot::enableRecordDataShift()
{
	m->setShiftData = 1;
}

void Linkbot::disableRecordDataShift()
{
	DEPRECATED("disableRecordDataShift", "recordNoDataShift");
	m->setShiftData = 0;
}

void Linkbot::recordNoDataShift()
{
	m->setShiftData = 0;
}

void Linkbot::recordDistanceOffset(double distance)
{
	m->distanceOffset = distance;
}

void Linkbot::recordAnglesBegin(
            robotRecordData_t &time,
            robotRecordData_t &angle1,
            robotRecordData_t &angle2,
            robotRecordData_t &angle3,
            double timeInterval,
            int mask,
			int shiftData)
{
    if (!isConnected()) {
        return;
    }
    std::unique_lock<std::mutex> lock(m->recordAnglesMutex);
    if(m->jointsRecordingActive) {
        return;
    }
    m->userRecordedTimes = &time;
    m->userRecordedAngles[0] = &angle1;
    m->userRecordedAngles[1] = &angle2;
    m->userRecordedAngles[2] = &angle3;
    m->userTimeInterval = timeInterval;
    m->threadJointAngles.clear();

    /* Initialize stuff for recording thread */
	if (shiftData != m->setShiftData)
	{
		m->userShiftData = m->setShiftData;
	}
	else
	{
		m->userShiftData = shiftData;
	}
    m->jointsRecordingActive = true;
    m->jointsRecordingMask = mask;
    std::thread recordThread
    (
        [this] () {
            double jointAngles[3];
            int timestamp;
            std::chrono::time_point<std::chrono::system_clock> lastPolledTime;
            std::unique_lock<std::mutex> lock (m->recordAnglesMutex);
            std::chrono::time_point<std::chrono::system_clock> now = 
                std::chrono::system_clock::now();
            while(m->jointsRecordingActive) {
                lastPolledTime = std::chrono::system_clock::now();
                /* Get the joint angles */
                c_impl::linkbotGetJointAngles(
                    m->linkbot,
                    &timestamp,
                    &jointAngles[0],
                    &jointAngles[1],
                    &jointAngles[2]);
                m->threadJointAngles.push_back(
                        std::make_tuple(timestamp, jointAngles[0], jointAngles[1],
                        jointAngles[2]));
                now = std::chrono::system_clock::now();
                if((now - lastPolledTime) <
                    std::chrono::milliseconds(int(m->userTimeInterval*1000))) 
                {
                    auto seconds =
                        std::chrono::milliseconds(int(m->userTimeInterval*1000))
                        - (now-lastPolledTime);
                    m->recordAnglesCond.wait_for(lock, seconds);
                } else {
                    lock.unlock();
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    lock.lock();
                }
            }
        }
    );
    m->recordAnglesThread.swap(recordThread);
}

void Linkbot::recordAnglesEnd(int &num)
{
    if (!isConnected()) {
        num = 0;
        return;
    }
    std::unique_lock<std::mutex> lock (m->recordAnglesMutex);
    uint8_t mask = m->jointsRecordingMask;
    if(!m->jointsRecordingActive) {
        return;
    }
    m->jointsRecordingActive = false;
    m->recordAnglesCond.notify_all();
    lock.unlock();
    m->recordAnglesThread.join();
    int startingIndex = 0;
    num = m->threadJointAngles.size();
    if(m->userShiftData) {
        auto initelem = m->threadJointAngles[0];
        double initangles[3];
        initangles[0] = std::get<1>(initelem);
        initangles[1] = std::get<2>(initelem);
        initangles[2] = std::get<3>(initelem);
        for(int i = 1; i < num; i++) {
            auto elem = m->threadJointAngles[i];
            double angles[3];
            angles[0] = std::get<1>(elem);
            angles[1] = std::get<2>(elem);
            angles[2] = std::get<3>(elem);
            for(int j = 0; j < 3; j++) {
                if(! ((1<<j)&mask) ) continue;
                if(ABS(angles[j]-initangles[j]) > m->userShiftDataThreshold) {
                    startingIndex = i;
                    i = num; // break outer loop
                    // if the inner loop is not done yet, we could end up
                    // setting startingIndex = num, so we need to break the
                    // inner loop, too
                    break;
                }
            }
        }
    }
    num = m->threadJointAngles.size() - startingIndex;
    *(m->userRecordedTimes) = new double[num];
    for(int i = 0; i < 3; i++) {
        if((1<<i) & mask) {
            *(m->userRecordedAngles[i]) = new double[num];
        }
    }
    double startTime = std::get<0>(m->threadJointAngles[startingIndex])/1000.0;
    for(int i = 0; i < num; i++, startingIndex++) {
        auto elem = m->threadJointAngles[startingIndex];
        (*(m->userRecordedTimes))[i] = std::get<0>(elem)/1000.0 - startTime;
        if(0x01 & mask)
            (*(m->userRecordedAngles[0]))[i] = std::get<1>(elem);
        if(0x02 & mask)
            (*(m->userRecordedAngles[1]))[i] = std::get<2>(elem);
        if(0x04 & mask)
            (*(m->userRecordedAngles[2]))[i] = std::get<3>(elem);
    }
}

void Linkbot::recordDistanceBegin(
            robotJointId_t id,
            robotRecordData_t &time,
            robotRecordData_t &distance,
            double radius,
            double timeInterval,
            int shiftData)
{
    m->userRadius = radius;
    recordAnglesBegin(
        time,
        distance,
        distance,
        distance,
        timeInterval,
		1 << (int(id) - 1),
        shiftData);
}

void Linkbot::recordDistanceEnd(robotJointId_t id, int& num)
{
	
	recordAnglesEnd(num);
    /* Convert values to distance */
    for(int i = 0; i < num; i++) {
        (*(m->userRecordedAngles[0]))[i] = (*(m->userRecordedAngles[0]))[i] *
            (M_PI/180.0)* m->userRadius;
    }
	/*add the offset*/
	for (int i = 0; i < num; i++) {
		(*(m->userRecordedAngles[0]))[i] += m->distanceOffset;
	}
}

void Linkbot::recordAnglesBegin2(
            robotRecordData_t &time,
            robotRecordData_t &angle1,
            robotRecordData_t &angle2,
            robotRecordData_t &angle3,
            int shiftData)
{
    std::lock_guard<std::mutex> lock(m->recordAnglesMutex);
    /* Get the joint angles right now */
    double anglesNow[3];
    int timestamp;
    auto rc = c_impl::linkbotGetJointAngles(
        m->linkbot,
        &timestamp,
        &m->userInitAngles[0],
        &m->userInitAngles[1],
        &m->userInitAngles[2]);
    if(rc) return;
    m->userInitTime = timestamp/1000.0;

    m->userShiftData = shiftData;
    m->userRecordedTimes = &time;
    m->userRecordedAngles[0] = &angle1;
    m->userRecordedAngles[1] = &angle2;
    m->userRecordedAngles[2] = &angle3;

    /* Set up encoder events */
    if(!m->jointsRecordingMask) {
        c_impl::linkbotSetEncoderEventCallback(m->linkbot, _encoderEventCB, 1.0, m);
    }
    m->jointsRecordingMask = 0x07;
}

void Linkbot::recordAnglesEnd2( int &num )
{
    if (!isConnected()) {
        num = 0;
        return;
    }

    std::lock_guard<std::mutex> lock(m->recordAnglesMutex);
    c_impl::linkbotSetEncoderEventCallback(m->linkbot, nullptr, 0, nullptr);
    m->jointsRecordingMask = 0x0;
    num = m->recordedJointAngles.size();
    if(!m->userShiftData) {
        num+=1; // Account for initial entry
    }
    *(m->userRecordedTimes) = new double[num];
    for(int i = 0; i < 3; i++) {
        *(m->userRecordedAngles[i]) = new double[num];
        (*(m->userRecordedAngles[i]))[0] = m->userInitAngles[i];
    }
    (*m->userRecordedTimes)[0] = 0; // m->userInitTime;
    int i;
    if(m->userShiftData) {
        i = 0;
    } else {
        i = 1;
    }
    for( auto &elem : m->recordedJointAngles ) {
        (*m->userRecordedTimes)[i] = (std::get<0>(elem) / 1000.0) - m->userInitTime;
        for(int j = 0; j < 3; j++) {
            (*(m->userRecordedAngles[j]))[i] = (*(m->userRecordedAngles[j]))[i-1];
        }
        (*(m->userRecordedAngles[std::get<1>(elem)]))[i] = std::get<2>(elem);
        i++;
    }
}

void Linkbot::recordDistanceBegin2(
            robotJointId_t id,
            robotRecordData_t &time,
            robotRecordData_t &distance,
            double radius,
            int shiftData)
{
    m->userRadius = radius;
    std::lock_guard<std::mutex> lock(m->recordAnglesMutex);
    /* Get the joint angles right now */
    double anglesNow[3];
    int timestamp;
    auto rc = c_impl::linkbotGetJointAngles(
        m->linkbot,
        &timestamp,
        &m->userInitAngles[0],
        &m->userInitAngles[1],
        &m->userInitAngles[2]);
    if(rc) return;
    m->userInitTime = timestamp/1000.0;

    m->userShiftData = shiftData;
    m->userRecordedTimes = &time;
    m->userRecordedAngles[int(id)-1] = &distance;

    /* Set up encoder events */
    if(!m->jointsRecordingMask) {
        c_impl::linkbotSetEncoderEventCallback(m->linkbot, _encoderEventCB, 1.0, m);
    }
    m->jointsRecordingMask = 1<<(int(id)-1);
}

void Linkbot::recordDistanceEnd2(robotJointId_t id, int &num)
{
}

void Linkbot::closeGripper()
{
	/* This is the old code. It doesn't work because when the two motors 
	start pushing on each oter there is no safety angle that stops the motion and
	the gripper pops out*/

   double gripperAngleOld= 0;
   double gripperAngleNew;
   
   getJointAngle(ROBOT_JOINT1, gripperAngleNew); // get the new position
    /* Close the gripper to grab an object */
    while(fabs(gripperAngleNew - gripperAngleOld) > 1) { //0.1
        gripperAngleOld = gripperAngleNew;    // update the old position
        getJointAngle(ROBOT_JOINT1, gripperAngleNew); // get the new position
		moveNB( 3, 0, 3); // move 8 degrees
        #ifndef _WIN32
        usleep(1000000);
        #else
        Sleep(1000);
        #endif
        getJointAngle(ROBOT_JOINT1, gripperAngleNew); // get the new position
		
    }
	moveNB(3, 0, 3);            // try to move another 8 degrees 
    #ifndef _WIN32
        usleep(1000000);
    #else
        Sleep(1000);
    #endif            // closing for 1 second
	setMovementStateNB(ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD); // hold the object
	stop();
}

void Linkbot::closeGripperNB()
{
	/* The part with threads is for the old code*/

	std::thread gripperThread(&Linkbot::closeGripper, this);
	gripperThread.join();

    /* New code*/
	/*double angle1, angle3;
	double delta;
	double angle;
	getJointAngle(ROBOT_JOINT1, angle1);
	getJointAngle(ROBOT_JOINT3, angle3);
	delta=360-(angle3+angle1);
	angle=(delta-310)/2.0; 
	if (angle <= 2){
		angle = 0;
	}
	std::cout<<"angle1 "<<angle1<<" angle3 "<<angle3<<std::endl;
	std::cout<<"delta "<<delta<<" angle "<<angle<<std::endl;
	moveNB(angle, 0, angle);*/ 
	//holdJoints();
}

void Linkbot::moveToNB(double angle1, double angle2, double angle3)
{
    m->setJointsMovingFlag(0x07);
    CALL_C_IMPL(linkbotMoveTo, 0x07, angle1, angle2, angle3);
}

void Linkbot::moveTo(double angle1, double angle2, double angle3)
{
	moveToNB(angle1, angle2, angle3);
	moveWait();
}

void Linkbot::moveToZero()
{
    moveToZeroNB();
	moveWait();
}

void Linkbot::moveToZeroNB()
{
    m->setJointsMovingFlag(0x07);
	CALL_C_IMPL(linkbotMoveTo, 0x07, 0, 0, 0);
}

void Linkbot::resetToZeroNB()
{
	c_impl::linkbotResetEncoderRevs(m->linkbot);
	moveToZeroNB();
}

void Linkbot::resetToZero()
{
    c_impl::linkbotResetEncoderRevs(m->linkbot);
	moveToZero();
}
/* MISC */

void Linkbot::blinkLED(double delay, int numBlinks)
{
    std::thread blinkThread {
        [this, numBlinks, delay] {
            int r,g,b;
            getLEDColorRGB(r,g,b);
            for(int i = 0; i < numBlinks; i++) {
                setLEDColorRGB(0,0,0);
                std::this_thread::sleep_for(std::chrono::milliseconds(int(delay*1000)));
                setLEDColorRGB(r,g,b);
                std::this_thread::sleep_for(std::chrono::milliseconds(int(delay*1000)));
            }
        }
    };
    blinkThread.detach();
}

void LinkbotImpl::jointEventCB(int jointNo, c_impl::barobo::JointState::Type state)
{
    std::unique_lock<std::mutex> lock(jointStateMutex, std::try_to_lock);
    if(!lock.owns_lock()) {
        return;
    }
    jointStates[jointNo] = state;
    jointStateCond.notify_all();
}

void LinkbotImpl::encoderEventCB(int jointNo, double angle, int timestamp)
{
    std::unique_lock<std::mutex> lock(recordAnglesMutex, std::try_to_lock);
    if(!lock.owns_lock()) {
        /* If we do not own this lock, we should return immediately. We cannot
         * wait to lock here because the same thread may already be in an RPC
         * method, thereby causing a deadlock */
        return;
    }
    if(jointsRecordingMask & (1<<jointNo)) {
        recordedJointAngles.push_back(
            std::tuple<int, int, double>(timestamp, jointNo, angle)
        );
    }
}

#if 0
void LinkbotImpl::startJointStateThread()
{
    /* Start the joint state thread */
    auto heartbeat = std::thread(
    [this] 
    {
        std::unique_lock<std::mutex> heartbeat_lock(jointStateThreadActiveLock);
        bool waitrc = false;
        while(!waitrc) {
            waitrc = jointStateThreadActiveCond.wait_for(
                heartbeat_lock,
                std::chrono::milliseconds(3000),
                [this] 
                { 
                    if(!jointStateThreadActive) {
                        return true;
                    }
                    int timestamp;
                    c_impl::barobo::JointState::Type jointStates[3];
                    c_impl::linkbotGetJointStates(
                        linkbot, &timestamp, 
                        &jointStates[0],
                        &jointStates[1],
                        &jointStates[2]);
                    return !mHeartbeatEnable; 
                }
            );
        }
    });
    std::swap(jointStateThread, heartbeat);
}

#endif

void Linkbot::delaySeconds(int seconds)
{
	#ifdef _WIN32
    Sleep(seconds*1000);
    #else
    sleep(seconds);
    #endif
}

void Linkbot::systemTime(double &time)
{
    #ifdef _WIN32
    time = (GetTickCount()/1000.0);
    #elif defined __MACH__
    clock_serv_t cclock;
    mach_timespec_t mts;
    mach_timespec_t cur_time;
	host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_nsec = mts.tv_nsec;
    time = mts.tv_sec;
    time += (mts.tv_nsec / 1000000000.0);
    #else
    struct timespec cur_time;
    clock_gettime(CLOCK_REALTIME, &cur_time);
    time = cur_time.tv_sec;
    time += (cur_time.tv_nsec/1000000000.0);
    #endif
}

