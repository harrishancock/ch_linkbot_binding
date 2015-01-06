#include <iostream>
namespace c_impl {
#include "baromesh/linkbot.h"
}
#include "barobo/linkbot.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/thread/condition_variable.hpp"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#define M_PI            3.14159265358979323846

#define RUNTIME_ERROR \
    std::runtime_error(std::string("Exception in ") + std::string(__func__))

#define CALL_C_IMPL(func, ...) \
do { \
    int rc = c_impl::func( m->linkbot, __VA_ARGS__ ); \
    if(rc != 0) { \
        throw std::runtime_error( \
            std::string("Error encountered calling function: <" #func \
                "> Error code: ") + std::to_string(rc)); \
    } \
} while(0)

#define ABS(x) ( (x<0) ? (-(x)) : (x) )

struct LinkbotImpl
{
    friend void _jointEventCB(int, c_impl::barobo::JointState::Type, int, void*);
    void jointEventCB(int jointNo, c_impl::barobo::JointState::Type state);
    boost::mutex jointStateMutex;
    boost::condition_variable jointStateCond;
    c_impl::barobo::JointState::Type jointStates[3];
    c_impl::baromesh::Linkbot *linkbot;
    uint8_t motorMask;
};


void _jointEventCB(int joint, c_impl::barobo::JointState::Type state, int timestamp, void *data)
{
    std::cout << "Received joint event." << std::endl;
    LinkbotImpl *l = static_cast<LinkbotImpl*>(data);
    l->jointEventCB(joint, state);
}

Linkbot::Linkbot(const char* serialId)
{
    std::cout << "In cons..." << std::endl;
    m = new LinkbotImpl();
    std::cout << "Creating impl..." << std::endl;
    m->linkbot = c_impl::linkbotNew(serialId);
    std::cout << "setting joint states..." << std::endl;
    for(int i = 0; i < 3; i++) {
        m->jointStates[i] = c_impl::barobo::JointState::STOP;
    }
    mMaxSpeed = 200;
}

Linkbot::~Linkbot()
{
    delete m;
}

void Linkbot::connect()
{
    c_impl::linkbotConnect(m->linkbot);
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
}

void Linkbot::disconnect() 
{
	c_impl::linkbotDisconnect(m->linkbot);
}

/* GETTERS */

void Linkbot::getDistance(double &distance, double radius)
{
    double angle;
    getJointAngle(robotJointId_t(1), angle);
    distance = (angle*M_PI/180.0)*radius;
}

void Linkbot::getJointAngle(robotJointId_t id, double &angle)
{
    double angles[3];
    getJointAngles(angles[0], angles[1], angles[2]);
    angle = angles[int(id)-1];
}

void Linkbot::getJointAngles(double &angle1, double &angle2, double &angle3)
{
    int timestamp;
    CALL_C_IMPL(linkbotGetJointAngles, 
        &timestamp,
        &angle1,
        &angle2,
        &angle3);
}

void Linkbot::getJointSpeed(robotJointId_t id, double &speed)
{
    double speeds[3];
    getJointSpeeds(speeds[0], speeds[1], speeds[2]);
    speed = speeds[int(id)-1];
}

void Linkbot::getJointSpeedRatio(robotJointId_t id, double &ratio)
{
    double speed;
    getJointSpeed(id, speed);
    ratio = speed / mMaxSpeed;
}

void Linkbot::getJointSpeeds(double &speed1, double &speed2, double &speed3)
{
    CALL_C_IMPL(linkbotGetJointSpeeds, &speed1, &speed2, &speed3);
}

void Linkbot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3)
{
    double speeds[3];
    getJointSpeeds(speeds[0], speeds[1], speeds[2]);
    ratio1 = speeds[0] / mMaxSpeed;
    ratio2 = speeds[1] / mMaxSpeed;
    ratio3 = speeds[2] / mMaxSpeed;
}

/* SETTERS */

void Linkbot::setJointMovementStateNB(robotJointId_t id, robotJointState_t dir)
{
    int index = int(id)-1;
    c_impl::barobo::JointState::Type state;
    bool speedNegative = false;
    switch(dir) {
        case ROBOT_NEUTRAL:
            state = c_impl::barobo::JointState::STOP;
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
    CALL_C_IMPL(linkbotSetJointStates, 1<<index,
            state, speedNegative?-1:1,
            state, speedNegative?-1:1,
            state, speedNegative?-1:1);
}

void Linkbot::setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds)
{
    setJointMovementStateNB(id, dir);
    #ifdef _WIN32
    Sleep(seconds*1000);
    #else
    usleep(seconds*1000000);
    #endif
    stop();
}

void Linkbot::setJointSpeed(robotJointId_t id, double speed)
{
    CALL_C_IMPL(linkbotSetJointSpeeds, 1<<(int(id)-1), speed, speed, speed);
}

void Linkbot::setJointSpeeds(double speed1, double speed2, double speed3)
{
    CALL_C_IMPL(linkbotSetJointSpeeds, 0x07, speed1, speed2, speed3);
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
                _states[i] = c_impl::barobo::JointState::STOP;
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
    setMovementStateNB(dir1, dir2, dir3);
    #ifdef _WIN32
    Sleep(seconds*1000);
    #else
    usleep(seconds*1000000);
    #endif
    stop();
}

void Linkbot::setMovementStateTimeNB( robotJointState_t dir1,
                robotJointState_t dir2,
                robotJointState_t dir3,
                double seconds)
{
}

void Linkbot::setTwoWheelRobotSpeed(double speed, double radius)
{
}


/* MOVEMENT */

void Linkbot::moveForeverNB()
{
	setMovementStateNB(ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_BACKWARD);
}

void Linkbot::moveJoint(robotJointId_t id, double angle)
{
	switch (id) {
		case 1:
			CALL_C_IMPL(linkbotMove, 0x01, angle, angle, angle);
	        moveWait();
			break;
		case 2:
			CALL_C_IMPL(linkbotMove, 0x02, angle, angle, angle);
	        moveWait();
			break;
		case 3:
			CALL_C_IMPL(linkbotMove, 0x04, angle, angle, angle);
	        moveWait();
			break;
		default:
			break;
	}

}

void Linkbot::moveJointNB(robotJointId_t id, double angle)
{
		switch (id) {
		case 1:
			CALL_C_IMPL(linkbotMove, 0x01, angle, angle, angle);
			break;
		case 2:
			CALL_C_IMPL(linkbotMove, 0x02, angle, angle, angle);
			break;
		case 3:
			CALL_C_IMPL(linkbotMove, 0x04, angle, angle, angle);
			break;
		default:
			break;
	}
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
	if(id == ROBOT_JOINT3) {
        setJointMovementStateTime(id, ROBOT_BACKWARD, time);
    } else {
        setJointMovementStateTime(id, ROBOT_FORWARD, time);
    }
}
void Linkbot::moveTime(double time)
{
	setMovementStateTime(ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_BACKWARD, time);
}

void Linkbot::moveJointToByTrackPos(robotJointId_t id, double angle)
{
    moveJointToByTrackPosNB(id, angle);
    moveWait(1<<(int(id)-1));
}

void Linkbot::moveJointToByTrackPosNB(robotJointId_t id, double angle)
{
    CALL_C_IMPL(linkbotDriveTo, 1<<(int(id)-1), angle, angle, angle);
}

void Linkbot::moveToByTrackPos(double angle1, double angle2, double angle3)
{
    moveToByTrackPosNB(angle1, angle2, angle3);
    moveWait();
}

void Linkbot::moveToByTrackPosNB(double angle1, double angle2, double angle3)
{
    CALL_C_IMPL(linkbotDriveTo, 0x07, angle1, angle2, angle3);
}

void Linkbot::driveBackward(double angle)
{
	CALL_C_IMPL(linkbotMove, 0x07, -angle, 0, angle);
	moveWait();
}

void Linkbot::driveBackwardNB(double angle)
{
	CALL_C_IMPL(linkbotMove, 0x07, -angle, 0, angle);
}

void Linkbot::driveDistance(double distance, double radius)
{
	double theta;
    theta = distance/radius;
	
	CALL_C_IMPL(linkbotMove, 0x07, theta, 0, -theta);
	moveWait();

}

void Linkbot::driveDistanceNB(double distance, double radius)
{
	double theta;
    theta = distance/radius;
	CALL_C_IMPL(linkbotMove, 0x07, theta, 0, -theta);
}

void Linkbot::driveForeverNB()
{
	setMovementStateNB(ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_NEGATIVE);
}

void Linkbot::driveForward(double angle)
{
	CALL_C_IMPL(linkbotMove, 0x07, angle, 0, -angle);
	moveWait();
}
void Linkbot::driveForwardNB(double angle)
{
	CALL_C_IMPL(linkbotMove, 0x07, angle, 0, -angle);
}

void Linkbot::driveTime(double seconds)
{
	setMovementStateTime(ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_NEGATIVE, seconds);
}

void Linkbot::driveTimeNB(double seconds)
{
}

void Linkbot::holdJoint(robotJointId_t id)
{
	setJointMovementStateNB(id, ROBOT_HOLD);
}

void Linkbot::holdJoints()
{
	setMovementStateNB(ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD);
}
void Linkbot::move(double j1, double j2, double j3)
{
    moveNB(j1, j2, j3);
    moveWait();
}

void Linkbot::moveNB(double j1, double j2, double j3)
{
    CALL_C_IMPL(linkbotMove, 0x07, j1, j2, j3);
}

void Linkbot::moveWait(int mask)
{
    /* Get the current joint states */
    std::cout << "moveWait()" << std::endl;
    int time;
    boost::unique_lock<boost::mutex> lock(m->jointStateMutex);

    c_impl::linkbotGetJointStates(
        m->linkbot,
        &time,
        &m->jointStates[0],
        &m->jointStates[1],
        &m->jointStates[2]);

    std::cout << "Jointstates: " << m->jointStates[0] << " "
                                 << m->jointStates[1] << " "
                                 << m->jointStates[2] << std::endl;

    for(int i = 0; i < 3; i++) {
        if(!(mask&1<<i)) { continue; }
        if(!((1<<i)&m->motorMask)) { continue; }
        while (m->jointStates[i] == c_impl::barobo::JointState::MOVING) {
            std::cout << "Waiting on " << i << std::endl;
            m->jointStateCond.wait(lock);
        }
    }
}

void Linkbot::moveContinuousNB(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3)
{
}

void Linkbot::moveContinuousTime(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3, 
        double seconds)
{
}

void Linkbot::moveDistance(double distance, double radius)
{
}

void Linkbot::moveDistanceNB(double distance, double radius)
{
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
}

void Linkbot::stopAllJoints()
{
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
   CALL_C_IMPL(linkbotMove, 0x07, theta, 0, theta);
}
/* MISC */

void LinkbotImpl::jointEventCB(int jointNo, c_impl::barobo::JointState::Type state)
{
    boost::lock_guard<boost::mutex> lock(jointStateMutex);
    std::cout << "Setting joint " << jointNo << " To " << state << std::endl;
    jointStates[jointNo] = state;
    jointStateCond.notify_all();
}

void Linkbot::delaySeconds(int seconds)
{
	#ifdef _WIN32
    Sleep(seconds*1000);
    #else
    sleep(seconds);
    #endif
}

