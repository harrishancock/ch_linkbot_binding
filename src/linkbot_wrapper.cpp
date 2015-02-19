#include <iostream>
namespace c_impl {
#include "baromesh/linkbot.h"
}
#include "barobo/linkbot.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/thread/condition_variable.hpp"
#include "boost/thread.hpp"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "../include/rgbhashtable.h"

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

Linkbot::Linkbot()
{
}

int Linkbot::connect(const char* serialId)
{
    std::cout << "In cons..." << std::endl;
    m = new LinkbotImpl();
    std::cout << "Creating impl..." << std::endl;
	
    try {
        m->linkbot = c_impl::linkbotNew(serialId);
		//std::cout<<"m->linkbot "<<m->linkbot<<std::endl;
    }
    catch (std::exception& e){
        fprintf(stderr, "Could not connect to robot: %s\n", e.what());
        return -1;
    }
    std::cout << "setting joint states..." << std::endl;
    for(int i = 0; i < 3; i++) {
        m->jointStates[i] = c_impl::barobo::JointState::STOP;
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
    return 0;
}

int Linkbot::disconnect()
{
    if(m && m->linkbot) {
        //c_impl::linkbotDelete(m->linkbot);
        delete m;
    }
    return 0;
}

Linkbot::~Linkbot()
{
	if(m && m->linkbot) { 
	stop(); //stop motors
	disconnect();
	}
}

/* GETTERS */

void Linkbot::getAccelerometerData(double &x, double &y, double &z)
{
	int timestamp;
	CALL_C_IMPL(linkbotGetAccelerometer, &timestamp, &x, &y, &z);
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
	std::cout<<"mask "<<mask<<std::endl;
	CALL_C_IMPL(linkbotSetJointSpeeds, mask, speed1, speed2, speed3);
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
	if (omega >= 240) {
		std::cout<<"Warning: cannot set joint speeds to "<<omega<<" degrees/second."<<std::endl;
		std::cout<<"It is beyond the limit of 240 degrees/second. Joints speeds will be set to 240 degrees/second."<<std::endl;
		omega = 240.0;
	}
	CALL_C_IMPL(linkbotSetJointSpeeds, 0x07, omega, 0, omega);
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

void Linkbot::moveJointTo(robotJointId_t id, double angle)
{
	switch (id) {
		case 1:
			CALL_C_IMPL(linkbotMoveTo, 0x01, angle, angle, angle);
	        moveWait();
			break;
		case 2:
			CALL_C_IMPL(linkbotMoveTo, 0x02, angle, angle, angle);
	        moveWait();
			break;
		case 3:
			CALL_C_IMPL(linkbotMoveTo, 0x04, angle, angle, angle);
	        moveWait();
			break;
		default:
			break;
	}

}

void Linkbot::moveJointToNB(robotJointId_t id, double angle)
{
		switch (id) {
		case 1:
			CALL_C_IMPL(linkbotMoveTo, 0x01, angle, angle, angle);
			break;
		case 2:
			CALL_C_IMPL(linkbotMoveTo, 0x02, angle, angle, angle);
			break;
		case 3:
			CALL_C_IMPL(linkbotMoveTo, 0x04, angle, angle, angle);
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
    theta = distance/radius; // in radians
	theta = (theta *180.0)/M_PI; // in degrees

	CALL_C_IMPL(linkbotMove, 0x07, theta, 0, -theta);
	moveWait();

}

void Linkbot::driveDistanceNB(double distance, double radius)
{
	double theta;
    theta = distance/radius; // in radians
	theta = (theta *180.0)/M_PI; // in degrees
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

void Linkbot::moveJointWait(robotJointId_t id)
{
    /* Get the current joint states */
    std::cout << "moveWait()" << std::endl;
    int time;
	int i = id-1;
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

	while(m->jointStates[i] == c_impl::barobo::JointState::MOVING){
		std::cout << "Waiting on Joint " << i << std::endl;
		m->jointStateCond.wait(lock);
	}
}

int Linkbot::isMoving(int mask)
{
    /* Get the current joint states */
    std::cout << "isMoving()" << std::endl;
    int time;
	int moving = 0;
    boost::unique_lock<boost::mutex> lock(m->jointStateMutex);

    c_impl::linkbotGetJointStates(
        m->linkbot,
        &time,
        &m->jointStates[0],
        &m->jointStates[1],
        &m->jointStates[2]);

    std::cout << "Jointstates in isMoving: " << m->jointStates[0] << " "
                                 << m->jointStates[1] << " "
                                 << m->jointStates[2] << std::endl;

    for(int i = 0; i < 3; i++) {
        if(!(mask&1<<i)) { continue; }
        if(!((1<<i)&m->motorMask)) { continue; }
        if (m->jointStates[i] == c_impl::barobo::JointState::MOVING) {
            std::cout << "Joint " << i <<" is stopped"<< std::endl;
			moving = 1;
            break;
        }
		else {
			continue;
		}
    }
	return moving;
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

void Linkbot::openGripper(double angle)
{
	move(-angle/2.0, 0, -angle/2.0);
}

void Linkbot::openGripperNB(double angle)
{
    moveNB(-angle/2.0, 0, -angle/2.0);	
}

void Linkbot::closeGripper()
{
	/* This is the old code. It doesn't work because when the two motors 
	start pushing on each oter there is no safety angle that stops the motion and
	the gripper pops out*/

   /*double gripperAngleOld= 0;
   double gripperAngleNew;
   
   getJointAngle(ROBOT_JOINT1, gripperAngleNew); // get the new position
   std::cout<<"angle "<<gripperAngleNew<<std::endl;*/
    /* Close the gripper to grab an object */
    /*while(fabs(gripperAngleNew - gripperAngleOld) > 0.1) { //0.1
        gripperAngleOld = gripperAngleNew;    // update the old position
        getJointAngle(ROBOT_JOINT1, gripperAngleNew); // get the new position
		std::cout<<"angle "<<gripperAngleNew<<std::endl;
		moveNB( 8, 0, 8); // move 8 degrees
        #ifndef _WIN32
        usleep(1000000);
        #else
        Sleep(1000);
        #endif
        getJointAngle(ROBOT_JOINT1, gripperAngleNew); // get the new position
		
    }
	//moveNB(8, 0, 8);            // try to move another 8 degrees 
    #ifndef _WIN32
        usleep(1000000);
    #else
        Sleep(1000);
    #endif            // closing for 1 second
	setMovementStateNB(ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD); // hold the object
	stop();
	return;*/
	
	/* New version. The value of angle needs tuning */
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
	move(angle, 0, angle);
	holdJoints();*/

}

void Linkbot::closeGripperNB()
{
	/* The part with threads is for the old code*/
	//boost::thread gripperThread(&Linkbot::closeGripper, this);
	//gripperThread.detach();

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
	int type;
	getFormFactor(type);
	switch(type){
		case 0:
			CALL_C_IMPL(linkbotMoveTo, 0x05, angle1, angle2, angle3);
			break;
		case 1:
			CALL_C_IMPL(linkbotMoveTo, 0x03, angle1, angle2, angle3);
			break;
		case 2:
			CALL_C_IMPL(linkbotMoveTo, 0x07, angle1, angle2, angle3);
			break;
		default:
			CALL_C_IMPL(linkbotMoveTo, 0x07, angle1, angle2, angle3);
			break;
	}
	
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
	
	CALL_C_IMPL(linkbotMoveTo, mask, 0, 0, 0);
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

