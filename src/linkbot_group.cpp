#include <iostream>
#include <vector>
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
#include <stdlib.h>
#include <string.h>

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

/*Structure that holds all the private members of the class*/
struct LinkbotGroupImpl 
{
    std::vector<Linkbot*> robots; /*array of istances of class linkbot*/
    std::vector<std::string> ids; /*array of the id of the robots*/
    int argInt;
    double argDouble;
	int motionInProgress;
	void *thread;
	int _numRobots = 0;
};

LinkbotGroup::LinkbotGroup()
{
  m = new LinkbotGroupImpl(); /*allocate the structure*/	
  m->motionInProgress = 0;
  //_thread = (THREAD_T*)malloc(sizeof(THREAD_T));
}

LinkbotGroup::~LinkbotGroup()
{
}

void LinkbotGroup::addRobot(Linkbot& robot)
{
    m->robots.push_back(&robot);
	m->_numRobots ++;
}

void LinkbotGroup::addRobots(Linkbot robot[], int numRobots) 
{
	int i;
	for (i = 0; i < numRobots; i++) 
	{
		addRobot(robot[i]);
	}
}

void LinkbotGroup::connect()
{
	for (Linkbot* robot : m->robots) {
		robot->connect();
	}
	
	/*for (std::string id : m->ids) {
		/*declare a new element of class Linkbot and put it at the end of the array
		  push_back = add at the end*/
        //m->robots.push_back(new Linkbot());  
		/*access the last element and call a member function
		  id.c_str() gets the C equivalent of the string*/
        //m->robots.back()->connect(id.c_str());
    //}
}

int LinkbotGroup::checkFormFactor(int type)
{
	int linkbotType;
	int check = 0;

	for (Linkbot* robot : m->robots) {
		robot->getFormFactor(linkbotType);
		if (linkbotType != type){
			check = -1;
			return check;
		}
	}
	return check;
}

/* movement functions */
void LinkbotGroup::driveBackwardNB(double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->driveBackwardNB(angle);
	}
} 

void LinkbotGroup::driveBackward(double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->driveBackwardNB(angle);
	}
	moveWait();
}

void LinkbotGroup::driveDistanceNB(double distance, double radius)
{
	for (Linkbot* robot : m->robots) {
		robot->driveDistanceNB(distance, radius);
	}
}

void LinkbotGroup::driveDistance(double distance, double radius)
{
	for (Linkbot* robot : m->robots) {
		robot->driveDistanceNB(distance, radius);
	}
	moveWait();
}

void LinkbotGroup::driveForeverNB()
{
	for (Linkbot* robot : m->robots) {
		robot->driveForeverNB();
	}
}

void LinkbotGroup::driveForwardNB(double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->driveForwardNB(angle);
	}
}

void LinkbotGroup::driveForward(double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->driveForwardNB(angle);
	}
	moveWait();
}

void LinkbotGroup::driveTimeNB(double time)
{
	for (Linkbot* robot : m->robots) {
		robot->driveTimeNB(time);
	}
}

void LinkbotGroup::driveTime(double time)
{
	for (Linkbot* robot : m->robots) {
		robot->driveTimeNB(time);
	}
	moveWait();
}

void LinkbotGroup::moveWait()
{
	int mask = 0x07;
	/*Wait for the last robot to stop*/
	m->robots.back()->moveWait();
}

void LinkbotGroup::moveJointWait(robotJointId_t id)
{
	/*Wait for the last robot to stop*/
	m->robots.back()->moveJointWait(id);
}

void LinkbotGroup::holdJoint(robotJointId_t id)
{
	for (Linkbot* robot : m->robots) {
		robot->holdJoint(id);
	}
}

void LinkbotGroup::holdJoints()
{
	for (Linkbot* robot : m->robots) {
		robot->holdJoints();
	}
}

void LinkbotGroup::turnLeftNB(double angle, double radius, double tracklength)
{
	for (Linkbot* robot : m->robots) {
		robot->turnLeftNB(angle, radius, tracklength);
	}
}

void LinkbotGroup::turnLeft(double angle, double radius, double tracklength)
{
	for (Linkbot* robot : m->robots) {
		robot->turnLeftNB(angle, radius, tracklength);
	}
	moveWait();
}

void LinkbotGroup::turnRightNB(double angle, double radius, double tracklength)
{
	for (Linkbot* robot : m->robots) {
		robot->turnRightNB(angle, radius, tracklength);
	}
}
void LinkbotGroup::turnRight(double angle, double radius, double tracklength)
{
	for (Linkbot* robot : m->robots) {
		robot->turnRightNB(angle, radius, tracklength);
	}
	moveWait();
}

void LinkbotGroup::relaxJoint(robotJointId_t id)
{
	for (Linkbot* robot : m->robots) {
		robot->relaxJoint(id);
	}
}

void LinkbotGroup::relaxJoints()
{
	for (Linkbot* robot : m->robots) {
		robot->relaxJoints();
	}
}
void LinkbotGroup::stop()
{
	for (Linkbot* robot : m->robots) {
		robot->stop();
	}
}

void LinkbotGroup::moveNB(double j1, double j2, double j3)
{
	for (Linkbot* robot : m->robots) {
		robot->moveNB(j1, j2, j3);
	}
}

void LinkbotGroup::move(double j1, double j2, double j3)
{
	for (Linkbot* robot : m->robots) {
		robot->moveNB(j1, j2, j3);
	}
	moveWait();
}

void LinkbotGroup::moveForeverNB()
{
	for (Linkbot* robot : m->robots) {
		robot->moveForeverNB();
	}
}

void LinkbotGroup::moveToNB(double angle1, double angle2, double angle3)
{
	for (Linkbot* robot : m->robots) {
		robot->moveToNB(angle1, angle2, angle3);
	}
}

void LinkbotGroup::moveTo(double angle1, double angle2, double angle3)
{
	for (Linkbot* robot : m->robots) {
		robot->moveToNB(angle1, angle2, angle3);
	}
	moveWait();
}

void LinkbotGroup::moveToZeroNB()
{
	for (Linkbot* robot : m->robots) {
		robot->moveToZeroNB();
	}
}

void LinkbotGroup::moveToZero()
{
	for (Linkbot* robot : m->robots) {
		robot->moveToZeroNB();
	}
	moveWait();
}

void LinkbotGroup::moveToByTrackPosNB(double angle1, double angle2, double angle3)
{
	for (Linkbot* robot : m->robots) {
		robot->moveToByTrackPosNB(angle1, angle2, angle3);
	}
}

void LinkbotGroup::moveToByTrackPos(double angle1, double angle2, double angle3)
{
	for (Linkbot* robot : m->robots) {
		robot->moveToByTrackPosNB(angle1, angle2, angle3);
	}
	moveWait();
}

void LinkbotGroup::moveJointNB(robotJointId_t id, double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->moveJointNB(id, angle);
	}
}

void LinkbotGroup::moveJoint(robotJointId_t id, double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->moveJointNB(id, angle);
	}
	moveWait();
}

void LinkbotGroup::moveJointForeverNB(robotJointId_t id)
{
	for (Linkbot* robot : m->robots) {
		robot->moveJointForeverNB(id);
	}
}

void LinkbotGroup::moveJointToNB(robotJointId_t id, double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->moveJointToNB(id, angle);
	}
}

void LinkbotGroup::moveJointTo(robotJointId_t id, double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->moveJointToNB(id, angle);
	}
	moveWait();
}

void LinkbotGroup::moveJointToByTrackPosNB(robotJointId_t id, double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->moveJointToByTrackPosNB(id, angle);
	}
}

void LinkbotGroup::openGripperNB(double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->openGripperNB(angle);
	}
}

void LinkbotGroup::openGripper(double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->openGripperNB(angle);
	}
	moveWait();
}

void LinkbotGroup::moveJointToByTrackPos(robotJointId_t id, double angle)
{
	for (Linkbot* robot : m->robots) {
		robot->moveJointToByTrackPosNB(id, angle);
	}
	moveWait();
}

void LinkbotGroup::resetToZeroNB()
{
	for (Linkbot* robot : m->robots) {
		robot->resetToZeroNB();
	}
}

void LinkbotGroup::resetToZero()
{
	for (Linkbot* robot : m->robots) {
		robot->resetToZeroNB();
	}
	moveWait();
}

int LinkbotGroup::isMoving(int mask)
{
	for (Linkbot* robot : m->robots) {
		if(robot->isMoving(mask)){
			return 1;
		}
	}
	return 0;
}

void LinkbotGroup::closeGripper()
{
	for (Linkbot* robot : m->robots) {
		robot->closeGripperNB();
	}
	moveWait();
	holdJoints();
}

void LinkbotGroup::closeGripperNB()
{
	for (Linkbot* robot : m->robots) {
		robot->closeGripperNB();
	}
}

/* set functions */
void LinkbotGroup::setJointSpeed(robotJointId_t id, double speed)
{
	for (Linkbot* robot : m->robots) {
		robot->setJointSpeed(id, speed);
	}
}

void LinkbotGroup::setJointSpeedRatio(robotJointId_t id, double ratio)
{
	for (Linkbot* robot : m->robots) {
		robot->setJointSpeedRatio(id, ratio);
	}
}

void LinkbotGroup::setJointSpeeds(double speed1, double speed2, double speed3)
{
	for (Linkbot* robot : m->robots) {
		robot->setJointSpeeds(speed1, speed2, speed3);
	}
}

void LinkbotGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3)
{
	for (Linkbot* robot : m->robots) {
		robot->setJointSpeedRatios(ratio1, ratio2, ratio3);
	}
}

void LinkbotGroup::setSpeed(double speed, double radius)
{
	for (Linkbot* robot : m->robots) {
		robot->setSpeed(speed, radius);
	}
}




