#include"../include/barobo/linkbot.hpp"
#include<ch.h>
#include<stdio.h>

/*class creator*/
EXPORTCH void CLinkbotI_CLinkbotI_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    char *id;
    
    Ch_VaStart(interp, ap, varg);
    id = Ch_VaArg(interp, ap, char *);
    class Linkbot *l= new Linkbot(id);
    Ch_CppChangeThisPointer(interp, l, sizeof(Linkbot));
    Ch_VaEnd(interp, ap);
    return;
}
 
/* class destructor*/   
EXPORTCH void CLinkbotI_dCLinkbotI_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
     
    Ch_VaStart(interp, ap, varg);
    l=Ch_VaArg(interp, ap, class Linkbot *);
    if(Ch_CppIsArrayElement(interp))
        l->~Linkbot();
    else
        delete l;
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot connect*/
EXPORTCH void CLinkbotI_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->connect();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot disconnect*/
EXPORTCH void CLinkbotI_disconnect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->disconnect();
    Ch_VaEnd(interp, ap);
    return;
}

/*MOVEMENT FUNCTIONS*/

/*linkbot move*/
EXPORTCH void CLinkbotI_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double j1;
    double j2;
    double j3;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    j1=Ch_VaArg(interp, ap, double);
    j2=Ch_VaArg(interp, ap, double);
    j3=Ch_VaArg(interp, ap, double);
    l->move(j1, j2, j3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveNB*/
EXPORTCH void CLinkbotI_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double j1;
    double j2;
    double j3;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    j1=Ch_VaArg(interp, ap, double);
    j2=Ch_VaArg(interp, ap, double);
    j3=Ch_VaArg(interp, ap, double);
    l->moveNB(j1, j2, j3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveToByTrackPos*/
EXPORTCH void CLinkbotI_moveToByTrackPos_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double angle1;
    double angle2;
    double angle3;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle1=Ch_VaArg(interp, ap, double);
    angle2=Ch_VaArg(interp, ap, double);
    angle3=Ch_VaArg(interp, ap, double);
    l->moveToByTrackPos(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}


/*linkbot moveToByTrackPosNB*/
EXPORTCH void CLinkbotI_moveToByTrackPosNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double angle1;
    double angle2;
    double angle3;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle1=Ch_VaArg(interp, ap, double);
    angle2=Ch_VaArg(interp, ap, double);
    angle3=Ch_VaArg(interp, ap, double);
    l->moveToByTrackPosNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointToByTrackPos*/
EXPORTCH void CLinkbotI_moveJointToByTrackPos_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    angle=Ch_VaArg(interp, ap, double);
    l->moveJointToByTrackPos(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointToByTrackPosNB*/
EXPORTCH void CLinkbotI_moveJointToByTrackPosNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    angle=Ch_VaArg(interp, ap, double);
    l->moveJointToByTrackPosNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotDriveBackward*/
EXPORTCH void CLinkbotI_driveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle=Ch_VaArg(interp, ap, double);
    l->driveBackward(angle);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotDriveBackwardNB*/
EXPORTCH void CLinkbotI_driveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle=Ch_VaArg(interp, ap, double);
    l->driveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotDriveForward*/
EXPORTCH void CLinkbotI_driveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle=Ch_VaArg(interp, ap, double);
    l->driveForward(angle);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotDriveForwardNB*/
EXPORTCH void CLinkbotI_driveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle=Ch_VaArg(interp, ap, double);
    l->driveForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotDriveDistance*/
EXPORTCH void CLinkbotI_driveDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double distance;
	double radius;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    distance=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
    l->driveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotDriveDistanceNB*/
EXPORTCH void CLinkbotI_driveDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double distance;
	double radius;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    distance=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
    l->driveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotDriveForeverNB*/
EXPORTCH void CLinkbotI_driveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->driveForeverNB();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotDriveTime*/
EXPORTCH void CLinkbotI_driveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double seconds;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	seconds=Ch_VaArg(interp, ap, double);
    l->driveTime(seconds);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveWait*/
EXPORTCH void CLinkbotI_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    int mask;    
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    mask=Ch_VaArg(interp, ap, int);
    l->moveWait(mask);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveContinuousNB*/
EXPORTCH void CLinkbotI_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    dir1=Ch_VaArg(interp, ap, robotJointState_t);
    dir2=Ch_VaArg(interp, ap, robotJointState_t);
    dir3=Ch_VaArg(interp, ap, robotJointState_t);
    l->moveContinuousNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveContinuousTime*/
EXPORTCH void CLinkbotI_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    dir1=Ch_VaArg(interp, ap, robotJointState_t);
    dir2=Ch_VaArg(interp, ap, robotJointState_t);
    dir3=Ch_VaArg(interp, ap, robotJointState_t);
    seconds=Ch_VaArg(interp, ap, double);
    l->moveContinuousTime(dir1, dir2, dir3,seconds);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveDistance*/
EXPORTCH void CLinkbotI_moveDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double distance;
    double radius;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    distance=Ch_VaArg(interp, ap, double);
    radius=Ch_VaArg(interp, ap, double);
    l->moveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveDistanceNB*/
EXPORTCH void CLinkbotI_moveDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double distance;
    double radius;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    distance=Ch_VaArg(interp, ap, double);
    radius=Ch_VaArg(interp, ap, double);
    l->moveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot stop*/
EXPORTCH void CLinkbotI_stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->stop();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot stopOneJoint*/
EXPORTCH void CLinkbotI_stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    l->stopOneJoint(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot stopAllJoins*/
EXPORTCH void CLinkbotI_stopAllJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->stopAllJoints();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot turnLeft*/
EXPORTCH void CLinkbotI_turnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double angle;
    double radius;
	double tracklength;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle=Ch_VaArg(interp, ap, double);
    radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    l->turnLeft(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbot turnLeftNB*/
EXPORTCH void CLinkbotI_turnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double angle;
    double radius;
	double tracklength;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle=Ch_VaArg(interp, ap, double);
    radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    l->turnLeftNB(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot turnRight*/
EXPORTCH void CLinkbotI_turnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double angle;
    double radius;
	double tracklength;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle=Ch_VaArg(interp, ap, double);
    radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    l->turnRight(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot turnRightNB*/
EXPORTCH void CLinkbotI_turnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double angle;
    double radius;
	double tracklength;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle=Ch_VaArg(interp, ap, double);
    radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    l->turnRightNB(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot holdJoint*/
EXPORTCH void CLinkbotI_holdJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
    l->holdJoint(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot holdJoints*/
EXPORTCH void CLinkbotI_holdJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->holdJoints();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot relaxJoint*/
EXPORTCH void CLinkbotI_relaxJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
    l->relaxJoint(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot relaxJoints*/
EXPORTCH void CLinkbotI_relaxJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->relaxJoints();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveForeverNB*/
EXPORTCH void CLinkbotI_moveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->moveForeverNB();
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbot moveJoint*/
EXPORTCH void CLinkbotI_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
	double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	angle=Ch_VaArg(interp, ap, double);
    l->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbot moveJointNB*/
EXPORTCH void CLinkbotI_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
	double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	angle=Ch_VaArg(interp, ap, double);
    l->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointForeverNB*/
EXPORTCH void CLinkbotI_moveJointForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
    l->moveJointForeverNB(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveTime*/
EXPORTCH void CLinkbotI_moveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	double time;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	time=Ch_VaArg(interp, ap, double);
    l->moveTime(time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointForeverNB*/
EXPORTCH void CLinkbotI_moveJointTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
	double time;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	time=Ch_VaArg(interp, ap, double);
    l->moveJointTime(id, time);
    Ch_VaEnd(interp, ap);
    return;
}
/*END MOVEMENT FUNCTIONS*/
/*GET FUNCTIONS*/

/*linkbot getDistance*/
EXPORTCH void CLinkbotI_getDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double *distance;
    double radius;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    distance=Ch_VaArg(interp, ap, double *);
    radius=Ch_VaArg(interp, ap, double);
    l->getDistance(*distance, radius);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointAngle*/
EXPORTCH void CLinkbotI_getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double *angle;
    robotJointId_t id;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    angle=Ch_VaArg(interp, ap, double *);
    l->getJointAngle(id, *angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointAngles*/
EXPORTCH void CLinkbotI_getJointAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double *angle1;
    double *angle2;
    double *angle3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    angle1=Ch_VaArg(interp, ap, double *);
    angle2=Ch_VaArg(interp, ap, double *);
    angle3=Ch_VaArg(interp, ap, double *);
    l->getJointAngles(*angle1, *angle2, *angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointSpeed*/
EXPORTCH void CLinkbotI_getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double *speed;
    robotJointId_t id;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    speed=Ch_VaArg(interp, ap, double *);
    l->getJointSpeed(id, *speed);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointSpeedRatio*/
EXPORTCH void CLinkbotI_getJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double *ratio;
    robotJointId_t id;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    ratio=Ch_VaArg(interp, ap, double *);
    l->getJointSpeedRatio(id, *ratio);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointSpeeds*/
EXPORTCH void CLinkbotI_getJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double *speed1;
    double *speed2;
    double *speed3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    speed1=Ch_VaArg(interp, ap, double *);
    speed2=Ch_VaArg(interp, ap, double *);
    speed3=Ch_VaArg(interp, ap, double *);
    l->getJointSpeeds(*speed1, *speed2, *speed3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointSpeedRatios*/
EXPORTCH void CLinkbotI_getJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double *ratio1;
    double *ratio2;
    double *ratio3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    ratio1=Ch_VaArg(interp, ap, double *);
    ratio2=Ch_VaArg(interp, ap, double *);
    ratio3=Ch_VaArg(interp, ap, double *);
    l->getJointSpeedRatios(*ratio1, *ratio2, *ratio3);
    Ch_VaEnd(interp, ap);
    return;
}

/*END GET FUNCTIONS*/
/*SET FUNCTIONS*/

/*linkbot setJointMovementStateNB*/
EXPORTCH void CLinkbotI_setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    robotJointState_t dir;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    dir=Ch_VaArg(interp, ap, robotJointState_t);
    l->setJointMovementStateNB(id, dir);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointMovementStateTime*/
EXPORTCH void CLinkbotI_setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    dir=Ch_VaArg(interp, ap, robotJointState_t);
    seconds=Ch_VaArg(interp, ap, double);
    l->setJointMovementStateTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointSpeed*/
EXPORTCH void CLinkbotI_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    double speed;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    speed=Ch_VaArg(interp, ap, double);
    l->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointSpeeds*/
EXPORTCH void CLinkbotI_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double speed1;
    double speed2;
    double speed3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    speed1=Ch_VaArg(interp, ap, double);
    speed2=Ch_VaArg(interp, ap, double);
    speed3=Ch_VaArg(interp, ap, double);
    l->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointSpeedRatio*/
EXPORTCH void CLinkbotI_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    double ratio;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    ratio=Ch_VaArg(interp, ap, double);
    l->setJointSpeedRatio(id, ratio);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointSpeedRatios*/
EXPORTCH void CLinkbotI_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double ratio1;
    double ratio2;
    double ratio3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    ratio1=Ch_VaArg(interp, ap, double);
    ratio2=Ch_VaArg(interp, ap, double);
    ratio3=Ch_VaArg(interp, ap, double);
    l->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointPower*/
EXPORTCH void CLinkbotI_setJointPower_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
    int power;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    power=Ch_VaArg(interp, ap, int);
    l->setJointPower(id, power);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setMotorPowers*/
EXPORTCH void CLinkbotI_setMotorPowers_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double p1;
    double p2;
    double p3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    p1=Ch_VaArg(interp, ap, double);
    p2=Ch_VaArg(interp, ap, double);
    p3=Ch_VaArg(interp, ap, double);
    l->setMotorPowers(p1, p2, p3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setMovementStateNB*/
EXPORTCH void CLinkbotI_setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    dir1=Ch_VaArg(interp, ap, robotJointState_t);
    dir2=Ch_VaArg(interp, ap, robotJointState_t);
    dir3=Ch_VaArg(interp, ap, robotJointState_t);
    l->setMovementStateNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setMovementStateTime*/
EXPORTCH void CLinkbotI_setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    dir1=Ch_VaArg(interp, ap, robotJointState_t);
    dir2=Ch_VaArg(interp, ap, robotJointState_t);
    dir3=Ch_VaArg(interp, ap, robotJointState_t);
    seconds=Ch_VaArg(interp, ap, double);
    l->setMovementStateTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setMovementStateTime*/
EXPORTCH void CLinkbotI_setMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    dir1=Ch_VaArg(interp, ap, robotJointState_t);
    dir2=Ch_VaArg(interp, ap, robotJointState_t);
    dir3=Ch_VaArg(interp, ap, robotJointState_t);
    seconds=Ch_VaArg(interp, ap, double);
    l->setMovementStateTimeNB(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setTwoWheelRobotSpeed*/
EXPORTCH void CLinkbotI_setTwoWheelRobotSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double speed;
    double radius;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    speed=Ch_VaArg(interp, ap, double);
    radius=Ch_VaArg(interp, ap, double);
    l->setTwoWheelRobotSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return;
}

/*END SET FUNCTIONS*/
/*MISCELLANEOUS FUNCTIONS*/

/*linkbot enableButtonCallback*/
EXPORTCH void CLinkbotI_enableButtonCallback_chdl(void *varg) {
    return; 
    #if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    void *data;
    void (*cb)(void*, int, int);    

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    data=Ch_VaArg(interp, ap, void*);
    cb=(void(*)(void*, int, int))Ch_VaArg(interp, ap, void*);
    l->enableButtonCallback(data, cb);
    Ch_VaEnd(interp, ap);
    return;
    #endif
}

/*linkbot disableButtonCallback*/
EXPORTCH void CLinkbotI_disableButtonCallback_chdl(void *varg) {
    return;
    #if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->disableButtonCallback();
    Ch_VaEnd(interp, ap);
    return;
    #endif
}

/*linkbot delaySeconds*/
EXPORTCH void CLinkbotI_delaySeconds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    int seconds;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	seconds=Ch_VaArg(interp, ap, int);
    l->delaySeconds(seconds);
    Ch_VaEnd(interp, ap);
    return;
}
