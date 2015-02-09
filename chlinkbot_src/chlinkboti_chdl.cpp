#include"../include/barobo/linkbot.hpp"
#include<ch.h>
#include<stdio.h>

/*class creator*/
EXPORTCH void CLinkbotI_CLinkbotI_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    
    Ch_VaStart(interp, ap, varg);
    class Linkbot *l= new Linkbot();
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
	if(Ch_CppIsArrayElement(interp)){
        l->~Linkbot();
	}
	else{
        delete l;
	}
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot connect*/
EXPORTCH int CLinkbotI_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    const char *id;
	int type;
    int rc;
    Ch_VaStart(interp, ap, varg);
   
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id = Ch_VaArg(interp, ap, const char*);
    printf("Connecting to %s\n", id);
    rc = l->connect(id);
    Ch_VaEnd(interp, ap);
    l->getFormFactor(type);
	if (type == 1)
	{
		printf("A LinkbotL is connected, not a LinkbotI.\nPlease connect a LinbotI. Exit..\n");
		exit(-1);
	}

    return rc;
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

/*linkbot isMoving*/
EXPORTCH int CLinkbotI_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    int mask;  
	int retval;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    mask=Ch_VaArg(interp, ap, int);
    retval=l->isMoving(mask);
    Ch_VaEnd(interp, ap);
    return retval;
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

/*linkbot openGripper*/
EXPORTCH void CLinkbotI_openGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	double angle;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	angle=Ch_VaArg(interp, ap, double);
    l->openGripper(angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot openGripperNB*/
EXPORTCH void CLinkbotI_openGripperNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	double angle;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	angle=Ch_VaArg(interp, ap, double);
    l->openGripperNB(angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointTo*/
EXPORTCH void CLinkbotI_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	robotJointId_t id;
	double angle;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	angle=Ch_VaArg(interp, ap, double);
    l->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointToNB*/
EXPORTCH void CLinkbotI_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	robotJointId_t id;
	double angle;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	angle=Ch_VaArg(interp, ap, double);
    l->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveTo*/
EXPORTCH void CLinkbotI_moveTo_chdl(void *varg) {
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
    l->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveToNB*/
EXPORTCH void CLinkbotI_moveToNB_chdl(void *varg) {
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
    l->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveToZero*/
EXPORTCH void CLinkbotI_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->moveToZero();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveToZeroNB*/
EXPORTCH void CLinkbotI_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbot closeGripper*/
EXPORTCH void CLinkbotI_closeGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->closeGripper();
    Ch_VaEnd(interp, ap);
    return;
}
/*END MOVEMENT FUNCTIONS*/
/*GET FUNCTIONS*/

/*linkbot getAccelerometerData*/
EXPORTCH void CLinkbotI_getAccelerometerData_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double *x;
    double *y;
	double *z;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    x=Ch_VaArg(interp, ap, double *);
    y=Ch_VaArg(interp, ap, double *);
	z=Ch_VaArg(interp, ap, double *);
    l->getAccelerometerData(*x, *y, *z);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getLEDColorRGB*/
EXPORTCH void CLinkbotI_getLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    int *r;
    int *g;
	int *b;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    r=Ch_VaArg(interp, ap, int *);
    g=Ch_VaArg(interp, ap, int *);
	b=Ch_VaArg(interp, ap, int *);
    l->getLEDColorRGB(*r, *g, *b);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getLEDColor*/
EXPORTCH void CLinkbotI_getLEDColor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    char * color;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    color=Ch_VaArg(interp, ap, char *);
    l->getLEDColor(color);
    Ch_VaEnd(interp, ap);
    return;
}


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

/*linkbot getJointAngleInstant*/
EXPORTCH void CLinkbotI_getJointAngleInstant_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    double *angle;
    robotJointId_t id;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    angle=Ch_VaArg(interp, ap, double *);
    l->getJointAngleInstant(id, *angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointAnglesInstant*/
EXPORTCH void CLinkbotI_getJointAnglesInstant_chdl(void *varg) {
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
    l->getJointAnglesInstant(*angle1, *angle2, *angle3);
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


/*linkbot setBuzzerFrequnencyOn*/
EXPORTCH void CLinkbotI_setBuzzerFrequencyOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    int frequency;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    frequency=Ch_VaArg(interp, ap, int);
    l->setBuzzerFrequencyOn(frequency);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setBuzzerFrequnencyOff*/
EXPORTCH void CLinkbotI_setBuzzerFrequencyOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->setBuzzerFrequencyOff();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setBuzzerFrequnency*/
EXPORTCH void CLinkbotI_setBuzzerFrequency_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    int frequency;
	double time;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    frequency=Ch_VaArg(interp, ap, int);
	time=Ch_VaArg(interp, ap, double);
    l->setBuzzerFrequency(frequency, time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setSpeed*/
EXPORTCH void CLinkbotI_setSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	double speed;
	double radius;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    speed=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
    l->setSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setLEDColorRGB*/
EXPORTCH void CLinkbotI_setLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	int r;
	int g;
	int b;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    r=Ch_VaArg(interp, ap, int);
	g=Ch_VaArg(interp, ap, int);
	b=Ch_VaArg(interp, ap, int);
    l->setLEDColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setLEDColor*/
EXPORTCH void CLinkbotI_setLEDColor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	char *color;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    color=Ch_VaArg(interp, ap, char*);
    l->setLEDColor(color);
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

/*linkbot systemTime*/
EXPORTCH void CLinkbotI_systemTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	double *time;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	time=Ch_VaArg(interp, ap, double *);
    l->systemTime(*time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot resetToZeroNB*/
EXPORTCH void CLinkbotI_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot resetToZero*/
EXPORTCH void CLinkbotI_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->resetToZero();
    Ch_VaEnd(interp, ap);
    return;
}

/* CLinkbotIGroup functions */

/*Constructor*/
EXPORTCH void CLinkbotIGroup_CLinkbotIGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;

  Ch_VaStart(interp, ap, varg);
  class LinkbotGroup *c=new LinkbotGroup();
  Ch_CppChangeThisPointer(interp, c, sizeof(LinkbotGroup));
  Ch_VaEnd(interp, ap);
}

/*Destructor*/
EXPORTCH void CLinkbotIGroup_dCLinkbotIGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *c;
  
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class LinkbotGroup *);
  if(Ch_CppIsArrayElement(interp)){
    c->~LinkbotGroup();
	printf("Calling group destructor\n");
  }
  else {
    delete c;
  }
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup addRobot*/
EXPORTCH void CLinkbotIGroup_addRobot_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    char *id;

    Ch_VaStart(interp, ap, varg);
   
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    id = Ch_VaArg(interp, ap, char*);
    printf("Adding %s\n", id);
    g->addRobot(id);
    Ch_VaEnd(interp, ap);
}

/*linkbotGroup connect*/
EXPORTCH void CLinkbotIGroup_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;

    Ch_VaStart(interp, ap, varg);

    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->connect();
    Ch_VaEnd(interp, ap);
}

/*linkbotGroup driveDistanceNB*/
EXPORTCH void CLinkbotIGroup_driveDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    double distance;
	double radius;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    distance=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
    g->driveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup driveDistance*/
EXPORTCH void CLinkbotIGroup_driveDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    double distance;
	double radius;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    distance=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
    g->driveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveWait*/
EXPORTCH void CLinkbotIGroup_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->moveWait();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup driveBackwardNB*/
EXPORTCH void CLinkbotIGroup_driveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    angle=Ch_VaArg(interp, ap, double);
    g->driveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotGroup driveBackward*/
EXPORTCH void CLinkbotIGroup_driveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    angle=Ch_VaArg(interp, ap, double);
    g->driveBackward(angle);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotGroup driveForeverNB*/
EXPORTCH void CLinkbotIGroup_driveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
   
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->driveForeverNB();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup driveForwardNB*/
EXPORTCH void CLinkbotIGroup_driveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    angle=Ch_VaArg(interp, ap, double);
    g->driveForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotGroup driveForward*/
EXPORTCH void CLinkbotIGroup_driveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    angle=Ch_VaArg(interp, ap, double);
    g->driveForward(angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup driveTimeNB*/
EXPORTCH void CLinkbotIGroup_driveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    double time;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    time=Ch_VaArg(interp, ap, double);
    g->driveTimeNB(time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup driveTime*/
EXPORTCH void CLinkbotIGroup_driveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    double time;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    time=Ch_VaArg(interp, ap, double);
    g->driveTime(time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup holdJoint*/
EXPORTCH void CLinkbotIGroup_holdJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    robotJointId_t id;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    g->holdJoint(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup holdJoints*/
EXPORTCH void CLinkbotIGroup_holdJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->holdJoints();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup turnLeftNB*/
EXPORTCH void CLinkbotIGroup_turnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double angle;
	double radius;
	double tracklength;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    g->turnLeftNB(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup turnLeft*/
EXPORTCH void CLinkbotIGroup_turnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double angle;
	double radius;
	double tracklength;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    g->turnLeft(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotGroup turnRightNB*/
EXPORTCH void CLinkbotIGroup_turnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double angle;
	double radius;
	double tracklength;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    g->turnRightNB(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotGroup turnRight*/
EXPORTCH void CLinkbotIGroup_turnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double angle;
	double radius;
	double tracklength;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    g->turnRight(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup relaxJoint*/
EXPORTCH void CLinkbotIGroup_relaxJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;

    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
    g->relaxJoint(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup relaxJoints*/
EXPORTCH void CLinkbotIGroup_relaxJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;

    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->relaxJoints();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup stop*/
EXPORTCH void CLinkbotIGroup_stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;

    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->stop();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveNB*/
EXPORTCH void CLinkbotIGroup_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double j1;
	double j2;
	double j3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	j1=Ch_VaArg(interp, ap, double);
	j2=Ch_VaArg(interp, ap, double);
	j3=Ch_VaArg(interp, ap, double);
    g->moveNB(j1, j2, j3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup move*/
EXPORTCH void CLinkbotIGroup_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double j1;
	double j2;
	double j3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	j1=Ch_VaArg(interp, ap, double);
	j2=Ch_VaArg(interp, ap, double);
	j3=Ch_VaArg(interp, ap, double);
    g->move(j1, j2, j3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveForeverNB*/
EXPORTCH void CLinkbotIGroup_moveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->moveForeverNB();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveToNB*/
EXPORTCH void CLinkbotIGroup_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double angle1;
	double angle2;
	double angle3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	angle1=Ch_VaArg(interp, ap, double);
	angle2=Ch_VaArg(interp, ap, double);
	angle3=Ch_VaArg(interp, ap, double);
    g->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveTo*/
EXPORTCH void CLinkbotIGroup_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double angle1;
	double angle2;
	double angle3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	angle1=Ch_VaArg(interp, ap, double);
	angle2=Ch_VaArg(interp, ap, double);
	angle3=Ch_VaArg(interp, ap, double);
    g->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveToNB*/
EXPORTCH void CLinkbotIGroup_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveToNB*/
EXPORTCH void CLinkbotIGroup_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->moveToZero();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveToByTrackPosNB*/
EXPORTCH void CLinkbotIGroup_moveToByTrackPosNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double angle1;
	double angle2;
	double angle3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	angle1=Ch_VaArg(interp, ap, double);
	angle2=Ch_VaArg(interp, ap, double);
	angle3=Ch_VaArg(interp, ap, double);
    g->moveToByTrackPosNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveToByTrackPos*/
EXPORTCH void CLinkbotIGroup_moveToByTrackPos_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double angle1;
	double angle2;
	double angle3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	angle1=Ch_VaArg(interp, ap, double);
	angle2=Ch_VaArg(interp, ap, double);
	angle3=Ch_VaArg(interp, ap, double);
    g->moveToByTrackPos(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveJointNB*/
EXPORTCH void CLinkbotIGroup_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}


/*linkbotGroup moveJoint*/
EXPORTCH void CLinkbotIGroup_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}


/*linkbotGroup moveJointForeverNB*/
EXPORTCH void CLinkbotIGroup_moveJointForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
    g->moveJointForeverNB(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveJointToNB*/
EXPORTCH void CLinkbotIGroup_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveJointTo*/
EXPORTCH void CLinkbotIGroup_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveJointToByTrackPosNB*/
EXPORTCH void CLinkbotIGroup_moveJointToByTrackPosNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJointToByTrackPosNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveJointToByTrackPos*/
EXPORTCH void CLinkbotIGroup_moveJointToByTrackPos_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJointToByTrackPos(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup openGripperNB*/
EXPORTCH void CLinkbotIGroup_openGripperNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
    g->openGripperNB(angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup openGripper*/
EXPORTCH void CLinkbotIGroup_openGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
    g->openGripper(angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup setJointSpeed*/
EXPORTCH void CLinkbotIGroup_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	double speed;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	speed=Ch_VaArg(interp, ap, double);
    g->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup setJointSpeedRatio*/
EXPORTCH void CLinkbotIGroup_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	double ratio;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	ratio=Ch_VaArg(interp, ap, double);
    g->setJointSpeedRatio(id, ratio);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup setJointSpeeds*/
EXPORTCH void CLinkbotIGroup_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double speed1;
	double speed2;
	double speed3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	speed1=Ch_VaArg(interp, ap, double);
	speed2=Ch_VaArg(interp, ap, double);
	speed3=Ch_VaArg(interp, ap, double);
    g->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup setJointSpeedRatios*/
EXPORTCH void CLinkbotIGroup_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double ratio1;
	double ratio2;
	double ratio3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	ratio1=Ch_VaArg(interp, ap, double);
	ratio2=Ch_VaArg(interp, ap, double);
	ratio3=Ch_VaArg(interp, ap, double);
    g->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup setSpeed*/
EXPORTCH void CLinkbotIGroup_setSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double speed;
	double radius;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	speed=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
    g->setSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup resetToZero*/
EXPORTCH void CLinkbotIGroup_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->resetToZero();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup resetToZeroNB*/
EXPORTCH void CLinkbotIGroup_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return;
}


