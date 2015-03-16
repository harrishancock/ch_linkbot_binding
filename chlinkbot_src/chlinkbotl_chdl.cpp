#include"../include/barobo/linkbot.hpp"
#include<ch.h>
#include<stdio.h>

/*class creator*/
EXPORTCH void CLinkbotL_CLinkbotL_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    
    Ch_VaStart(interp, ap, varg);
    class Linkbot *l= new Linkbot();
    Ch_CppChangeThisPointer(interp, l, sizeof(Linkbot));
    Ch_VaEnd(interp, ap);
    return;
}
 
/* class destructor*/   
EXPORTCH void CLinkbotL_dCLinkbotL_chdl(void *varg) {
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
EXPORTCH int CLinkbotL_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	int type;
    int rc;
    Ch_VaStart(interp, ap, varg);
   
    l=Ch_VaArg(interp, ap, class Linkbot *);
    rc = l->connect();
	printf("Connected\n");
    Ch_VaEnd(interp, ap);
    l->getFormFactor(type);
	if (type == 0)
	{
		printf("WARNING: A Linkbot-I is connected, not a Linkbot-L.\nPlease connect a Linbot-L.\nExiting...\n");
		exit(-1);
	}
    return rc;
}

/*linkbot connectWithSerialID*/
EXPORTCH int CLinkbotL_connectWithSerialID_chdl(void *varg) {
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
    rc = l->connectWithSerialID(id);
    Ch_VaEnd(interp, ap);
    l->getFormFactor(type);
	if (type == 0)
	{
		printf("WARNING: A Linkbot-I is connected, not a Linkbot-L.\nPlease connect a Linbot-L.\nExiting...\n");
		exit(-1);
	}
    return rc;
}

/*linkbot disconnect*/
EXPORTCH void CLinkbotL_disconnect_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_move_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveToByTrackPos_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveToByTrackPosNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveJointToByTrackPos_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveJointToByTrackPosNB_chdl(void *varg) {
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

/*linkbot moveWait*/
EXPORTCH void CLinkbotL_moveWait_chdl(void *varg) {
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

/*linkbot moveJointWait*/
EXPORTCH void CLinkbotL_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;    
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    id=Ch_VaArg(interp, ap, robotJointId_t);
    l->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot stop*/
EXPORTCH void CLinkbotL_stop_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_stopOneJoint_chdl(void *varg) {
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

/*linkbot isMoving*/
EXPORTCH int CLinkbotL_isMoving_chdl(void *varg) {
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

/*linkbot holdJoint*/
EXPORTCH void CLinkbotL_holdJoint_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_holdJoints_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_relaxJoint_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_relaxJoints_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveForeverNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveJoint_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveJointNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveJointForeverNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveTime_chdl(void *varg) {
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

/*linkbot moveTimeNB*/
EXPORTCH void CLinkbotL_moveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
	double time;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	time=Ch_VaArg(interp, ap, double);
    l->moveTimeNB(time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointTime*/
EXPORTCH void CLinkbotL_moveJointTime_chdl(void *varg) {
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

/*linkbot moveJointTimeNB*/
EXPORTCH void CLinkbotL_moveJointTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    robotJointId_t id;
	double time;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	time=Ch_VaArg(interp, ap, double);
    l->moveJointTimeNB(id, time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointTo*/
EXPORTCH void CLinkbotL_moveJointTo_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveJointToNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveTo_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveToNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveToZero_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return;
}

/*END MOVEMENT FUNCTIONS*/
/*GET FUNCTIONS*/

/*linkbot getAccelerometerData*/
EXPORTCH void CLinkbotL_getAccelerometerData_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_getLEDColorRGB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_getLEDColor_chdl(void *varg) {
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

/*linkbot getJointAngleInstant*/
EXPORTCH void CLinkbotL_getJointAngleInstant_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_getJointAnglesInstant_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_getJointAngle_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_getJointAngles_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_getJointSpeed_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_getJointSpeedRatio_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_getJointSpeeds_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_getJointSpeedRatios_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setJointMovementStateNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setJointMovementStateTime_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setJointSpeed_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setJointSpeeds_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setJointSpeedRatio_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setJointSpeedRatios_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setJointPower_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setMotorPowers_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setMovementStateNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setMovementStateTime_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setMovementStateTimeNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setBuzzerFrequencyOn_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setBuzzerFrequencyOff_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setBuzzerFrequency_chdl(void *varg) {
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

/*linkbot setLEDColorRGB*/
EXPORTCH void CLinkbotL_setLEDColorRGB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_setLEDColor_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_enableButtonCallback_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_disableButtonCallback_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_delaySeconds_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_systemTime_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_resetToZeroNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotL_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class Linkbot *);
    l->resetToZero();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot recordAngleBegin*/
EXPORTCH void CLinkbotL_recordAngleBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class Linkbot *l;
	robotJointId_t id;
	double** time;
	double** angle;
	double seconds;
	int shiftData;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class Linkbot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	time = Ch_VaArg(interp, ap, double**);
	angle = Ch_VaArg(interp, ap, double**);
	seconds = Ch_VaArg(interp, ap, double);
	if (Ch_VaCount(interp, ap) == 1) {
		shiftData = Ch_VaArg(interp, ap, int);
		l->recordAnglesBegin(*time, *angle, *angle, *angle, seconds, 1 << (int(id) - 1), shiftData);
	}
	else {
		l->recordAnglesBegin(*time, *angle, *angle, *angle, seconds, 1 << (int(id) - 1));
	}
	Ch_VaEnd(interp, ap);
	return;
}

/*linkbot recordAngleEnd*/
EXPORTCH void CLinkbotL_recordAngleEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class Linkbot *l;
	robotJointId_t id;
	int *num;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class Linkbot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	num = Ch_VaArg(interp, ap, int*);
	l->recordAnglesEnd(*num);
	Ch_VaEnd(interp, ap);
	return;
}

/*linkbot enableRecordDataShift*/
EXPORTCH void CLinkbotL_enableRecordDataShift_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class Linkbot *l;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class Linkbot *);
	l->enableRecordDataShift();
	Ch_VaEnd(interp, ap);
	return;
}

/*linkbot disableRecordDataShift*/
EXPORTCH void CLinkbotL_disableRecordDataShift_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class Linkbot *l;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class Linkbot *);
	l->disableRecordDataShift();
	Ch_VaEnd(interp, ap);
	return;
}


/* CLinkbotLGroup functions */

/*Constructor*/
EXPORTCH void CLinkbotLGroup_CLinkbotLGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;

  Ch_VaStart(interp, ap, varg);
  class LinkbotGroup *c=new LinkbotGroup();
  Ch_CppChangeThisPointer(interp, c, sizeof(LinkbotGroup));
  Ch_VaEnd(interp, ap);
}

/*Destructor*/
EXPORTCH void CLinkbotLGroup_dCLinkbotLGroup_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_addRobot_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    class Linkbot *l;

    Ch_VaStart(interp, ap, varg);
   
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	l=Ch_VaArg(interp, ap, class Linkbot *);
    g->addRobot(*l);
    Ch_VaEnd(interp, ap);
}

/*linkbotGroup connect*/
EXPORTCH void CLinkbotLGroup_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	int checkType;
	int type = 1;

    Ch_VaStart(interp, ap, varg);

    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->connect();
    Ch_VaEnd(interp, ap);

	checkType=g->checkFormFactor(type);
	if (checkType == -1){
		printf("WARNING: Not all the Linkbots in the group are Linkbot-Ls.\nPlease check the Linkbots.\nExiting...\n");
	    exit(-1);
	}
}

/*linkbotGroup moveWait*/
EXPORTCH void CLinkbotLGroup_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->moveWait();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveJointWait*/
EXPORTCH void CLinkbotLGroup_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
    g->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup holdJoint*/
EXPORTCH void CLinkbotLGroup_holdJoint_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_holdJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->holdJoints();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup relaxJoint*/
EXPORTCH void CLinkbotLGroup_relaxJoint_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_relaxJoints_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_stop_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_move_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveForeverNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveToNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveTo_chdl(void *varg) {
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

/*linkbotGroup moveToZeroNB*/
EXPORTCH void CLinkbotLGroup_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveToZero*/
EXPORTCH void CLinkbotLGroup_moveToZero_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveToByTrackPosNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveToByTrackPos_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveJointNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveJoint_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveJointForeverNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveJointToNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveJointTo_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveJointToByTrackPosNB_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_moveJointToByTrackPos_chdl(void *varg) {
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

/*linkbotGroup setJointSpeed*/
EXPORTCH void CLinkbotLGroup_setJointSpeed_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_setJointSpeedRatio_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_setJointSpeeds_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_setJointSpeedRatios_chdl(void *varg) {
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

/*linkbotGroup resetToZero*/
EXPORTCH void CLinkbotLGroup_resetToZero_chdl(void *varg) {
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
EXPORTCH void CLinkbotLGroup_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
    g->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup isMoving*/
EXPORTCH int CLinkbotLGroup_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	int mask;
	int retval;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	mask=Ch_VaArg(interp, ap, int);
    retval=g->isMoving(mask);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*linkbotGroup moveJointTime*/
EXPORTCH void CLinkbotLGroup_moveJointTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	double time;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	time=Ch_VaArg(interp, ap, double);
    g->moveJointTime(id, time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveJointTimeNB*/
EXPORTCH void CLinkbotLGroup_moveJointTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	robotJointId_t id;
	double time;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	id=Ch_VaArg(interp, ap, robotJointId_t);
	time=Ch_VaArg(interp, ap, double);
    g->moveJointTimeNB(id, time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveTime*/
EXPORTCH void CLinkbotLGroup_moveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double time;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	time=Ch_VaArg(interp, ap, double);
    g->moveTime(time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveTimeNB*/
EXPORTCH void CLinkbotLGroup_moveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class LinkbotGroup *g;
	double time;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class LinkbotGroup *);
	time=Ch_VaArg(interp, ap, double);
    g->moveTimeNB(time);
    Ch_VaEnd(interp, ap);
    return;
}

