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
class CLinkbotI {
    public:
        CLinkbotI();
        ~CLinkbotI();
        void connect();

        /* GETTERS */

        void getDistance(double &distance, double radius);
        void getJointAngle(robotJointId_t id, double &angle);
        void getJointAngles(double &angle1, double &angle2, double &angle3);
        void getJointSpeed(robotJointId_t id, double &speed);
        void getJointSpeedRatio(robotJointId_t id, double &ratio);
        void getJointSpeeds(double &speed1, double &speed2, double &speed3);
        void getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);

        /* SETTERS */
        void setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
        void setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
        void setJointSpeed(robotJointId_t id, double speed);
        void setJointSpeeds(double speed1, double speed2, double speed3);
        void setJointSpeedRatio(robotJointId_t id, double ratio);
        void setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
        void setMotorPower(robotJointId_t id, int power);
        void setJointPower(robotJointId_t id, double power);
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
        void setTwoWheelRobotSpeed(double speed, double radius);

        /* MOVEMENT */

        void driveJointTo(robotJointId_t id, double angle);
        void driveJointToNB(robotJointId_t id, double angle);
        void driveTo(double angle1, double angle2, double angle3);
        void driveToNB(double angle1, double angle2, double angle3);
        void move(double j1, double j2, double j3);
        void moveNB(double j1, double j2, double j3);
        void moveWait();
        void moveContinuousNB(robotJointState_t dir1, 
                robotJointState_t dir2, 
                robotJointState_t dir3);
        void moveContinuousTime(robotJointState_t dir1, 
                robotJointState_t dir2, 
                robotJointState_t dir3, 
                double seconds);
        void moveDistance(double distance, double radius);
        void moveDistanceNB(double distance, double radius);
        void stop();
        void stopOneJoint(robotJointId_t id);
        void stopAllJoints();

        /* MISC */
        void enableButtonCallback(void* userdata, void (*buttonCallback)(void* data, int button, int buttonDown));
        void disableButtonCallback();

        LinkbotImpl *m;

        static void *g_chlinkbot_dlhandle;
        static int g_chlinkbot_dlcount;

        private:
        double mMaxSpeed;
        
};

void *CLinkbotI::g_chlinkbot_dlhandle=NULL;
int CLinkbotI::g_chlinkbot_dlcount=0;
#pragma importf "chlinkbot.chf"
#endif
