/* File: setjointspeedratio.ch 
   Get and set joint speed ratio. */
#include <linkbot.h>
CLinkbotI robot;
double ratio;   // joint speed ratio
double speed;  // the joint speed in degrees per second

/* get the joint speed ratio for joint 1 */
robot.getJointSpeedRatio(JOINT1, ratio);
printf("Joint1 speed ratio = %lf\n", ratio);

/* set the joint speed ratio for joint 1 to 0.75 (75% of the max speed).  */
ratio = 0.75;
robot.setJointSpeedRatio(JOINT1, ratio);

/* rotate joint 1 by 180 degrees */
robot.moveJoint(JOINT1, 180);

/* get the joint speed ratio for joint 1 */
robot.getJointSpeedRatio(JOINT1, ratio);
printf("Joint1 speed ratio = %lf\n", ratio);

/* get the joint speed for joint 1 */
robot.getJointSpeed(JOINT1, speed);
printf("Joint1 speed = %lf degrees per second\n", speed);
