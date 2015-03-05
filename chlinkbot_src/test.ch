/* Sample code to test the Ch binding*/

#include"linkbot.h"

double radius = 1.75;
double seconds = 5;
double angle=90;
double distance=5;
double trackwidth = 3.69;
double t;
double angle1, angle2, angle3;
int r, g, b;
string_t color;
int moving=0;
double a1, a3, da, da_2;


/*CLinkbotIGroup group;
printf("addRobot\n");
group.addRobot("SRS8");
group.addRobot("TP51");
group.connect();
group.moveJointNB(JOINT1, 180);
group.moveJointWait(JOINT1);*/
CLinkbotI robot;
robot.connect();
robot.openGripperNB(20);
robot.moveWait();
robot.closeGripperNB();
robot.moveWait();


