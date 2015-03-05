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

CLinkbotI robot1, robot2;
CLinkbotIGroup group;
group.addRobot(robot1);
group.addRobot(robot2);
group.connect();
group.moveJoint(JOINT1, 180);



