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


CLinkbotIGroup group;
printf("addRobot\n");
group.addRobot("SRS8");
group.addRobot("TP51");
group.connect();
group.move(180, 0, -180);
group.resetToZero();
//group.moveWait();
/*CLinkbotL robot;
robot.connect("JBPC");
robot.moveNB(180, 80, 0);
robot.moveWait();
robot.resetToZero();
robot.moveWait();*/

















