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

CLinkbotI robot;
CLinkbotIGroup group;
printf("addRobot\n");
group.addRobot("SRS8");
group.addRobot("TP51");
printf("connect\n");
group.connect();
group.setJointSpeeds(45, 0, 45);
group.move(90, 0, -90);
/*group.setSpeed(5, radius);
group.driveForward(angle);*/
/*robot.connect("TP51");
robot.setJointSpeedRatios(0.5, 0, 0.8);
robot.move(90, 0, -90);*/
















