/* Sample code to test the Ch binding*/

#include"linkbot.h"

const char* id = "SRS8";
printf("id %s\n", id);
double radius = 1.75;
double seconds = 5;
double angle=90;
double distance=20;
double trackwidth = 3.69;
double t;

CLinkbotI robot;
sleep(2);
robot.connect();

robot.driveTime(seconds);
robot.systemTime(t);
printf("t = %lf\n", t);




