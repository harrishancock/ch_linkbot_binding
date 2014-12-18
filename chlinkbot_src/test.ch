/* Sample code to test the Ch binding*/

#include"linkbot.h"

const char* id = "SRS8";
printf("id %s\n", id);
double radius = 1.75;
double seconds = 5;
double angle=90;
double distance=20;
double trackwidth = 3.69;
CLinkbotI robot;
sleep(2);
robot.connect();
robot.turnRight(angle, radius, trackwidth);
printf("Movement 1 done\n");
robot.holdJoints();
printf("Movement 2 done\n");
