/* Sample code to test the Ch binding*/

#include"linkbot.h"

const char* id = "SRS8";
printf("id %s\n", id);
double radius = 1.75;
double seconds = 5;
double angle=180;
double distance=20;
CLinkbotI robot;
sleep(2);
robot.connect();
robot.moveToByTrackPos(angle, angle, angle);
printf("Movement 2 done\n");

