/* Sample code to test the Ch binding*/

#include"linkbot.h"

double radius = 1.75;
double seconds = 5;
double angle=90;
double distance=20;
double trackwidth = 3.69;
double t;

CLinkbotI robot;
sleep(2);
if(robot.connect("4QFS")) {
    printf("Connect failed.\n");
    exit(-1);
}

robot.move(30, 30, 30);
robot.move(-30, -30, -30);

robot.driveTime(seconds);
robot.systemTime(t);
printf("t = %lf\n", t);




