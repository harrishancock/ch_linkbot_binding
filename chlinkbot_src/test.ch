/* Sample code to test the Ch binding*/

#include"linkbot.h"

double radius = 1.75;
double seconds = 5;
double angle=90;
double distance=20;
double trackwidth = 3.69;
double t;
double angle1, angle2, angle3;

CLinkbotI robot;
sleep(2);
if(robot.connect("SRS8")) {
    printf("Connect failed.\n");
    exit(-1);
}

robot.move(20, 0, 80);




