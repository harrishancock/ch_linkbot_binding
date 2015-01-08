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


robot.moveToNB(180, 0, 180);
robot.moveWait();
sleep(5);
robot.getJointAnglesInstant(angle1, angle2, angle3);
printf("angle1, angle2, angle3 %lf %lf %lf\n", angle1, angle2, angle3);

robot.moveJointTo(JOINT3, 20);

robot.systemTime(t);
printf("t = %lf\n", t);




