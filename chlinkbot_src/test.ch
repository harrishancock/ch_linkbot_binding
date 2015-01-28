/* Sample code to test the Ch binding*/

#include"linkbot.h"

double radius = 1.75;
double seconds = 5;
double angle=90;
double distance=10;
double trackwidth = 3.69;
double t;
double angle1, angle2, angle3;
int r, g, b;
string_t color;


CLinkbotIGroup group;
printf("addRobot\n");
group.addRobot("ZC22");
printf("connect\n");
group.connect();

/*
CLinkbotI robot;
if(robot.connect("ZC22")) {
    printf("Connect failed.\n");
    exit(-1);
}
*/

/*robot.setLEDColorRGB(0, 255, 0);
robot.getLEDColorRGB(r, g, b);
printf("r %d, g %d, b %d\n", r, g, b);
sleep(5);*/
/*robot.setLEDColor("red");
robot.getLEDColor(color);
printf("color %s\n", color);

robot.driveDistance(distance, radius);*/





