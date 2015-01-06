/* Sample code to test the Ch binding*/

#include"linkbot.h"

const char* id = "SRS8";
printf("id %s\n", id);
double radius = 1.75;
double seconds = 5;
double angle=90;
double distance=20;
double trackwidth = 3.69;
int i;
double x, y, z;
int r, g , b;
CLinkbotI robot;
sleep(2);
robot.connect();
while(i < 10)
{
    robot.getAccelerometerData(x, y, z);
    robot.getLEDColorRGB(r, g, b);
    printf("x %lf, y %lf, z %lf\n", x, y, z);
    printf("r %d, g %d, b %d\n", r, g, b);
    sleep(2);
    i++;
}
robot.move(180,0,0);
robot.disconnect();


