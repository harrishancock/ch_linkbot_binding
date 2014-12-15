/* Sample code to test the Ch binding*/

#include"linkbot.h"

const char* id = "SRS8";
printf("id %s\n", id);
double distance;
double radius = 1.75;
double seconds = 5;
double angle1, angle2, angle3;
CLinkbotI robot;
sleep(2);
robot.connect();
robot.setJointSpeeds(50, 0, 50);
robot.move(180, 0, 180);
//robot.move(90, 90, 90);
//robot.moveNB(90, 90, 90);
//robot.moveWait();
sleep(2);
robot.getJointSpeed(ROBOT_JOINT3, angle1);
printf("Ratio1 = %.3lf\n",angle1);
robot.stop();
