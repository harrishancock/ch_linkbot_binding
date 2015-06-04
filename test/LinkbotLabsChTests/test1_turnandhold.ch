/* File: turnandhold.ch 
   Turn left and turn right, hold joints at exit */
#include <linkbot.h>
CLinkbotI robot;
double radius = 1.75;      // radius of 1.75 inches 
double trackwidth = 3.69;  // the track width, the distance between two wheels

robot.driveAngle(360);
robot.turnRight(90, radius, trackwidth);
robot.driveAngle(360);
robot.turnLeft(90, radius, trackwidth);
robot.driveAngle(360);

/* Hold the joints at exit .*/
robot.holdJointsAtExit();