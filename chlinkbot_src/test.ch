/* File: recordangleneg.ch
   Record a joint angle and time, plot the acquired data,
   with a negative slope */
#include <linkbot.h>
#include <chplot.h>
CLinkbotI robot1, robot2;
CLinkbotIGroup group;
double speed = 45;         // speed in 45 degrees/seconds 
double timeInterval = 0.1; // time interval in 0.1 second 
int numDataPoints;         // number of data points recorded
robotRecordData_t timedata, angledata; // recorded time and angles for joint 1
CPlot plot;                // plotting class
double radius = 1.75;
double offset = 3;
int connected;

group.addRobot(robot1);
group.addRobot(robot2);

group.connect();

connected=group.isConnected();
if (connected == 1){
    printf("Connected!\n");
}

group.resetToZero();
//robot.holdJointsAtExit();


group.driveDistance(5, radius);
//group.move(180, 180, NaN);





