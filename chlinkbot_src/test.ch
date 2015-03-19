/*Sample code for testing the recording functions*/
#include <linkbot.h>
#include <chplot.h>
CLinkbotI robot1, robot2;
CLinkbotIGroup group;

double speed = 45;         // speed in 45 degrees/seconds 
double timeInterval = 0.1; // time interval in 0.1 second 
int numDataPoints;         // number of data points recorded
double radius = 1.75;
robotRecordData_t timedata, distancedata; // recorded time and angles for joint 1
CPlot plot;                // plotting class
double seconds = 5.5;
double angle = 180;

group.addRobot(robot1);
group.addRobot(robot2);

group.connect();

group.driveBackward(angle);



