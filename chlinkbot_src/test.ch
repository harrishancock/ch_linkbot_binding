/*Sample code for testing the recording functions*/
#include <linkbot.h>
#include <chplot.h>
CLinkbotI robot;

double speed = 45;         // speed in 45 degrees/seconds 
double timeInterval = 0.1; // time interval in 0.1 second 
int numDataPoints;         // number of data points recorded
double radius = 1.75;
robotRecordData_t timedata, distancedata; // recorded time and angles for joint 1
CPlot plot;                // plotting class
double seconds = 5.5;

robot.connect();

robot.moveJointTime(JOINT1, seconds);



