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
robotRecordData_t timedata, angledata1, angledata2, angledata3; // recorded time and angles for joint 1
CPlot plot;                // plotting class
double radius = 1.75;
double offset = 3;
int connected;

robot1.connect();
robot1.recordAngleBegin(
    1,
    timedata,
    angledata1,
    0.1,
    1);
sleep(2);
robot1.move(90, 90, 90);
int num;
robot1.recordAngleEnd(1, num);
int i;
for(i = 0; i < num; i++) {
    printf("%f\t%f\n", timedata[i], 
        angledata1[i]);
}
    



