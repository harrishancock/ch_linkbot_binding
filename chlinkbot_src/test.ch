/* File: recordangleneg.ch
   Record a joint angle and time, plot the acquired data,
   with a negative slope */
#include <linkbot.h>
#include <chplot.h>
CLinkbotI robot;
double speed = 45;         // speed in 45 degrees/seconds 
double timeInterval = 0.1; // time interval in 0.1 second 
int numDataPoints;         // number of data points recorded
robotRecordData_t timedata, angledata; // recorded time and angles for joint 1
CPlot plot;                // plotting class
double radius = 1.75;

/* connect to the paired robot and move to the zero position */
robot.connect();
robot.resetToZero();

/*robot.setJointSpeed(JOINT1, speed);
robot.setJointSpeed(JOINT3, speed);*/

//robot.recordAngleBegin(JOINT1, timedata, angledata, timeInterval);
robot.recordDistanceBegin(timedata, angledata, radius, timeInterval, 0);


robot.move(180, NaN, -180);


//robot.recordAngleEnd(JOINT1, numDataPoints);
robot.recordDistanceEnd(JOINT1, numDataPoints);

plot.mathCoord();
plot.title("Angles for joint 1 versus time");
plot.label(PLOT_AXIS_X, "time (seconds)");
plot.label(PLOT_AXIS_Y, "angle for joint1 (degrees)");
plot.data2DCurve(timedata, angledata, numDataPoints);
plot.plotting();


