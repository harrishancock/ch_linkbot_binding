/*Sample code for testing the recording functions*/
#include <linkbot.h>
#include <chplot.h>
CLinkbotL robot;
double speed = 45;         // speed in 45 degrees/seconds 
double timeInterval = 0.1; // time interval in 0.1 second 
int numDataPoints;         // number of data points recorded
double radius = 1.75;
robotRecordData_t timedata, distancedata; // recorded time and angles for joint 1
CPlot plot;                // plotting class

/* connect to the paired robot and move to the zero position */
robot.connect();
robot.resetToZero();

/* set the joints 1 and 3 speed */
robot.setJointSpeed(JOINT1, speed);
robot.setJointSpeed(JOINT2, speed);

/* begin recording time and angle */
robot.recordAngleBegin(JOINT2, timedata, distancedata, radius, timeInterval);

/* move the Linkbot-I forward by 720 degrees */
robot.move(180, -180, NaN);

/* end recording time and angle */
robot.recordAngleEnd(JOINT2, numDataPoints);

/* plot the data */
plot.mathCoord();
plot.title("Angles for joint 1 versus time");
plot.label(PLOT_AXIS_X, "time (seconds)");
plot.label(PLOT_AXIS_Y, "angle (degrees)");
plot.data2DCurve(timedata, distancedata, numDataPoints);
plot.plotting();



