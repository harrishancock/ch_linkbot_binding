/* Filename: speedcontrol.ch
   The joint angles for robot1 in green color will determine
   the speed of joints for robot2 in red color. */
#include <linkbot.h>
CLinkbotI robot1, robot2;
double angle1, angle3;
double speed1, speed3;

/* Set colors for robots. Use the green robot to control the red one */
robot1.setLEDColor("green");
robot2.setLEDColor("red");

/* relax all joints of robot1 */
robot1.relaxJoints();

/* set joints of robot2 to move forward */
robot2.driveForeverNB();

while(1) {
    robot1.getJointAngles(angle1, NaN, angle3);

    speed1 = angle1;
    if(speed1 > 240) {
        speed1 = 240;
    } 
    else if(speed1 < -240) {
        speed1 = -240;
    }

    speed3 = angle3;
    if(speed3 > 240) {
        speed3 = 240;
    } 
    else if(speed3 < -240) {
        speed3 = -240;
    }

    robot2.setJointSpeeds(speed1, NaN, -speed3);
}
