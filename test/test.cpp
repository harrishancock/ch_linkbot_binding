#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#define sleep(x) Sleep((x)*1000)
#endif

#include <iostream>
#include "barobo/linkbot.hpp"

int main(int argc, char *argv[]) {
    if(argc != 2) {
        std::cout << "Usage: " << argv[0] << " <serial_id>"<<std::endl;
        return 0;
    }

    Linkbot *l = new Linkbot(argv[1]);
    sleep(2);
	double seconds=5;
    l->connect();
    /*l->move(90, 90, 90);
    l->move(-90, -90, -90);
    l->setMovementStateTime(
            ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_FORWARD, 3);
    l->setMovementStateTime(
            ROBOT_BACKWARD, ROBOT_BACKWARD, ROBOT_BACKWARD, 3);
    l->setMovementStateTime(
            ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_POSITIVE, 3);
    l->setMovementStateTime(
            ROBOT_NEGATIVE, ROBOT_NEGATIVE, ROBOT_NEGATIVE, 3);*/
	l->driveTime(seconds);
	std::cout<<"movement 1 done"<<std::endl;
	l->driveTimeNB(seconds);
	std::cout<<"movement 2 done"<<std::endl;
    return 0;
}
