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
    l->connect();

    for(double p = -1; p < 1.0; p += 0.01) {
        l->setMotorPower(ROBOT_JOINT1, p);
    }
    return 0;
}
