#include <unistd.h>
#include "barobo/linkbot.hpp"

int main() {
    Linkbot *l = new Linkbot("13Z8");
    sleep(2);
    l->connect();
    l->setMovementStateTime(
            ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_FORWARD, 3);
    l->setMovementStateTime(
            ROBOT_BACKWARD, ROBOT_BACKWARD, ROBOT_BACKWARD, 3);
    l->setMovementStateTime(
            ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_POSITIVE, 3);
    l->setMovementStateTime(
            ROBOT_NEGATIVE, ROBOT_NEGATIVE, ROBOT_NEGATIVE, 3);
    return 0;
}
