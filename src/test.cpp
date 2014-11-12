#include <unistd.h>
#include "linkbot_wrapper.hpp"

int main() {
    Linkbot *l = new Linkbot("LOCL");
    sleep(2);
    l->connect();
    /*
    l->move(0, 90, 90);
    l->move(0, -90, -90);
    l->move(0, 90, 90);
    l->move(0, -90, -90);
    l->move(90, 90, 0);
    l->move(-90, -90, 0);
    l->move(90, 90, 0);
    l->move(-90, -90, 0);
    */
    return 0;
}
