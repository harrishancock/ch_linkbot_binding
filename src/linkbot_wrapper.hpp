#ifndef LINKBOT_WRAPPER_HPP_
#define LINKBOT_WRAPPER_HPP_

struct LinkbotImpl;

class Linkbot {
    public:
        Linkbot(const char* serialID);
        ~Linkbot();
        void connect();

        void move(double j1, double j2, double j3);
        void moveNB(double j1, double j2, double j3);
        void moveWait();

        LinkbotImpl *m;
};

#endif
