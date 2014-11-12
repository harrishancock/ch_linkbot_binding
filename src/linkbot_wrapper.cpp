#include <iostream>
namespace c_impl {
#include "baromesh/linkbot.h"
}
#include "linkbot_wrapper.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/thread/condition_variable.hpp"

#define RUNTIME_ERROR \
    std::runtime_error(std::string("Exception in ") + std::string(__func__))


struct LinkbotImpl
{
    friend void _jointEventCB(int, c_impl::barobo::JointState::Type, int, void*);
    void jointEventCB(int jointNo, c_impl::barobo::JointState::Type state);
    boost::mutex jointStateMutex;
    boost::condition_variable jointStateCond;
    c_impl::barobo::JointState::Type jointStates[3];
    c_impl::baromesh::Linkbot *linkbot;
    uint8_t motorMask;
};


void _jointEventCB(int joint, c_impl::barobo::JointState::Type state, int timestamp, void *data)
{
    std::cout << "Received joint event." << std::endl;
    LinkbotImpl *l = static_cast<LinkbotImpl*>(data);
    l->jointEventCB(joint, state);
}

Linkbot::Linkbot(const char* serialId)
{
    m = new LinkbotImpl();
    m->linkbot = c_impl::linkbotNew(serialId);
    for(int i = 0; i < 3; i++) {
        m->jointStates[i] = c_impl::barobo::JointState::STOP;
    }
}

Linkbot::~Linkbot()
{
}

void Linkbot::connect()
{
    c_impl::linkbotConnect(m->linkbot);
    /* Enable joint callbacks */
    c_impl::linkbotSetJointEventCallback(m->linkbot, _jointEventCB, m);
    /* Get the form factor */
    c_impl::barobo::FormFactor::Type formFactor;
    c_impl::linkbotGetFormFactor(m->linkbot, &formFactor);
    switch (formFactor) {
        case c_impl::barobo::FormFactor::I:
            m->motorMask = 0x05;
            break;
        case c_impl::barobo::FormFactor::L:
            m->motorMask = 0x03;
            break;
        case c_impl::barobo::FormFactor::T:
            m->motorMask = 0x07;
            break;
    }
}

void Linkbot::move(double j1, double j2, double j3)
{
    moveNB(j1, j2, j3);
    moveWait();
}

void Linkbot::moveNB(double j1, double j2, double j3)
{
    c_impl::linkbotMove(m->linkbot, 0x07, j1, j2, j3);
}

void Linkbot::moveWait()
{
    /* Get the current joint states */
    std::cout << "moveWait()" << std::endl;
    int time;
    boost::unique_lock<boost::mutex> lock(m->jointStateMutex);

    c_impl::linkbotGetJointStates(
        m->linkbot,
        &time,
        &m->jointStates[0],
        &m->jointStates[1],
        &m->jointStates[2]);

    std::cout << "Jointstates: " << m->jointStates[0] << " "
                                 << m->jointStates[1] << " "
                                 << m->jointStates[2] << std::endl;

    for(int i = 0; i < 3; i++) {
        if(!((1<<i)&m->motorMask)) { continue; }
        while (m->jointStates[i] == c_impl::barobo::JointState::MOVING) {
            std::cout << "Waiting on " << i << std::endl;
            m->jointStateCond.wait(lock);
        }
    }
}

void LinkbotImpl::jointEventCB(int jointNo, c_impl::barobo::JointState::Type state)
{
    boost::lock_guard<boost::mutex> lock(jointStateMutex);
    std::cout << "Setting joint " << jointNo << " To " << state << std::endl;
    jointStates[jointNo] = state;
    jointStateCond.notify_all();
}
