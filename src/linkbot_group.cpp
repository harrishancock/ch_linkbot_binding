#include <iostream>
#include <vector>
namespace c_impl {
#include "baromesh/linkbot.h"
}
#include "barobo/linkbot.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/thread/condition_variable.hpp"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <stdlib.h>
#include <string.h>

#include "../include/rgbhashtable.h"

#define M_PI            3.14159265358979323846

#define RUNTIME_ERROR \
    std::runtime_error(std::string("Exception in ") + std::string(__func__))

#define CALL_C_IMPL(func, ...) \
do { \
    int rc = c_impl::func( m->linkbot, __VA_ARGS__ ); \
    if(rc != 0) { \
        throw std::runtime_error( \
            std::string("Error encountered calling function: <" #func \
                "> Error code: ") + std::to_string(rc)); \
    } \
} while(0)

#define ABS(x) ( (x<0) ? (-(x)) : (x) )

struct LinkbotGroupImpl
{
    std::vector<Linkbot*> robots;
    std::vector<std::string> ids;
    int argInt;
    double argDouble;
	int motionInProgress;
	void *thread;
};

LinkbotGroup::LinkbotGroup()
{
  m = new LinkbotGroupImpl();	
  m->motionInProgress = 0;
  //_thread = (THREAD_T*)malloc(sizeof(THREAD_T));
}

LinkbotGroup::~LinkbotGroup()
{
	
}

void LinkbotGroup::addRobot(char* serialID)
{
    m->ids.push_back(std::string(serialID));
}

void LinkbotGroup::connect()
{
    for (std::string id : m->ids) {
        m->robots.push_back(new Linkbot());
        m->robots.back()->connect(id.c_str());
    }
}
