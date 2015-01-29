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

/*Structure that holds all the private members of the class*/
struct LinkbotGroupImpl 
{
    std::vector<Linkbot*> robots; /*array of istances of class linkbot*/
    std::vector<std::string> ids; /*array of the id of the robots*/
    int argInt;
    double argDouble;
	int motionInProgress;
	void *thread;
};

LinkbotGroup::LinkbotGroup()
{
  m = new LinkbotGroupImpl(); /*allocate the structure*/	
  m->motionInProgress = 0;
  //_thread = (THREAD_T*)malloc(sizeof(THREAD_T));
}

LinkbotGroup::~LinkbotGroup()
{
	
}

void LinkbotGroup::addRobot(char* serialID)
{
    m->ids.push_back(std::string(serialID)); /*add the serialID element at the end of the array */
}

void LinkbotGroup::connect()
{
    for (std::string id : m->ids) {
		/*declare a new element of class Linkbot and put it at the end of the array
		  push_back = add at the end*/
        m->robots.push_back(new Linkbot());  
		/*access the last element and call a member function
		  id.c_str() gets the C equivalent of the string*/
        m->robots.back()->connect(id.c_str());
    }
}

void LinkbotGroup::driveDistanceNB(double distance, double radius)
{
	for (Linkbot* robot : m->robots) {
		robot->driveDistanceNB(distance, radius);
	}
}

void LinkbotGroup::moveWait()
{
	int mask = 0x07;
	for (Linkbot* robot : m->robots) {
		//m->robots.back()->moveWait(mask);
		robot->moveWait(mask);
	}
}
