#include <iostream>
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

LinkbotGroup::LinkbotGroup()
{
	_numRobots = 0;
  _motionInProgress = 0;
  //_thread = (THREAD_T*)malloc(sizeof(THREAD_T));
  _numAllocated = 0;
  _robots = NULL;
  _ID = NULL;
}

LinkbotGroup::~LinkbotGroup()
{
}

void LinkbotGroup::addRobot(char* serialID)
{
	int idLength = 5; /*number of letters in the robot ID + \0 */
	int allocStep=64;
	/* See if we need to allocate more robots. The space allocated in reality is more than the actual number of robots */
	if(_numRobots >= _numAllocated){
		char** tmpid; /*store all the robot IDs*/
		Linkbot** tmp; /*store the objects of class robot*/

		tmpid = (char**)malloc(sizeof(char*)*(_numAllocated + allocStep)); /*allocate more memory for IDs*/
		tmp = (Linkbot**)malloc(sizeof(Linkbot*)*(_numAllocated + allocStep)); /*allocate more memory for robots*/
		if (_ID != NULL) { /*already some robots*/
			memcpy(tmp, _ID, sizeof(char*)*_numRobots);
			free(_ID);
		}
		if (_robots != NULL) {
			memcpy(tmp, _robots, sizeof(Linkbot*)*_numRobots);
			free(_robots);
		}
		_ID = tmpid; /*copy over the IDs*/
		_robots = tmp; /*copy over the objects*/
		_numAllocated += allocStep;
	}
	_ID[_numRobots] = serialID;
	_numRobots++;
}

void LinkbotGroup::connect()
{
	int type;
	for(int i = 0; i < _numRobots; i++) {
    _robots[i]->connect(_ID[_numRobots], &type);
  }
}
