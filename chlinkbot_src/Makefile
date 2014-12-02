CFLAGS= -std=c++11
BOOST_LIBS=$(BOOST_ROOT)/stage/lib

target: liblinkbot.dl

LINKLIBS="../build-linux/liblinkbot.a -lbaromesh -pthread -lboost_thread \
-lboost_filesystem \
-lboost_system \
-lboost_log \
-lboost_coroutine \
-lboost_context"

LINKDIRS="-L$(BOOST_LIBS) -L../build-linux/baromesh"

liblinkbot.dl: chlinkboti_chdl.o
	ch dllink liblinkbot.dl cplusplus chlinkboti_chdl.o $(LINKDIRS) $(LINKLIBS)

chlinkboti_chdl.o: chlinkboti_chdl.cpp
	ch dlcomp liblinkbot.dl cplusplus chlinkboti_chdl.cpp -I/../include/barobo/linkbot.hpp

clean:
	rm -f *.o *.dl
