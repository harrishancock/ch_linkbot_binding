CFLAGS= -std=c++11
BOOST_LIBS=$(BOOST_ROOT)/stage/lib

target: liblinkbot.dl

LINKLIBS="../build-linux/liblinkbot.a -lbaromesh -pthread -lboost_thread \
-lboost_filesystem \
-lboost_system \
-lboost_log \
-lboost_coroutine \
-lboost_context \
-lsfp \
-lrpc \
-lrpc-proto \
-lrobot-interface \
-ldaemon-interface \
-ldongle-interface \
-lcommontypes-proto"

LINKDIRS="-L$(BOOST_LIBS) \
-L../build-linux/baromesh \
-L../build-linux/baromesh/libsfp \
-L../build-linux/baromesh/ribbon-bridge \
-L../build-linux/baromesh/ribbon-bridge-interfaces"

liblinkbot.dl: chlinkboti_chdl.o
	ch dllink liblinkbot.dl cplusplus chlinkboti_chdl.o $(LINKDIRS) $(LINKLIBS)

chlinkboti_chdl.o: chlinkboti_chdl.cpp
	ch dlcomp liblinkbot.dl cplusplus chlinkboti_chdl.cpp -I/../include/barobo/linkbot.hpp

clean:
	rm -f *.o *.dl

install:
	sh ./createpkg.sh
	ch pkginstall.ch chbarobo-0.2
