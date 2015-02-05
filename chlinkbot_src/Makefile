BUILD_DIR=../build-linux
CFLAGS= -std=c++11
BOOST_LIBS=$(BOOST_ROOT)/stage/lib

target: liblinkbot.dl

LINKLIBS="$(BUILD_DIR)/liblinkbot.a -lbaromesh -pthread -lboost_thread \
-lboost_filesystem \
-lboost_system \
-lboost_log \
-lsfp \
-lrpc \
-lrpc-proto \
-lrobot-interface \
-ldaemon-interface \
-ldongle-interface \
-lcommontypes-proto \
-lbaromesh-common \
-lcxx-util"

LINKDIRS="-L$(BOOST_LIBS) \
-L$(BUILD_DIR)/baromesh/library \
-L$(BUILD_DIR)/baromesh/library/libsfp \
-L$(BUILD_DIR)/baromesh/library/ribbon-bridge \
-L$(BUILD_DIR)/baromesh/library/baromesh/interfaces \
-L$(BUILD_DIR)/baromesh/library/baromesh/common \
-L$(BUILD_DIR)/baromesh/library/cxx-util"

liblinkbot.dl: chlinkboti_chdl.o chlinkbotl_chdl.o
	ch dllink liblinkbot.dl cplusplus chlinkboti_chdl.o chlinkbotl_chdl.o $(LINKDIRS) $(LINKLIBS)

chlinkboti_chdl.o: chlinkboti_chdl.cpp
	ch dlcomp liblinkbot.dl cplusplus chlinkboti_chdl.cpp -I/../include/barobo/linkbot.hpp

chlinkbotl_chdl.o: chlinkbotl_chdl.cpp
	ch dlcomp liblinkbot.dl cplusplus chlinkbotl_chdl.cpp -I/../include/barobo/linkbot.hpp


clean:
	rm -f *.o *.dl

install:
	sh ./createpkg.sh
	ch pkginstall.ch chbarobo-0.2
