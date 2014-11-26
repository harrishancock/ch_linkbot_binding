CFLAGS= -std=c++11

target: liblinkbot.dl

liblinkbot.dl: linkbot_wrapper.o chlinkboti_chdl.o
	ch dllink liblinkbot.dl cplusplus linkbot_wrapper.o chlinkboti_chdl.o -lboost_thread -lboost_filesystem -lboost_system -lboost_log ../build-linux/liblinkbot.a 

chlinkboti_chdl.o: chlinkboti_chdl.cpp
	ch dlcomp liblinkbot.dl cplusplus chlinkboti_chdl.cpp -I/../include/barobo/linkbot.hpp

linkbot_wrapper.o: ../src/linkbot_wrapper.cpp
	ch dlcomp liblinkbot.dl cplusplus $(CFLAGS) ../src/linkbot_wrapper.cpp -I/../include/barobo/linkbot.hpp  

clean:
	rm -f *.o *.dl
