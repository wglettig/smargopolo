ifndef DESTDIR
	DESTDIR = /usr/local/Orchestra/Core
endif

ifndef RTAI_INSTALL_DIR
	RTAI_INSTALL_DIR = /usr/realtime
endif

ifndef LINUX_SRC_DIR
	LINUX_SRC_DIR = /usr/src/linux
endif

ifndef CXX
	CXX = g++
endif

ifndef CXXFLAGS
	CXXFLAGS = -O2 -Wall
endif

default: libprigo

libprigo: prigo.cpp MatriceVectLib.cpp
# Creates the prigo library libprigo.a, which can be linked with via -lprigo
# NOTE: this library is optimized -O3
	$(CXX) -O3 -Wall -c -fPIC prigo.cpp  -I/usr/include 
	$(CXX) -O3 -Wall -c -fPIC MatriceVectLib.cpp 
	ar cr libprigo.a prigo.o MatriceVectLib.o

libprigodebug: prigo.cpp MatriceVectLib.cpp
# Creates the prigo library libprigo.a, which can be linked with via -lprigod
# NOTE: Use this make if you want to do debugging (-g)
	$(CXX) -g -Wall -c -fPIC prigo.cpp  -I/usr/include 
	$(CXX) -g -Wall -c -fPIC MatriceVectLib.cpp 
	ar cr libprigod.a prigo.o MatriceVectLib.o

test: libprigodebug
	$(CXX) -g -Wall -c -fPIC test.cpp
	$(CXX) test.o  -L. -lprigod -o test
#	$(CXX) test.cpp -Wall -L. -lprigo -lmeschach -o test


#	$(CXX) test.o prigo.o MatriceVectLib.o -lmeschach -o test
#	ar rcs libprigo.a prigo.o MatriceVectLib.o
with_s626:
#	$(CXX) ExternalTask.cpp -o ExternalTask -I$(ORCHESTRACORE_SRC_DIR)/include \
		-I$(LINUX_SRC_DIR)/include -I$(RTAI_INSTALL_DIR)/include $(CXXFLAGS) $(RTAI_INSTALL_DIR)/lib/liblxrt.a \
		-L$(ORCHESTRACORE_SRC_DIR)/../lib -lCoreUtils -lpthread

#	makeDLC PID.cpp --runtime-checks
#	makeDLC TrajectoryGenerator.cpp --runtime-checks0
#	makeDLC s626enc.cpp --includes="-I$(RTAI_INSTALL_DIR)/include -I$(DESTDIR)/include -I." --libs="$(RTAI_INSTALL_DIR)/lib/libkcomedilxrt.a" --runtime-checks
#	makeDLC s626ao.cpp  --includes="-I$(RTAI_INSTALL_DIR)/include -I$(DESTDIR)/include -I." --libs="$(RTAI_INSTALL_DIR)/lib/libkcomedilxrt.a" --runtime-checks

clean:
#	rm -f ExternalTask
#	rm -f *.dlc
	rm -f *.o
	rm -f libprigo.a
	rm -f libprigod.a
	rm -f test	
