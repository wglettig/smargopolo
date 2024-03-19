ifndef CXX
	CXX = g++
endif

ifndef CXXFLAGS
	CXXFLAGS = -O2 -Wall
endif

default: libsmargon

libsmargon: smargon.cpp MatriceVectLib.cpp
# Creates the smargon library libsmargon.a, which can be linked with via -lsmargon
# NOTE: this library is optimized -O3
	$(CXX) -O3 -Wall -c -fPIC smargon.cpp  -I/usr/include 
	$(CXX) -O3 -Wall -c -fPIC MatriceVectLib.cpp 
	ar cr libsmargon.a smargon.o MatriceVectLib.o

libsmargondebug: smargon.cpp MatriceVectLib.cpp
# Creates the smargon library libsmargon.a, which can be linked with via -lsmargond
# NOTE: Use this make if you want to do debugging (-g)
	$(CXX) -g -Wall -c -fPIC smargon.cpp  -I/usr/include 
	$(CXX) -g -Wall -c -fPIC MatriceVectLib.cpp 
	ar cr libsmargond.a smargon.o MatriceVectLib.o

test: libsmargon
	$(CXX) -g -Wall -c -fPIC test.cpp
	$(CXX) test.o  -L. -lsmargon -o test

#	$(CXX) test.cpp -Wall -L. -lprigo -lmeschach -o test
#	$(CXX) test.o prigo.o MatriceVectLib.o -lmeschach -o test
#	ar rcs libprigo.a prigo.o MatriceVectLib.o

clean:
	rm -f *.o
	rm -f libsmargon.a
	rm -f libsmargond.a
	rm -f test	
