CC=g++
CFLAGS=-I. -std=c++11 -g
FILES = rtClock.cpp
OUT_EXE = rtClock
 
build: $(FILES)
	$(CC) -o $(OUT_EXE) $(FILES) $(CFLAGS)
 
clean:
	rm -f *.o core
 
rebuild: clean build
