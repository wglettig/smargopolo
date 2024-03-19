CC=g++
CFLAGS=-I. -std=c++11 -g
FILES = MCS.cpp
OUT_EXE = MCS
 
build: $(FILES)
	$(CC) -o $(OUT_EXE) $(FILES) $(CFLAGS)
 
clean:
	rm -f *.o core
 
rebuild: clean build
