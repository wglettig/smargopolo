CC=g++
CFLAGS=-I. -std=c++11 -g
FILES = ascii.cpp
OUT_EXE = Test
 
build: $(FILES)
	$(CC) -o $(OUT_EXE) $(FILES) $(CFLAGS)
 
clean:
	rm -f *.o Test
 
rebuild: clean build
