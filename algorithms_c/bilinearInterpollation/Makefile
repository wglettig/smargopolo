CC=g++
CFLAGS=-I. -std=c++11 -g
FILES = interp.cpp
OUT_EXE = Interp
 
build: $(FILES)
	$(CC) -o $(OUT_EXE) $(FILES) $(CFLAGS)
 
clean:
	rm -f *.o Interp
 
rebuild: clean build
