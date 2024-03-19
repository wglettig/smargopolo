all: diag move configure onoff

diag: MCS2_USB-diag.c 
	gcc -o diag MCS2_USB-diag.c -L/usr/lib -lsmaractctl -I/usr/include

move: MCS2_USB-move.c 
	gcc -o move MCS2_USB-move.c -L/usr/lib -lsmaractctl -I/usr/include

configure: MCS2_USB-configure.c 
	gcc -o configure MCS2_USB-configure.c -L/usr/lib -lsmaractctl -I/usr/include

onoff100x: onoff100x.cpp
	gcc -o onoff100x onoff100x.cpp -L/usr/lib -lsmaractctl -I/usr/include

.PHONY: clean
clean:
	rm -f *.o  diag move configure onoff100x

