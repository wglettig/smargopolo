#include <stdio.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>

#define O )*2+1
#define _ )*2
#define s ((((((((0


class LCDClass {
	char str1[256];
	char str2[256];
	int serialport; 	//serial port no:  0:/dev/ttyS0, 1:/dev/ttyS1
	int fd; 		//file descriptor for Serial port
        int togglestate = 0;
	public:
	int open_port(int serialportno);
	int close_port();
	int cls();
	int drawbox();
	int printOrchestraStarted ();
        int toggle();
	int blink ();
	int drawLogo(int x, int y);
	int line1(const char * msg);
	int line2(const char * msg);
	int line3(const char * msg);

};
/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */

int LCDClass::open_port(int serialportno) {
	if      (serialportno == 0) fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	else if (serialportno == 1) fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
	else if (serialportno == 2) fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
	else if (serialportno == 3) fd = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NDELAY);
	else if (serialportno == 4) fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	else if (serialportno == 5) fd = open("/dev/cu.usbserial", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
	 /*
	 * Could not open the port.
	 */

	 printf("open_port: Unable to open /dev/ttyS1 - %d ", serialportno);
      	}
     	else
	  fcntl(fd, F_SETFL, 0);
      	return (fd);
}

int LCDClass::close_port() {
    return close(fd);
}


int LCDClass::cls (){
	return write(fd, "DL", 2);
}

int LCDClass::drawbox (){
	int x=1, y=2, dx=10, dy=2, ax=12,n; //Box dimensions
	char cmd[100];

	//First Draw Big Frame
	sprintf(cmd,"%c%c%c%c%c%c", 'O', 0,6,119,45,0);
	write(fd, cmd, 6);

	//Then draw the little boxes
 	sprintf(cmd,"%c%c%c%c%c%c", 'O', x,y,x+dx,y+dy,1);
 	n= write(fd, cmd, 6);
 	sprintf(cmd,"%c%c%c%c%c%c", 'O', x+ax,y,x+dx+ax,y+dy,1);
 	n= write(fd, cmd, 6);
 	sprintf(cmd,"%c%c%c%c%c%c", 'O', x+2*ax,y,x+dx+2*ax,y+dy,1);
 	n= write(fd, cmd, 6);
 	sprintf(cmd,"%c%c%c%c%c%c", 'O', x+3*ax,y,x+dx+3*ax,y+dy,0);
 	n= write(fd, cmd, 6);
 	sprintf(cmd,"%c%c%c%c%c%c", 'O', x+4*ax,y,x+dx+4*ax,y+dy,0);
 	n= write(fd, cmd, 6);
//	sprintf(cmd,"%c%c%c%c%c%c", 'O', x+5*ax,y,x+dx+5*ax,y+dy,0);
//	n= write(fd, cmd, 6);
//	sprintf(cmd,"%c%c%c%c%c%c", 'O', x+6*ax,y,x+dx+6*ax,y+dy,0);
//	n= write(fd, cmd, 6);
	return 1;
}
int LCDClass::drawLogo(int x, int y) {
char img[65] =
{
  'U',(char)x,(char)y, 59,7,
  s _ O O O O O O O,
  s _ O _ _ _ _ _ O,
  s _ O O O O O _ O,
  s _ O _ O _ O _ O,
  s _ O _ O _ O _ O,
  s _ O _ O O O _ O,
  s _ O _ _ _ _ _ O,
  s _ O _ O O O _ O,
  s _ O _ _ _ O _ O,
  s _ O _ _ _ O _ O,
  s _ O _ _ _ O _ O,
  s _ O _ _ _ _ _ O,
  s _ O _ O O O _ O,
  s _ O _ _ _ _ _ O,
  s _ O _ O O O _ O,
  s _ O _ O _ O _ O,
  s _ O O O _ O _ O,
  s _ O _ O O O _ O,
  s _ O _ _ _ _ _ O,
  s _ O _ O O O _ O,
  s _ O _ O _ O _ O,
  s _ O _ O _ O _ O,
  s _ O _ O O O _ O,
  s _ O _ _ _ _ _ _,
  s _ O _ O O O O O,
  s _ O _ O _ _ _ O,
  s _ O _ O _ _ _ O,
  s _ O _ O _ _ _ O,
  s _ O _ _ _ _ _ _,
  s _ O _ O O O O O,
  s _ O _ O _ _ _ O,
  s _ O _ O _ _ _ O,
  s _ O _ O O O O O,
  s _ O _ _ _ _ _ _,
  s _ O _ O O O O O,
  s _ O _ _ _ _ _ O,
  s _ O _ _ _ _ _ O,
  s _ O _ O O O O O,
  s _ O _ _ _ _ _ _,
  s _ O _ _ _ _ _ O,
  s _ O _ O O O O O,
  s _ O _ _ _ _ _ O,
  s _ O _ _ _ _ _ _,
  s _ O _ O O O O O,
  s _ O _ _ _ O _ O,
  s _ O _ _ _ O O O,
  s _ O _ O O O _ _,
  s _ O _ _ _ _ _ _,
  s _ O _ O O O O O,
  s _ O _ O _ _ _ O,
  s _ O _ O _ _ _ O,
  s _ O _ O O O O O,
  s _ O _ _ _ _ _ _,
  s _ O _ O O O O O,
  s _ O _ O _ _ _ _,
  s _ O _ O _ _ _ O,
  s _ O _ O _ _ _ O,
  s _ O _ _ _ _ _ O,
  s _ O O O O O O O,
  0
};
return write(fd, img, 65);
}

int LCDClass::printOrchestraStarted (){
	char cmd[100];
	//set font
	sprintf(cmd,"%c%c%c%c%c", 'F', 2,1,1,0);
	write(fd, cmd, strlen(cmd));
	//Print Orchestra Started
	sprintf(cmd,"%c%c%c%s%c", 90, 5,9,"Linux Booted.",0);
	write(fd, cmd, strlen(cmd));
	//Print Orchestra Started
	sprintf(cmd,"%c%c%c%s%c", 90, 5,20,"Orchestra Running.",0);
	write(fd, cmd, strlen(cmd));
	return 0;
}
	
int LCDClass::line1 (const char * msg){
	char cmd[256];
	//set font
	sprintf(cmd,"%c%c%c%c", 'F', 2,1,1);
	write(fd, cmd, 4);
	//Print Line 1
	sprintf(cmd,"%c%c%c%s", 'Z', 6,8,msg);
	write(fd, cmd, strlen(cmd)+1);
	return 0;
}
int LCDClass::line2 (const char * msg){
	char cmd[256];
	//set font
	sprintf(cmd,"%c%c%c%c", 'F', 2,1,1);
	write(fd, cmd, 4);
	//Print Line 2
	sprintf(cmd,"%c%c%c%s%c", 'Z', 6,16,msg,0);
	write(fd, cmd, strlen(cmd)+1);
	return 0;
}
int LCDClass::line3 (const char * msg){
	char cmd[256];
	//set font
	sprintf(cmd,"%c%c%c%c", 'F', 1,1,1);
	write(fd, cmd, 4);
	//Print Line 2
	sprintf(cmd,"%c%c%c%s", 'Z', 6,25,msg);
	write(fd, cmd, strlen(cmd)+1);
	return 0;
}
int LCDClass::toggle(){
    char cmd[256];
    if (togglestate) {
        sprintf(cmd,"%c%c%c%c%c%c\n", 'O', 25,2,35,4,1);
        write(fd, cmd, 7);
    } else {
        sprintf(cmd,"%c%c%c%c%c%c\n", 'O', 25,2,35,4,0);
        write(fd, cmd, 7);
    }

    togglestate =!togglestate;
    return 0;
}
int LCDClass::blink () {
//watch out this is an endless loop:
	clock_t start, end;
	double elapsed;
	char cmd[100];
	int n, i;
	for (i=0;i<10;i++){
		elapsed=0;
		start = clock();
		while (elapsed < 0.5){
		end=clock();
		elapsed = ((double)(end-start))/CLOCKS_PER_SEC;
		}
//		sprintf(cmd,"%c%c%c%c%c%c", 'O', 25,2,35,4,1);
//		n= write(fd, cmd, 6);
		toggle();
		
		elapsed=0;
		start = clock();
		while (elapsed < 0.5){
		end=clock();
		elapsed = ((double)(end-start))/CLOCKS_PER_SEC;
		}
//		sprintf(cmd,"%c%c%c%c%c%c", 'O', 25,2,35,4,0);
//		n= write(fd, cmd, 6);
		toggle();
	}
	return 0;
}


