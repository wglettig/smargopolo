/***********************************************************************
 *
 * File name: SmarActMCS_lib.h
 * Author   : Wayne Glettig, Marco Salathe
 * Version  : Dez. 2011
 *
 * Libray containing Methods to communicate with SmarActs Motor 
 * Control System (MCS)
 *
 ***********************************************************************
 * Description of Error Nos:
 *
 * Most functions return an int, which corresponds to an error message.
 *    0: OK! (no errors, operation successful)
 *  Scan Port Errors
 *    2: After automatic scan, no SmarAct MCS module could be found.
 *  Serial Connection issues:
 *    10: /dev/ttyS0 cannot be opened
 *    11: /dev/ttyS1 cannot be opened
 *    12: /dev/ttyS2 cannot be opened
 *    13: /dev/ttyS3 cannot be opened
 *    14: Invalid TTY number specified. (only 0-3 are supported)
 *  Baud rate setting
 *    15: Invalid Baud Rate (see table for valid entries)
 *  Closing Serial Port errors:
 *    16: EBADF fd isn't a valid open file descriptor
 *    17: EINTR
 *    18: EIO
 *    19: Other close error.
 *  Writing to Serial Port errors:
 *    20: EAGAIN
 *    21: EBADF fd is not open for writing (->try opening serial port)
 *    22: EFAULT 
 *    23: EFBIG 
 *    24: EINTR 
 *    25: EINVAL
 *    26: EIO
 *    27: ENOSPC
 *    28: EPIPE
 *    29: Other write error.
 *  Reading from Serial Port errors:
 *    30: EAGAIN nothing in input buffer (->MCS might be unplugged or on 
 *        another serial port)
 *    31: EBADF fd is not open for reading (->try opening serial port 
 *        read mode)
 *    32: EFAULT
 *    33: EINTR
 *    34: EINVAL
 *    35: EIO
 *    36: EISDIR
 *    37: Other read error.
 *  Problems with reading Status back: (in readGS())
 *    41: parseInput(): Reading from serial port worked, but could not 
 *        detect command beginning ':'. 
 *    42: read.. in: channelIndex invalid (must be 0-5)
 *
 **********************************************************************/

//#define VERBOSE
//#define REALTIME  //use this if rtai serial port access is required

// Includes:
#include <cstdio>        /* Standard input/output definitions */
#include <cmath>

#ifndef REALTIME
#include <cstdlib>       /* Standard input/output definitions */
#include <string.h>      /* String function definitions */
#include <unistd.h>      /* UNIX standard function definitions */
#include <fcntl.h>       /* File control definitions */
#include <errno.h>       /* Error number definitions */
#include <termios.h>     /* POSIX terminal control definitions */
#endif

#ifdef REALTIME
//realtime libraries are build in C, it doesn't understand C++ (badly build) hence this extern is important
extern "C" {
#include <rtai_serial.h> /*RTAI Serial Port Access*/
#include <rtai_lxrt.h>   /*RTAI Linux Realtime*/
}
#endif

#define BUFFER_SIZE 2048

using namespace std;

// Definition of structure "state" used in SmaractMCS
struct state {
	double lastupdated;
	double value;
};

/***********************************************************************
 * SmaractMCS class definition:
 **********************************************************************/
class SmarActMCS  {

private:
	// Serial Connection Parameters:
	int fd;			              // file descriptor
	unsigned int TTY;	          // Serial Port No.
	unsigned int baudrate;	      // Baud Rate (speed_t is unsigned int)
	char readBuffer[BUFFER_SIZE]; // Buffer for serial in reading.
	unsigned int insBuf;          // current insert position for new entries into Buffer
	                              // if insBuf>=BUFFER_SIZE
	
	// MCS State Variables:
	double currentTime;           // To be used with state.lastupdated
	int  MCS_IV[3];               // Interface Version: High, low, Build
	char MCS_ID [50];             // System ID String
	int  MCS_NC;                  // Number of Channels

	// Axis state variables that are updated in parseInput()
	state statePPK[6];            // Physical Position known Status ("initialized?" Status)
	state stateP[6];              // Position State of Axes
	state stateS[6];              // State of Axes
	state lastError[6];           // Last Error no.

public:
	// Input handling and Synchronisation
	int setTimestamp(double timestamp);    // Set the timestamp
	int parseInput();                      // empty input buffer and process replies (using currentTime variable)
	int parseInput(double timestamp);      // empty input buffer and process replies
	int parseMCSReplyStr(char* str, double timestamp);

	SmarActMCS();  //Constructor
	~SmarActMCS(); //Destructor

	// Serial Port Commands
	int initialize(int port, int baud);	   // Opens and configures serial port
	int initialize();                      // Opens and configures serial port (using default values)
	int openSerialPort();                  // Opens Serial Port Connection
	int openSerialPort(int tty);           // Opens Serial Port Connection (ttyS0, 1, 2, 3)
	int openSerialPort(int tty, int baud); // Opens Serial Port Connection (with tty & baudrate)
	int setBaudRate(int baud);             // Set baud rate variable.
	int closeSerialPort();                 // Close Serial Port
	int clearBuf();                        // Clear Buffer
	int sendMCS (char* cmd);               // Send Command to MCS
	#ifdef REALTIME
	int readMCS ();                        // Read Result from MCS -> fill up readBuffer[]
	#endif
	#ifndef REALTIME
	int readMCS(char* buf, int* len);
	#endif

	// SmarAct MCS Commands: (See MCS RS232 Interface Documentation for details)
	int FRM(int channelIndex, int direction, int holdTime, int autoZero); // Find Reference Marks
	int sendGS(int channelIndex);                  // send Get Status Command
 	int readGS(int channelIndex,double* status);   // read Get Status Command reply
	int readGS(int channelIndex,double* status, double timestamp, double* delay);
	int sendGP(int channelIndex);                  // send Get Position Command
	int readGP(int channelIndex,double* status);   // read Get Position Command reply
	int readGP(int channelIndex,double* status, double timestamp, double* delay);
	int sendGA(int channelIndex);                  // send Get Angle Command
	int readGA(int channelIndex,double* status);   // read Get Angle Command reply
	int readGA(int channelIndex,double* status, double timestamp, double* delay);
	int sendGPPK(int channelIndex);			       // send Get Physical Position known?
	int readGPPK(int channelIndex,double* status); // read Get Physical Position known? reply
	int readGPPK(int channelIndex,double* status, double timestamp, double* delay);
	int sendGSI();                                 // send Get System ID
	int readGSI(char* ID);                         // read Get System ID reply
	int sendGNC();                                 // send Get Number of channels
	int readGNC(char* ID);                         // read Get Number of channels reply
	int SCLF(int channelIndex, int frequency);     // Set Closed Loop Max Frequency (50-10'000)
	int SCLS(int channelIndex, int speed);         // Set Closed Loop Speed
	int MPA(int channelIndex, int position, int holdTime); // Move to Position Absolute
	int MAA(int channelIndex, int position, int holdTime); // Move to Angle Absolute
	int MPR(int channelIndex, int position, int holdTime);
	int MAR(int channelIndex, int position, int holdTime);
	int SCM(int mode);                             // Set Communication Mode (0: Sync, 1:Async (no reply))
	int SHE(int mode);                             // Set hand control module enable
	int S();                                       // Stop All Channels
	int S(int channelIndex);                       // Stop Channel
	int SP(int channelIndex, int position);        // Set Position if channel
	int R();                                       // Reset
	int CB(int baudrate);                          // set Baudrate
	int CS(int channelIndex);                      // Calibrate sensor
};


/***********************************************************************
 * Definition of methods:
 **********************************************************************/
SmarActMCS::SmarActMCS()
{	
	strcpy(MCS_ID, "Not Set");
	fd=0;
	memset(readBuffer,NULL, BUFFER_SIZE); // initialize read string to NUL NUL NUL NUL
	insBuf=0;
}

SmarActMCS::~SmarActMCS() {
	closeSerialPort();
}

int SmarActMCS::initialize()
{
/*	//AUTOMATIC INITIALISATION (Find port & baud rate automatically)
	//List possible ports & baudrates: 	
	int ports [] = {0,1};
	//int bauds [] = {50,300,1200,2400,4800,9600,19200,38400,57600,115200};//aufsteigend
	//int bauds [] = {115200,57600,38400,19200,9600,4800,2400,1200,300,50};//absteigend
	int bauds [] = {115200,57600,38400,19200,9600};//absteigend
	//calculate lengths of above arrays.
	int num_ports = sizeof(ports)/sizeof(int);
	int num_bauds = sizeof(bauds)/sizeof(int);
	//set flags to -1: nothing found.
	int found_port = -1;
	int found_baud = -1;
	//scan ports & baudrates.
	for (int i=0; i<num_ports; i++) {
		for (int j=0; j<num_bauds; j++) {
			if (!openSerialPort(ports[i], bauds[j])){
				usleep(100000);
				if (!sendGSI()) {
					usleep(200000);
					char ID[254];
					if (!readGSI(ID)){
						found_port=ports[i];
						found_baud=bauds[j];
						//stop search:
						i=num_ports;
						j=num_bauds;
						printf("=====ID: |%s|\n",ID);
						
					}
				}
			}
			closeSerialPort();
			printf("port:%d, baud:%d\n",ports[i], bauds[j]);
		}
	} 
	printf ("\n=====AUTO INITIALIZSATION: found_port: %d, found_baud: %d\n\n",found_port,found_baud);
	//If no SmarAct MCS could be found:
	if (found_port==-1 && found_baud==-1) {
		//set back to standard TTY & baudrate
		TTY = 0;
		setBaudRate(9600);
		return 2;
	}
*/	//If MCS found:
	return 0;
//	return initialize(0,9600);
}

int SmarActMCS::initialize(int port, int baud)
{
	//INITIALISATION:
	//  1.Configure serial port and try to open it.
	//  2.Configure MCS and set it up for standard use
	int err;
	if ((err= openSerialPort(port, baud) )) return err;
	usleep(1000);
	if ((err= R() )) return err;     // Reset
	usleep(1000);
	if ((err= SCM(1) )) return err;  // 1:Asynchronus communication.(no answer)
	usleep(1000);

	printf("Initialisation done\n");
	return 0;
}

int SmarActMCS::openSerialPort(int tty) {
	// Set Class Variable TTY
	TTY = tty;
	return openSerialPort();
}

int SmarActMCS::openSerialPort(int tty, int baud) {
	// Set Class Variable TTY
	TTY = tty;
	// Set Class Variable baudrate
	int err = setBaudRate(baud);
	if (err) return err;
	// Run Class Method openSerialPort()
	return openSerialPort();
}

#ifndef REALTIME
int SmarActMCS::openSerialPort() {
	// OPEN PORT (depending on current TTY variable):
	switch (TTY) {
		case 0:  fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY); break;
		case 1:  fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY); break;
		case 2:  fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY); break;
		case 3:  fd = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NDELAY); break;
		default: printf("SmarActMCS::openSerialPort(): ERROR: Invalid TTY No.%d!\n", TTY); 
			 return 14;
	}

	// if port didn't open give feedback and return.
	if (fd == -1) {
		printf("SmarActMCS::openSerialPort(): ERROR: Unable to open /dev/ttyS%d- ",TTY );
		int err = 10 + TTY;
		return err;
	}
	printf("Serial port /dev/ttyS%d opened. (fd=%d)\n",TTY, fd );

	// CONFIGURE PORT:
	tcflush(fd, TCIOFLUSH);      // Flush Buffers
	fcntl(fd, F_SETFL, FNDELAY); // During reading, don't block serial read (FNDELAY)
	//fcntl(fd, F_SETFL, 0);       // 0 blocks the system while reading serial port	
	struct termios options;
	// Get the current options for the port...
	tcgetattr(fd, &options);
	// Set the baud rates...
	cfsetispeed(&options, baudrate);
	cfsetospeed(&options, baudrate);
	// Enable the receiver and set local mode...
	options.c_cflag |= (CLOCAL | CREAD); // set local mode & enable the receiver 
	options.c_cflag &= ~PARENB;	         // 8N1:no parity
	options.c_cflag &= ~CSTOPB;	         // 8N1:1 stopbit
	options.c_cflag &= ~CSIZE;	         // 8N1:bit mask for data bits
	options.c_cflag |= CS8;	             // 8N1:8 Data bits
	options.c_cflag &= ~CRTSCTS;         // disable flow control
	// ----oflags:-----
	options.c_oflag = 0;          //raw output
	//options.c_oflag &= ~ONLCR;    //do not replace (lf) with (cr)(lf) (important for SmarAct!)
	//options.c_oflag |= OCRNL;     //ENABLE CR to NL
	//options.c_oflag &= ~OCRNL;    //DISABLE CR to NL
	//options.c_oflag |= OLCUC;     //ENABLE  Map lowercase to uppercase
	//options.c_oflag &= ~OLCUC;    //DISABLE Map lowercase to uppercase
	//----iflags:-----
	//options.c_iflag |= INLCR;   //replace (lf) with (cr)(lf)
	options.c_iflag &= ~INLCR;  //do not replace (lf) with (cr)(lf)
	//----lflags:-----
	options.c_lflag &= ~(ICANON| ECHO | ECHOE | ISIG); //Raw input
	options.c_lflag |= ICANON;                         //make input CANNONICAL (read by line)

	// Set the new options for the port...
	//tcsetattr(fd, TCSANOW, &options); //set options for the port now.
	tcsetattr(fd, TCSANOW, &options); //flush input and set options for the port
	printf("Serial port /dev/ttyS%d configured.\n",TTY);

	usleep(1000);

	return 0;
}
#endif

#ifdef REALTIME
int SmarActMCS::openSerialPort() {
	// OPEN PORT (depending on current TTY variable):
	int res;
	switch (TTY) {
		case 0:  res = rt_spopen(COM1, baudrate, 8, 1, RT_SP_PARITY_NONE, RT_SP_NO_HAND_SHAKE, RT_SP_FIFO_DISABLE); break;
		case 1:  res = rt_spopen(COM2, baudrate, 8, 1, RT_SP_PARITY_NONE, RT_SP_NO_HAND_SHAKE, RT_SP_FIFO_DISABLE); break;
		default: printf("SmarActMCS::openSerialPort(): ERROR: Invalid TTY No.%d!\n", TTY); 
			 return 14;
	}

	// If there was an error, treat it.
	if (res) {
		printf("Error: rt_spopen\n");
		switch(res) {
			case -ENODEV:
			printf("No device %d\n", res);
			break;
			case -EINVAL:
			printf("Invalid val %d\n", res);
			break;
			case -EADDRINUSE:
			printf("Address in use %d\n", res);
			break;
			default:
			printf("Unknown %d\n", res);
			break;
		}
		return res;
	}
	printf("Serial port /dev/ttyS%d opened. (res=%d)\n",TTY, res );

	// Serial Port Clear
	switch (TTY) {
		case 0: rt_spclear_rx(COM1);rt_spclear_tx(COM1); break;
		case 1: rt_spclear_rx(COM2);rt_spclear_tx(COM2); break;
	}
	printf("Serial port /dev/ttyS%d cleared.\n",TTY);
	return 0;
}
#endif


int SmarActMCS::clearBuf() {
#ifndef REALTIME
	return 	tcflush(fd, TCIOFLUSH);  // Flush Buffers
#endif

#ifdef REALTIME
	memset(readBuffer,NULL, BUFFER_SIZE);  // initialize read string to NUL NUL NUL NUL
	insBuf=0;
	return 0;
#endif
}

#ifndef REALTIME
int SmarActMCS::setBaudRate(int baud) {
	switch (baud) {
		case 50:     baudrate = B50; break;
		case 300:    baudrate = B300; break;
		case 1200:   baudrate = B1200; break;
		case 2400:   baudrate = B2400; break;
		case 4800:   baudrate = B4800; break;
		case 9600:   baudrate = B9600; break;
		case 19200:  baudrate = B19200; break;
		case 38400:  baudrate = B38400; break;
		case 57600:  baudrate = B57600; break;
		case 115200: baudrate = B115200; break;
		default: printf("SmarActMCS::setBaudRate(): ERROR: Baudrate invalid: %d!\n", baud); 
		return 15;
	}
	return 0;
}
#endif

#ifdef REALTIME
int SmarActMCS::setBaudRate(int baud) {
	switch (baud) {
		case 50:     baudrate = 50; break;
		case 300:    baudrate = 300; break;
		case 1200:   baudrate = 1200; break;
		case 2400:   baudrate = 2400; break;
		case 4800:   baudrate = 4800; break;
		case 9600:   baudrate = 9600; break;
		case 19200:  baudrate = 19200; break;
		case 38400:  baudrate = 38400; break;
		case 57600:  baudrate = 57600; break;
		case 115200: baudrate = 115200; break;
		default: printf("SmarActMCS::setBaudRate(): ERROR: Baudrate invalid: %d!\n", baud); 
                         return 15;
	}
	return 0;
}
#endif

#ifndef REALTIME
int SmarActMCS::closeSerialPort() 
{
	int ret = close(fd);	
	if (ret == -1) {
		// Error handling (See description further up, after class definition):
		// printf("SmarActMCS::closeSerialPort(): ERROR\n%s\n", strerror(errno));
		if        (errno == EBADF) {
			  return 16;
		} else if (errno == EINTR) {
			  return 17;
		} else if (errno == EIO) {
			  return 18;
		} else {
			  return 19;
		}
	}
	return 0;	
}
#endif

#ifdef REALTIME
int SmarActMCS::closeSerialPort() 
{
	switch (TTY) {
		case 0: rt_spclose(COM1); break;
		case 1: rt_spclose(COM2); break;
		default: printf("SmarActMCS::closeSerialPort(): ERROR: Invalid TTY No.%d!\n", TTY); 
			 return 19;
	}
	return 0;	
}
#endif

#ifndef REALTIME
int SmarActMCS::sendMCS(char* cmd) 
{
	int len = strlen(cmd);
	cmd[len+1] = 0x00; // terminate the string properly
	char full_cmd [254];
	sprintf(full_cmd, ":%s%c", cmd,10); // add ':' before and '\n' after cmd (SmarAct Command Syntax)
	//fcntl(fd, F_SETFL, 0);	// 0 blocks the system while reading serial port
	int ret = write(fd, full_cmd, strlen(full_cmd));
	//fcntl(fd, F_SETFL, FNDELAY);	// During reading, don't block serial read (FNDELAY)
	if (ret < 0) {
		// Error handling (See description further up, after class definition):
		// printf("SmarActMCS::writeMCS(): ERROR: Write failed!\n%s\n", strerror(errno));
		if        (errno == EAGAIN) {
			  return 20;
		} else if (errno == EBADF) {
			  return 21;
		} else if (errno == EFAULT) {
			  return 22;
		} else if (errno == EFBIG) {
			  return 23;
		} else if (errno == EINTR) {
			  return 24;
		} else if (errno == EINVAL) {
			  return 25;
		} else if (errno == EIO) {
			  return 26;
		} else if (errno == ENOSPC) {
			  return 27;
		} else if (errno == EPIPE) {
			  return 28;
		} else {
			  return 29;
		}
	}
#ifdef VERBOSE 
printf ("WROTE: '%s'\n", full_cmd);
#endif
	return 0;
}
#endif

#ifdef REALTIME
int SmarActMCS::sendMCS(char* cmd) 
{
	switch (TTY) {
		case 0: fd = COM1; break;
		case 1: fd = COM2; break;
		default: printf("SmarActMCS::sendMCS(): ERROR: Invalid TTY No.%d!\n", TTY); 
			 return 29;
	}

	int len = strlen(cmd);
	cmd[len+1] = 0x00; // terminate the string properly
	char full_cmd [254];
	sprintf(full_cmd, ":%s%c", cmd,10); // add ':' before and '\n' after cmd (SmarAct Command Syntax)
	//fcntl(fd, F_SETFL, 0);	// 0 blocks the system while reading serial port	
	int ret = rt_spwrite(fd, full_cmd, strlen(full_cmd));
	//fcntl(fd, F_SETFL, FNDELAY);	// During reading, don't block serial read (FNDELAY)
	if (ret < 0) {
		// Error handling (See description further up, after class definition):
		// printf("SmarActMCS::writeMCS(): ERROR: Write failed!\n%s\n", strerror(errno));
		if        (errno == EAGAIN) {
			  return 20;
		} else if (errno == EBADF) {
			  return 21;
		} else if (errno == EFAULT) {
			  return 22;
		} else if (errno == EFBIG) {
			  return 23;
		} else if (errno == EINTR) {
			  return 24;
		} else if (errno == EINVAL) {
			  return 25;
		} else if (errno == EIO) {
			  return 26;
		} else if (errno == ENOSPC) {
			  return 27;
		} else if (errno == EPIPE) {
			  return 28;
		} else {
			  return 29;
		}
	}
#ifdef VERBOSE 
for (unsigned int i=0; i<strlen(full_cmd);i++) {
	printf ("WROTE %d: %d: %c\n", i, full_cmd[i], full_cmd[i]);
}
#endif
	return 0;
}
#endif

#ifndef REALTIME
int SmarActMCS::readMCS(char* buf, int* len) 
{
	*len = read(fd, buf, 254);
	buf[*len-1] = 0x00; // terminate the string properly
	if (*len < 0) {
		// Error handling (See description further up, after class definition):
		// printf("SmarActMCS::readMCS(): ERROR: Read failed!\n%s\n", strerror(errno));
		if        (errno == EAGAIN) {
			  return 30;
		} else if (errno == EBADF) {
			  return 31;
		} else if (errno == EFAULT) {
			  return 32;
		} else if (errno == EINTR) {
			  return 33;
		} else if (errno == EINVAL) {
			  return 34;
		} else if (errno == EIO) {
			  return 35;
		} else if (errno == EISDIR) {
			  return 36;
		} else {
			  return 37;
		}
	} 
#ifdef VERBOSE 
	printf ("READ %d: '%s'\n", *len, buf);            
#endif
	return 0;
}
#endif

#ifdef REALTIME
int SmarActMCS::readMCS() 
{
	/* this function will read in all characters from the serial port, until no more can be read. Also, it will continue also
	 * when the readBuffer is full, rejecting the most recent data. 
	 * The function returns:
	 *   - 0 when all was ok.
	 *   - 37 when no characters were read
	 *   - 38 when buffer was full. (expect lost data)
	 */
	bool ok=true;
	bool bufferfull=false;
	char readchar; //local holder for 1 char
	unsigned int insBufonStart = insBuf;
	while (ok) {
		int ret;
		// Read in 1 character at a time. Timeout 1us.
		ret = rt_spread_timed(fd, &readchar, 1, nano2count(1000));
		// if a successful character was read, ret will be 0.
		if (ret == 0) {
			if (insBuf<BUFFER_SIZE-1) { // make sure we don't overload the buffer. (-1 because last element of buffer must be String terminator NULL)
				// if input successfully read, write character to global readBuffer and increment readchar
				readBuffer[insBuf] = readchar;
				insBuf++;
				readBuffer[insBuf] = NULL; // null terminate in case this is the end of the string. this is overwritten if read continues.
#ifdef VERBOSE 
printf ("READ: insBuf=%d: ASCII:%d '%c'\n", insBuf, readchar, readchar);
#endif
			} else {
				bufferfull= true;
				printf ("READ: Error: Buffer Full, lost data. Try increasing BUFFER_SIZE, or running parseInput() more frequently.\n");
			}
		} else {
		// if no character could be read. Stop loop.
			ok=false;
		}
	}
	// generate return codes depending on what happened:
	if (bufferfull==true) return 38;        // Buffer full (expect lost data)
	int charsRead = (insBuf-insBufonStart); // Calculate number of chars read
        if (charsRead == 0) return 37;      // no characters were read (complies with use above (non realtime).
	else return 0;
}
#endif

int SmarActMCS::setTimestamp(double timestamp){
	currentTime = timestamp;
	return 0;
}
int SmarActMCS::parseInput(){
	return parseInput(currentTime);
}

#ifndef REALTIME
int SmarActMCS::parseInput(double timestamp){
	//Read from Serial Port
	int err, len, counter =0;
	char sInput[1000];
	bool done = false;
	while (!done) {
		err = readMCS(sInput,&len);
		if (err==0) {
			parseMCSReplyStr(sInput, timestamp);
			counter++;
		} else {done=true;}
		if (counter > 10) done=true;
#ifdef VERBOSE 
		printf("parseinput read: '%s'\n",sInput);
#endif
	}
#ifdef VERBOSE 
	printf("parseinput counter: '%d'\n",counter);
#endif

//	while (((err = readMCS(sInput,&len)) == 0)&&counter<10) {
//#ifdef VERBOSE 
//		printf("parseinput read: '%s'\n",sInput);
//#endif
//		parseMCSReplyStr(sInput, timestamp);
//		counter++;
//	}
	return counter;
}
#endif

	#ifdef REALTIME
int SmarActMCS::parseInput(double timestamp){
	// read all contents from RS232 Serial Port to readBuffer
	readMCS();

	/* Isolate the commands and pass to parseMCSReplyStr()
	 * What to do:
	 * - Find first occurence of ':'
	 * if no occurence found, empty buffer (make buffer[0] = NULL). return.
	 * if occurence found, ignore all before that ':'. (memmove(str, pStart, strlen(pStart));
	 * - Find first occurance of '\n';
	 *     if no occurence found, return
	 *     if occurence found, isolate command: (from pStart to pStop)=>char Command[]. 
	 * Then remove command from buffer (memmove(str, pStop, strlen(pStop));
	 * Then further treat command:
	 * go back to step 1.
	 */

	readBuffer[BUFFER_SIZE-1]=NULL; // make sure last element of readBuffer is NULL (string termination)
	// replace all NULL chars in buffer (from 0 to insBuf) with \n 
	// (in case a ASCII 0 was sent via RS232.)
	for (unsigned int i=0;i<insBuf;i++){
		if (readBuffer[i]==0) readBuffer[i]=0x0A;
	}
	char * pStart=NULL;
	char * pStop=NULL;
	bool done=false;
	while (done==false) {
		pStart = (char*)memchr(readBuffer,':',insBuf); // detect first occurence of ':' (start command).Returns a pointer
		if (pStart ==NULL) { // if no occurence found, empty buffer (make buffer[0] = NULL). return.
			readBuffer[0]=NULL;
			insBuf=0;
			done=true;
		} else { // if occurence found, ignore all before that ':'. (memmove(str, pStart, strlen(pStart));
			strcpy(readBuffer, pStart);
			insBuf=insBuf-(pStart-readBuffer);

			pStop = (char*)memchr(readBuffer,0x0A,BUFFER_SIZE); // detect first occurence of '\n' (end command).Returns a pointer
			if (pStop==NULL) { // if no occurence found, just return (further parsing next time the function parseInput is called)
				done=true;
			} else { // if occurence found, isolate command: (from pStart to pStop)=>char Command[]. 
				char command[BUFFER_SIZE];
				size_t length = pStop+1-pStart;
				memcpy(command, pStart, length);
        			command[length]=NULL; // terminate with NULL
				// Then remove command from buffer (memmove(str, pStop, strlen(pStop));
				strcpy(readBuffer, pStop+1);
				insBuf=insBuf-((pStop+1)-readBuffer);
				// Then further treat command:
#ifdef VERBOSE 
				printf("DETECTED COMMAND: '%s'\n",command);
				printf("insBuf:%d, Buf:%s\n",insBuf, readBuffer);
#endif
				// Send command to MCS Reply Parser.
				parseMCSReplyStr(command, timestamp);
			}
		}
	}
	return 0;
}
#endif

int SmarActMCS::parseMCSReplyStr(char* str, double timestamp) {
// Start Processing input string
	char *start = str;
	char sCommand [256]="";
	char sArg1 [256]="";
	char sArg2 [256]="";
	char sArg3 [256]="";
	int no_of_terms;

	// move start pointer to begging of command ':', if not found -> error
	if ((start = strchr(str,':'))==NULL) { return 41; }
	// Process command: Split up into Command, Arg1, Arg2, ....
	no_of_terms = sscanf(start,":%12[^1234567890-]%12[^,\n],%12[^,\n],%12[^,\n]", sCommand,sArg1,sArg2,sArg3);
#ifdef VERBOSE 
	printf("parsedInput: Number of terms: '%d'\n",no_of_terms);
	printf("parsedInput: String :'%s'\n",sCommand);
	printf("parsedInput: arg1   :'%s'\n",sArg1);
	printf("parsedInput: arg2   :'%s'\n",sArg2);
#endif

	/* React according to Command:
	 * E<sourceChannel>,<errorCode> 
	 * IV<versionHigh>,<versionLow>,<versionBuild> (reply of GIV Get Interface Version) 
	 * ID<id>  (reply of GSI, Get System ID) 
	 * NC<id>  (reply of GNC, Get Number of Channels) 
	 * P<channelIndex>,<position> (reply of GP, Get Position, position in nm) 
	 * A<channelIndex,<angle> (reply of GA, Get Angle, angle in micro degrees) 
	 * S<channelIndex,<status> (reply of GS, Get Status) 
	 * PPK<channelIndex>,<known> (reply of GPPK, Get Physical Position Known, 0:unknown, 1: known) 
	 *
	 * Not yet Supported:
	 * CLS<channelIndex><speed> (reply of GCLS, Get closed loop speed) 
	 * SE<mode> (reply of GSE, Get Sensor Enabled) 
	 * ST<channelIndex>,<type> (Reply of GST, Get Sensor Type) 
	 * VL<channelIndex>,<level> (reply of GVL, Get Voltage Level) 
	 * BR<baudrate> (reply of CB, Configure Baudrate) 
	 * ESM<mode> (reply of GESM, get Emergency Stop Mode) 
	 *
	 * IGNORE:
	 * E-1,0
	 * E Channelindex, 0
	 */

	// E<sourceChannel>,<errorCode> :
	if (strcmp(sCommand,"E")==0) {
		int channel = atoi(sArg1);
		double errorcode = atof(sArg2);
		if (channel == -1) {
#ifdef VERBOSE 
			printf("UNDERSTOOD: E-1,%s (Acknowledge message)\n",sArg2);
#endif
		} else if (channel>=0&&channel<=5) {
			// update Last Error State
			lastError[channel].lastupdated = timestamp;
			lastError[channel].value = errorcode;
#ifdef VERBOSE 
			printf("UNDERSTOOD: E %lf\n", errorcode);
#endif
			return 0;
		}
	}
	// IV<versionHigh>,<versionLow>,<versionBuild> (reply of GIV Get Interface Version) 
	else if (strcmp(sCommand,"IV")==0) {
		MCS_IV[0] = atoi(sArg1);
		MCS_IV[1] = atoi(sArg2);
		MCS_IV[2] = atoi(sArg3);
#ifdef VERBOSE 
		printf("UNDERSTOOD: IV %d, %d, %d\n", MCS_IV[0], MCS_IV[1], MCS_IV[2]);
#endif
		return 0;
	}
	// ID<id>  (reply of GSI, Get System ID)  
	else if (strcmp(sCommand,"ID")==0) {
		strcpy(MCS_ID, sArg1);
#ifdef VERBOSE 
		printf("UNDERSTOOD: ID %s\n", MCS_ID);
#endif
		return 0;
	}
	// NC<id>  (reply of GNC, Get Number of Channels) 
	else if (strcmp(sCommand,"NC")==0) {
		MCS_NC = atoi(sArg1);
#ifdef VERBOSE 
		printf("UNDERSTOOD: NC %d\n", MCS_NC);
#endif
		return 0;
	}
	// P<channelIndex>,<position> (reply of GP, Get Position, position in nm) 
	else if (strcmp(sCommand,"P")==0) {
		int channelIndex = atoi(sArg1);
		double postion = atof(sArg2);
		if (channelIndex>=0&&channelIndex<=5) {
			// update Position State
			stateP[channelIndex].lastupdated = timestamp;
			stateP[channelIndex].value = postion;
#ifdef VERBOSE 
			printf("UNDERSTOOD: P %d %lf\n", channelIndex, postion);
#endif
			return 0;
		}
	}
	// A<channelIndex,<angle> (reply of GA, Get Angle, angle in micro degrees) 
	else if (strcmp(sCommand,"A")==0) {
		int channelIndex = atoi(sArg1);
		double angle = atof(sArg2) + atof(sArg3)*360000000;
		if (channelIndex>=0&&channelIndex<=5) {
			// update Position State
			stateP[channelIndex].lastupdated = timestamp;
			stateP[channelIndex].value = angle;
#ifdef VERBOSE 
			printf("UNDERSTOOD: A %d %lf\n", channelIndex, angle);
#endif
			return 0;
		}
	}
	// PPK<channelIndex>,<known> (reply of GPPK, Get Physical Position Known, 0:unknown, 1: known) 
	else if (strcmp(sCommand,"PPK")==0) {
		int channelIndex = atoi(sArg1);		
		double known = atof(sArg2);		
		if (channelIndex>=0&&channelIndex<=5) {
			// update Position State
			statePPK[channelIndex].lastupdated = timestamp;
			statePPK[channelIndex].value = known;
#ifdef VERBOSE 
			printf("UNDERSTOOD: PPK %d %lf\n", channelIndex, known);
#endif
			return 0;
		}
	}
	// S<channelIndex>,<status> (reply of GS, Get Status) 
	else if (strcmp(sCommand,"S")==0) {
		int channelIndex = atoi(sArg1);
		double status = atof(sArg2);
		if (channelIndex>=0&&channelIndex<=5) {
			// update Position State
			stateS[channelIndex].lastupdated = timestamp;
			stateS[channelIndex].value = status;
#ifdef VERBOSE 
			printf("UNDERSTOOD: S %d %lf\n", channelIndex, status);
#endif
			return 0;
		}
	}
	return 1;
}

int SmarActMCS::FRM(int channelIndex, int direction, int holdTime, int autoZero) 
{
	char cmd[254];
	sprintf(cmd, "FRM%d,%d,%d,%d",channelIndex, direction, holdTime, autoZero);
	return sendMCS(cmd);
}

int SmarActMCS::sendGS(int channelIndex) 
{
	char cmd[254];
	sprintf(cmd, "GS%d",channelIndex);
	return sendMCS(cmd);
}

int SmarActMCS::readGS(int channelIndex, double* status) 
{
	double delay;
	return readGS(channelIndex, status, currentTime, &delay);
}

int SmarActMCS::readGS(int channelIndex, double* status, double timestamp, double* delay) 
{
	// get new messages from input
	parseInput();
	// channelIndex validtiy check (must be 0-5) 
	if (channelIndex<0 && channelIndex>5) return 42;
	// read back status
	*delay = timestamp - stateS[channelIndex].lastupdated;
	*status = stateS[channelIndex].value;
	return 0;
}

int SmarActMCS::sendGP(int channelIndex) 
{
	char cmd[254];
	sprintf(cmd, "GP%d",channelIndex);
	return sendMCS(cmd);
}

int SmarActMCS::readGP(int channelIndex, double* status) 
{
	double delay;
	return readGP(channelIndex, status, currentTime, &delay);
}

int SmarActMCS::readGP(int channelIndex, double* status, double timestamp, double* delay) 
{
	// get new messages from input
	parseInput();
	// channelIndex validtiy check (must be 0-5) 
	if (channelIndex<0 && channelIndex>5) return 42;
	// read back status
	*delay = timestamp - stateP[channelIndex].lastupdated;
	*status = stateP[channelIndex].value;
	return 0;
}

int SmarActMCS::sendGA(int channelIndex) 
{
	char cmd[254];
	sprintf(cmd, "GA%d",channelIndex);
	return sendMCS(cmd);
}

int SmarActMCS::readGA(int channelIndex, double* status) 
{
	double delay;
	return readGA(channelIndex, status, currentTime, &delay);
}

int SmarActMCS::readGA(int channelIndex, double* status, double timestamp, double* delay) 
{
	// get new messages from input
	parseInput();
	// channelIndex validtiy check (must be 0-5) 
	if (channelIndex<0 && channelIndex>5) return 42;
	// read back status
	*delay = timestamp - stateP[channelIndex].lastupdated;
	*status = stateP[channelIndex].value;
	return 0;
}

int SmarActMCS::sendGPPK(int channelIndex) 
{
	char cmd[254];
	sprintf(cmd, "GPPK%d",channelIndex);
	return sendMCS(cmd);
}

int SmarActMCS::readGPPK(int channelIndex, double* status) 
{
	double delay;
	return readGPPK(channelIndex, status, currentTime, &delay);
}

int SmarActMCS::readGPPK(int channelIndex, double* status, double timestamp, double* delay) 
{
	// get new messages from input
	parseInput();
	// channelIndex validtiy check (must be 0-5) 
	if (channelIndex<0 && channelIndex>5) return 42;
	// read back status
	*delay = timestamp - statePPK[channelIndex].lastupdated;
	*status = statePPK[channelIndex].value;
	return 0;
}

int SmarActMCS::sendGSI() 
{
	char cmd[254];
	sprintf(cmd, "GSI");
	return sendMCS(cmd);
}

int SmarActMCS::readGSI(char* ID) 
{
	parseInput();
	strcpy(ID, MCS_ID);
	return 0;
}

int SmarActMCS::sendGNC() 
{
	char cmd[254];
	sprintf(cmd, "GNC");
	return sendMCS(cmd);
}

int SmarActMCS::readGNC(char* ID) 
{
	parseInput();
	strcpy(ID, MCS_ID);
	return 0;
}

int SmarActMCS::SCLF(int channelIndex, int frequency) 
{
	char cmd[254];
	sprintf(cmd, "SCLF%d,%d",channelIndex, frequency);
	return sendMCS(cmd);
}

int SmarActMCS::SCLS(int channelIndex, int speed) 
{
	char cmd[254];
	sprintf(cmd, "SCLS%d,%d",channelIndex, speed);
	return sendMCS(cmd);
}

int SmarActMCS::MPA(int channelIndex, int position, int holdTime)
{
	char cmd[254];
	sprintf(cmd, "MPA%d,%d,%d",channelIndex, position,holdTime);
	return sendMCS(cmd);
}

int SmarActMCS::MPR(int channelIndex, int position, int holdTime)
{
	char cmd[254];
	sprintf(cmd, "MPR%d,%d,%d",channelIndex, position,holdTime);
	return sendMCS(cmd);
}

int SmarActMCS::MAA(int channelIndex, int position, int holdTime)
{
	int period = (int) position / 360000000;
	int angle = position % 360000000;

	if(angle<0){
		angle = 360000000 + angle;
		period--;
	}
	char cmd[254];
	sprintf(cmd, "MAA%d,%d,%d,%d",channelIndex, angle, period, holdTime);
	return sendMCS(cmd);
}

int SmarActMCS::MAR(int channelIndex, int position, int holdTime)
{
	int period = (int) position / 360000000;
	int angle = position % 360000000;

	if(angle<0){
		angle = 360000000 + angle;
		period--;
	}
	char cmd[254];
	sprintf(cmd, "MAR%d,%d,%d,%d",channelIndex, angle, period, holdTime);
	return sendMCS(cmd);
}

int SmarActMCS::SCM(int mode)
{
	char cmd[254];
	sprintf(cmd, "SCM%d",mode);
	return sendMCS(cmd);
}

int SmarActMCS::SHE(int mode)
{
/*0: In this mode the Hand Control Module is disabled. It may not be used to control positioners.
 *1: This is the default setting where the Hand Control Module may be used to control the positioners.
 *2: In this mode the Hand Control Module cannot be used to control the positioners. However, if there
 *   are positioners with sensors attached, their position data will still be displayed.
 */
	if (mode<3 && mode>=0){
		char cmd[254];
		sprintf(cmd, "SHE%d",mode);
		return sendMCS(cmd);
	}
	return 1;
}

int SmarActMCS::S()
{
	char cmd[254];
	sprintf(cmd, "S");
	return sendMCS(cmd);
}

int SmarActMCS::S(int channelIndex)
{
	char cmd[254];
	sprintf(cmd, "S%d",channelIndex);
	return sendMCS(cmd);
}

int SmarActMCS::SP(int channelIndex, int position)
{
	char cmd[254];
	sprintf(cmd, "sP%d,%d",channelIndex, position);
	return sendMCS(cmd);
}

int SmarActMCS::R()
{
	char cmd[254];
	sprintf(cmd, "R");
	return sendMCS(cmd);
}

int SmarActMCS::CB(int baudrate)
{
	char cmd[254];
	sprintf(cmd, "CB%d",baudrate);
	return sendMCS(cmd);
}

int SmarActMCS::CS(int channelIndex) 
{
	char cmd[254];
	sprintf(cmd, "CS%d",channelIndex);
	return sendMCS(cmd);
}
