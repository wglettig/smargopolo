// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close(), usleep()


int openSerialPort() {
    int serial_port;
    // open serial port:
    serial_port = open("/dev/ttyUSB0", O_RDWR);
    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }
    
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
    
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    
    // Set in/out baud rate to be 9600
    //cfsetispeed(&tty, B9600);
    //cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    
    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
    
    

    return serial_port;
}

int main()
{
    int fd = 0;
    fd=openSerialPort();
 
    char full_cmd [1000];
    // add ':' before and '\n' after cmd (SmarAct Command Syntax)
    //sprintf(full_cmd, ":GSI\n:GNC\n:FRM0,1,1,0\n");   
    sprintf(full_cmd, ":GSI\n:GIV\n:GNC\n:GCT0\n:GCT1\n:GCT2\n:GCT3\n:GCT4\n:GCT5\n:SHE1\n:SCM1\n");   
    //fcntl(fd, F_SETFL, 0);        // 0 blocks the system while reading serial port
    int ret = write(fd, full_cmd, strlen(full_cmd));


    usleep(100000); //0.1s
    sprintf(full_cmd, ":FRM0,0,60000,0\n:FRM1,0,60000,0\n:FRM2,0,60000,0\n");
//    sprintf(full_cmd, ":S\n");   
    write(fd, full_cmd, strlen(full_cmd));

    
    sleep(1);

    // Allocate memory for read buffer, set size according to your needs
    char read_buf [256];
    memset(&read_buf, '\0', sizeof(read_buf));

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    int n = read(fd, &read_buf, sizeof(read_buf));
    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (n < 0) {
        printf("Error reading: %s", strerror(errno));
    }
    close(fd);
    
    // n is the number of bytes read. n may be 0 if no bytes were received,
    // and can also be negative to signal an error.
    printf("%d Chars:'%s'\n", n, read_buf);   
       
    return 0;
}

