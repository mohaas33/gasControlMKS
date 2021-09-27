// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream> // For file I/O (reading/writing to COM port)
#include <sstream>
#include <vector>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
//#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <sys/ioctl.h>// Used for TCGETS2/TCSETS2, which is required for custom baud rates
#include <asm/termbits.h>

//struct termios tty;
//#define    BOTHER 0010000

int main(int argc, char **argv) {
	// Create serial port object and open serial port
	const std::string device = "/dev/ttyUSB1";
	int serial_port = open(device.c_str(), O_RDWR);
	printf("%d \n",serial_port);
	// Check for errors
	if (serial_port < 0) {
	    printf("Error %i from open: %s\n", errno, strerror(errno));
	}
	// Create new termios struct, we call it 'tty' for convention
	// No need for "= {0}" at the end as we'll immediately write the existing
	// config to this struct
	struct termios2 tty;
	ioctl(serial_port, TCGETS2, &tty);
	//if(tcgetattr(serial_port, &tty) != 0) {
	//	printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	//}
	//================= (.c_cflag) ===============//
	//tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
	tty.c_cflag |= PARODD; //Odd parity is used
	tty.c_cflag     &=  ~CSTOPB;		// Only one stop-bit is used
	tty.c_cflag     &=  ~CSIZE;			// CSIZE is a mask for the number of bits per character
	tty.c_cflag     |=  CS8;			// Set to 8 bits per character
	tty.c_cflag     &=  ~CRTSCTS;       // Disable hadrware flow control (RTS/CTS)
	tty.c_cflag     |=  CREAD | CLOCAL;     				// Turn on READ & ignore ctrl lines (CLOCAL = 1)


    //===================== BAUD RATE =================//
	tty.c_cflag &= ~CBAUD;
	tty.c_cflag |= CBAUDEX;

	tty.c_ispeed = 9600;
	tty.c_ospeed = 9600;
		
	//===================== (.c_oflag) =================//

	tty.c_oflag     =   0;              // No remapping, no delays
	tty.c_oflag     &=  ~OPOST;			// Make raw

	//================= CONTROL CHARACTERS (.c_cc[]) ==================//

    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

	//======================== (.c_iflag) ====================//

	tty.c_iflag     &= ~(IXON | IXOFF | IXANY);			// Turn off s/w flow ctrl
	tty.c_iflag 	&= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);


	//=========================== LOCAL MODES (c_lflag) =======================//
	tty.c_lflag |= ECHO;
	//tty.c_lflag &= ~(ECHO);
	tty.c_lflag		&= ~ECHOE;								// Turn off echo erase (echo erase only relevant if canonical input is active)
	tty.c_lflag		&= ~ECHONL;								//
	tty.c_lflag		&= ~ISIG;								// Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

	//  apply attributes
	ioctl(serial_port, TCSETS2, &tty);
	//tcflush(serial_port, TCIFLUSH);
	//if(tcsetattr(serial_port, TCSANOW, &tty) != 0)
	//{
	// 	// Error occurred
	// 	std::cout << "Could not apply terminal attributes for \"" << device << "\" - " << strerror(errno) << std::endl;
	// 	throw std::system_error(EFAULT, std::system_category());
	//
	//}
	// Read in existing settings, and handle any error
	// NOTE: This is important! POSIX states that the struct passed to tcsetattr()
	// must have been initialized with a call to tcgetattr() overwise behaviour
	// is undefined
	//if(tcgetattr(serial_port, &tty) != 0) {
    //	printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	//}

	//Read/Write
	//std::string msg = argv[1];//"ID";
	std::string msg = "ID";
	//unsigned char msg[] = { 'I', 'D','\r'};

	printf("write \n");
	printf("Send: %s \n",msg.c_str());
	int writeResult = write(serial_port, msg.c_str(), msg.size());
	if (writeResult < 0) {
	    printf("Error %i from open: %s\n", errno, strerror(errno));
	}
	//int writeResult = write(serial_port, msg, sizeof(msg));
	// Check status
	//if (writeResult == -1) {
	//	throw std::system_error(EFAULT, std::system_category());
	//}
	usleep ((7 + 25) * 100); 
	// Allocate memory for read buffer, set size according to your needs
	std::vector<char> read_buf;
	int def_size = 1024;
	printf("read \n");
	// Read bytes. The behaviour of read() (e.g. does it block?,
	// how long does it block for?) depends on the configuration
	// settings above, specifically VMIN and VTIME
	printf("%d",serial_port);
	ssize_t n = read(serial_port, &read_buf[0], def_size);
	// Error Handling
	//if(n < 0) {
	//	// Read was unsuccessful
	//	throw std::system_error(EFAULT, std::system_category());
	//}
	// Check for errors
	if (n < 0) {
	    printf("Error %i from open: %s\n", errno, strerror(errno));
	}
	printf("convert to string \n");
	//std::string data = std::string(&read_buf[0], n);
	printf("%d \n",n);
	//printf("%s \n",data.c_str());
	// n is the number of bytes read. n may be 0 if no bytes were received, and can also be negative to signal an error.
	printf("close \n");
	close(serial_port);
	/*
	SerialPort serialPort("/dev/ttyUSB1", BaudRate::B_9600);
	// Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
	serialPort.SetTimeout(0); // Block when reading until any data is received
	serialPort.Open();

	// Write some ASCII datae
	serialPort.Write("ID");

	// Read some data back (will block until at least 1 byte is received due to the SetTimeout(-1) call above)
	std::string readData;
	printf("Read \n");
	serialPort.Read(readData);
	printf("%s \n",readData);
	// Close the serial port
	serialPort.Close();
	*/
}