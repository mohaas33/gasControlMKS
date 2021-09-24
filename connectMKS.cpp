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
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	//tty.c_cflag |= PARENB;  // Set parity bit, enabling parity

	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	//tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication

	tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
	//tty.c_cflag |= CS5; // 5 bits per byte
	//tty.c_cflag |= CS6; // 6 bits per byte
	//tty.c_cflag |= CS7; // 7 bits per byte
	tty.c_cflag |= CS8; // 8 bits per byte (most common)

	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	//tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
	
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	// Set custom baud rate
	tty.c_cflag &= ~CBAUD;
	tty.c_cflag |= CBAUDEX;
	// Set in/out baud rate to be 9600
	tty.c_ispeed = 9600;
	tty.c_ospeed = 9600;
	
	tty.c_oflag = 0;
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	//tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
	
	// Setting both to 0 will give a non-blocking read
	tty.c_cc[VTIME] = 0;//10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;


	//======================== (.c_iflag) ====================//

	tty.c_iflag     &= ~(IXON | IXOFF | IXANY);			// Turn off s/w flow ctrl
	tty.c_iflag 	&= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
	//=========================== LOCAL MODES (c_lflag) =======================//

	// Canonical input is when read waits for EOL or EOF characters before returning. In non-canonical mode, the rate at which
	// read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]
	tty.c_lflag		&= ~ICANON;		
	// Configure echo depending on echo_ boolean
	//tty.c_lflag |= ECHO;
	tty.c_lflag 	&= ~(ECHO);
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
	std::string msg = argv[1];//"ID";
	//unsigned char msg[] = { 'I', 'D','\r'};

	printf("write \n");
	printf("Send: %s \n",msg.c_str());
	int writeResult = write(serial_port, msg.c_str(), msg.size());
	//int writeResult = write(serial_port, msg, sizeof(msg));
	// Check status
	if (writeResult == -1) {
		throw std::system_error(EFAULT, std::system_category());
	}

	// Allocate memory for read buffer, set size according to your needs
	std::vector<char> read_buf;
	int def_size = 255;
	printf("read \n");
	// Read bytes. The behaviour of read() (e.g. does it block?,
	// how long does it block for?) depends on the configuration
	// settings above, specifically VMIN and VTIME
	ssize_t n = read(serial_port, &read_buf[0], def_size);
	std::string data = std::string(&read_buf[0], n);
	printf("%d \n",n);
	printf("%s \n",data.c_str());
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