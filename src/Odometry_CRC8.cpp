// Standard includes

#include <string.h>  /* String function definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <cstdio>
#include <iostream>
#include <math.h>
#include <stdint.h>
#include <cstdlib>

#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

using std::string;
using namespace std;

#define MSG_EXTRA_DATA_SIZE 3
#define HEADER11 0xEE
#define HEADER12 0xFF
#define HEADER21 0xEE
#define HEADER22 0xAA
#define DEFAULT_CRC 0x01
#define FOOTER11 0x00
#define FOOTER12 0xBB
#define FOOTER21 0x01
#define FOOTER22 0xBB
#define DEFAULT_EXTRA1 0x01
#define DEFAULT_EXTRA2 0x00
#define DEFAULT_EXTRA3 0x00
#define DEBUG true
#define sw16(x) \
    ((short)( \
        (((short)(x) & (short)0x00ffU) << 8) | \
        (((short)(x) & (short)0xff00U) >> 8) ))

#pragma pack(push,1)
struct UpLoadMsg {
	//structure of message from Navigation Software To RMC
	unsigned char header1;
	unsigned char header2;
	/* main content */
	unsigned char vsign;
	short vabs;
	unsigned char wsign;
	short wabs;
	/* end main */
	unsigned char extra[MSG_EXTRA_DATA_SIZE];
	unsigned char crc;
	unsigned char footer1;
	unsigned char footer2;
};

struct DownLoadMsg {
	//structure of message from RMC To Navigation Software
	unsigned char header1;
	unsigned char header2;
	/* main content */
	short x;
	short y;
	short theta;
	short v_cur;
	short w_cur;
	/* end main */
	unsigned char extra[MSG_EXTRA_DATA_SIZE];
	unsigned char crc;
	unsigned char footer1;
	unsigned char footer2;
};
#pragma pack(pop)

int init_cmd(int serial_fd);
int write_port(int serial_fd);
int read_port(int serial_fd, struct DownLoadMsg *downMsg, struct timeval *startTime);
int open_port(const char* port);
bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity,
		bool hardware_control);
void close_port(int fd);

bool Run = false;
bool initialed = false;
short v_cmd = 0;
short w_cmd = 0;
int errorCount = 0;
int fd;

int main() {
	if (DEBUG)
		printf("mission start\n");

	char *uart_name = (char*) "/dev/ttyUSB0";
	int baudrate = 57600;

	if (DEBUG)
		printf("Trying to connect to %s.. \n", uart_name);

	fflush(stdout);
	fd = open_port(uart_name);

	if (fd == -1) {
		if (DEBUG)
			printf("failure, could not open port.\n");
		exit(EXIT_FAILURE);
	} else {
		if (DEBUG)
			printf("success.\n");
	}
	if (DEBUG)
		printf("Trying to configure %s.. \n", uart_name);
	bool setup = setup_port(fd, baudrate, 8, 1, false, false);
	if (!setup) {
		if (DEBUG)
			printf("failure, could not configure port.\n");
		exit(EXIT_FAILURE);
	} else {
		if (DEBUG)
			printf("success.\n");
	}

	int noErrors = 0;
	if (fd == -1 || fd == 0) {
		if (DEBUG)
			fprintf(stderr,
					"Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n",
					uart_name, baudrate);
		exit(EXIT_FAILURE);
	} else {
		if (DEBUG)
			fprintf(stderr,
					"\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n",
					uart_name, baudrate);
	}

	if (fd < 0) {
		exit(noErrors);
	}

	//start to communicate with RMC
	struct DownLoadMsg currMsg;

	//give initial command
	init_cmd(fd);

	// Run indefinitely while the serial loop handles data
	if (DEBUG)
		printf("\nREADY, waiting for serial data.\n");

	struct timeval start, end;
	while (1) {
		read_port(fd, &currMsg, &start);
		write_port(fd);
		gettimeofday( &end, NULL );
		int timeuse = 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec;
		int sleepTime = 49000 - timeuse;
		if (DEBUG)
			printf("Write back, done, sleep %d us\n", sleepTime);
		usleep(sleepTime);        //less than 50ms/20Hz
	}

	close_port(fd);

	if (DEBUG)
		printf("mission end\n");
	return 0;
}

/**
 *
 *
 * Returns the file descriptor on success or -1 on error.
 */

int open_port(const char* port) {
	int fd; /* File descriptor for the port */

	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		/* Could not open the port. */
		return (-1);
	} else {
		fcntl(fd, F_SETFL, 0);
	}

	return (fd);
}

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity,
		bool hardware_control) {
	//struct termios options;

	struct termios config;
	if (!isatty(fd)) {
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n",
				fd);
		return false;
	}
	if (tcgetattr(fd, &config) < 0) {
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK
			| ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

#ifdef OLCUC
	config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
	config.c_oflag &= ~ONOEOT;
#endif

	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN] = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	//tcgetattr(fd, &options);

	switch (baud) {
	case 1200:
		if (cfsetispeed(&config, B1200) < 0
				|| cfsetospeed(&config, B1200) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;
	case 1800:
		cfsetispeed(&config, B1800);
		cfsetospeed(&config, B1800);
		break;
	case 9600:
		cfsetispeed(&config, B9600);
		cfsetospeed(&config, B9600);
		break;
	case 19200:
		cfsetispeed(&config, B19200);
		cfsetospeed(&config, B19200);
		break;
	case 38400:
		if (cfsetispeed(&config, B38400) < 0
				|| cfsetospeed(&config, B38400) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;
	case 57600:
		if (cfsetispeed(&config, B57600) < 0
				|| cfsetospeed(&config, B57600) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;
	case 115200:
		if (cfsetispeed(&config, B115200) < 0
				|| cfsetospeed(&config, B115200) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
	case 460800:
		if (cfsetispeed(&config, 460800) < 0
				|| cfsetospeed(&config, 460800) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;
	case 921600:
		if (cfsetispeed(&config, 921600) < 0
				|| cfsetospeed(&config, 921600) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;
	default:
		fprintf(stderr,
				"ERROR: Desired baud rate %d could not be set, aborting.\n",
				baud);
		return false;

		break;
	}

	//
	// Finally, apply the configuration
	//
	if (tcsetattr(fd, TCSAFLUSH, &config) < 0) {
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}

void close_port(int fd) {
	close(fd);
}

int init_cmd(int serial_fd) {
	int fd = serial_fd;

	if (DEBUG)
		printf("Start Writing initial command message to port-- %d.\n", fd);

	unsigned char buf[sizeof(struct UpLoadMsg)] = { HEADER21, HEADER22, 0x01,
			0x00, 0x00, 0x01, 0x00, 0x00, DEFAULT_EXTRA1, DEFAULT_EXTRA2,
			DEFAULT_EXTRA3, DEFAULT_CRC, FOOTER11, FOOTER12 };

	write(fd, buf, sizeof(buf));
	return tcdrain(fd);
}

int write_port(int serial_fd) {
	int fd = serial_fd;

	if (DEBUG)
		printf("Start Writing message to port-- %d.\n", fd);

	if (!Run) {
		v_cmd = 0;
		w_cmd = 0;
	}

	int msgSize = sizeof(struct UpLoadMsg);
	struct UpLoadMsg upMsg;
	unsigned char ubuf[msgSize];

	/* initial upload message */
	upMsg.header1 = HEADER21;
	upMsg.header2 = HEADER22;
	upMsg.extra[0] = DEFAULT_EXTRA1;
	upMsg.extra[1] = DEFAULT_EXTRA2;
	upMsg.extra[2] = DEFAULT_EXTRA3;
	upMsg.crc = DEFAULT_CRC;
	upMsg.footer1 = FOOTER11;
	upMsg.footer2 = FOOTER12;

	if (v_cmd > 0) {
		upMsg.vsign = 0x01;
		upMsg.vabs = v_cmd * 1000;
	} else {
		upMsg.vsign = 0x02;
		upMsg.vabs = -v_cmd * 1000.00;
	}
	if (w_cmd > 0) {
		upMsg.wsign = 0x01;
		upMsg.wabs = w_cmd * 180 / 3.141592653;
	} else {
		upMsg.wsign = 0x02;
		upMsg.wabs = -w_cmd * 180 / 3.141592653;
	}

	if (DEBUG)
		printf(
				"Write message to RMC--vsign: %d, vabs: %d, wsign: %d, wabs: %d\n",
				upMsg.vsign, upMsg.vabs, upMsg.wsign, upMsg.wabs);

	//revert ubuf from little-endian to big-endian
	if (DEBUG)
		printf("DEBUG: start convert endian from little to big.\n");
	upMsg.vabs = sw16(upMsg.vabs);
	upMsg.wabs = sw16(upMsg.wabs);
	if (DEBUG)
		printf("DEBUG: end convert endian from little to big with success.\n");

	if (DEBUG)
		printf("DEBUG: start convert structured message to hex data range.\n");
	memcpy((void *) ubuf, (void *) &upMsg, msgSize);
	if (DEBUG)
		printf(
				"DEBUG: end convert structured message to hex data range with success.\n");

	//delete ubuf;

	if (DEBUG)
		printf("DEBUG: write buff to port -- %d.\n", fd);
	write(fd, ubuf, sizeof(ubuf));
	if (DEBUG)
		printf("DEBUG: write buff to port -- %d, done.\n", fd);
	return tcdrain(fd);
}

unsigned char CRC8(const unsigned char* data, int len) {
	unsigned char crc = 0xFF;
	for (int i = 0; i < len; i++) {
		crc ^= (int) data[i];
		for (int j = 0; j < 8; j++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x31;
			} else {
				crc = crc << 1;
			}
		}
		crc &= 0xff;
	}
	return crc;
}

int read_port(int serial_fd, struct DownLoadMsg *downMsg, struct timeval *startTime) {
	int fd = serial_fd;

	if (DEBUG)
		printf("Start Reading message from port-- %d.\n", fd);

	unsigned int msgSize = sizeof(struct DownLoadMsg);
	unsigned char dbuf[msgSize];

	unsigned char cp;
	unsigned char msgReceived = false;
	unsigned char lastWord = 0x00;
	unsigned char firstRead = true;
	unsigned int validCount = 0;

	while (1) {
		if (read(fd, &cp, 1) > 0) {
			if (firstRead) {
				lastWord = cp;
				firstRead = false;
				continue;
			}
			if (validCount > 0 && validCount < msgSize) {
				//if already start download, append read to message buffer
				dbuf[validCount++] = cp;
				if (validCount == msgSize) {
					msgReceived = true;
				}
			} else if (lastWord == HEADER11 && cp == HEADER12
					&& validCount == 0) {
				gettimeofday( startTime, NULL );
				//test this is the start and if it is, start download message
				dbuf[validCount++] = lastWord;
				dbuf[validCount++] = cp;
			} else {
				if (DEBUG)
					printf("DEBUG: We receive crap-- %c\n", cp);
			}
			lastWord = cp;
		} else {
			if (DEBUG)
				printf("WARNING: Could not read from fd %d\n", fd);
			if (validCount > 0) {
				msgReceived = true;
			}
		}

		if (!msgReceived)
			continue;

		if (DEBUG)
			printf("DEBUG: read range data from fd %d\n", fd);

		//if message has not been complete received, jump out the loop, assume RMC already give us a message but we can not get it
		if (validCount != msgSize || sizeof(dbuf) != msgSize)
			break;

		//CRC8 check
		int cbufSize = msgSize - 3;
		unsigned char cbuf[cbufSize];
		if (DEBUG)
			printf("DEBUG: start crc check.\n");
		memcpy((void *) cbuf, (void *) dbuf, cbufSize);
		if (dbuf[15] != CRC8(cbuf, cbufSize)) {
			if (DEBUG)
				printf("ERROR: CRC8 check failed.\n");
			break;
		}
		if (DEBUG)
			printf("DEBUG: end crc check with success.\n");

		Run = dbuf[16] == 0x00;

		short lx = downMsg->x;
		short ly = downMsg->y;
		short lt = downMsg->theta;

		//read received message
		if (DEBUG)
			printf("DEBUG: start convert data to structured message.\n");
		memcpy((void *) downMsg, (void *) dbuf, msgSize);
		if (DEBUG)
			printf(
					"DEBUG: end convert data to structured message with success.\n");

		//revert dbuf from big-endian to little-endian before parse it
		if (DEBUG)
			printf("DEBUG: start convert endian from big to little.\n");
		downMsg->x = sw16(downMsg->x);
		downMsg->y = sw16(downMsg->y);
		downMsg->theta = sw16(downMsg->theta);
		downMsg->v_cur = sw16(downMsg->v_cur);
		downMsg->w_cur = sw16(downMsg->w_cur);
		if (DEBUG)
			printf("DEBUG: end convert endian from big to little.\n");

		if (DEBUG)
			printf(
					"Read original message from RMC--x: %d, y: %d, theta: %d, v_cur: %d, w_cur: %d\n",
					downMsg->x, downMsg->y, downMsg->theta, downMsg->v_cur,
					downMsg->w_cur);

		//deal with received message
		downMsg->x = downMsg->x / 100.0;
		downMsg->y = downMsg->y / 100.0;
		downMsg->theta = downMsg->theta / 180.0 * 3.141592653;
		downMsg->v_cur /= 100.0;
		downMsg->w_cur /= 100.0;

		if (DEBUG)
			printf(
					"Read converted message from RMC--x: %d, y: %d, theta: %d, v_cur: %d, w_cur: %d\n",
					downMsg->x, downMsg->y, downMsg->theta, downMsg->v_cur,
					downMsg->w_cur);

		if (initialed) {
			if (fabs(downMsg->theta) > 6.5 || fabs(lx - downMsg->x) > 0.5
					or fabs(ly - downMsg->y) > 0.5) {
				errorCount++;
				//may be the odometry is reset
				if (DEBUG)
					printf("error package-- %d, %d, %d, now-- %d, %d, %d\n", lx,
							ly, lt, downMsg->x, downMsg->y, downMsg->theta);
				if (errorCount < 50) {
					downMsg->x = lx;
					downMsg->y = ly;
					downMsg->theta = lt;
				}
			} else {
				errorCount = 0;
			}
		}
		initialed = true;
		//done with read current message, break endless loop
		break;
	}

	return 0;
}
