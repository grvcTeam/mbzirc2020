/*
 * Functions ripped straight from player's rflex driver to fix the issue
 * with port initialization.
 */

#ifndef RFLEX_PLAYER_HH_
#define RFLEX_PLAYER_HH_

#include <rflex/rflex_info.h>
#include <fcntl.h>
#include <termios.h>

#define MAX_NAME_LENGTH                256

enum PARITY_TYPE   { N, E, O };

typedef struct {
  char                       ttyport[MAX_NAME_LENGTH];
  int                        baud;
  enum PARITY_TYPE           parity;
  int                        fd;
  int                        databits;
  int                        stopbits;
  int                        hwf;
  int                        swf;
} RFLEX_Device;

// From rflex-io.cc
int iParity(enum PARITY_TYPE par) {
	if (par == N)
		return (IGNPAR);
	else
		return (INPCK);
}

int iSoftControl(int flowcontrol) {
	if (flowcontrol)
		return (IXON);
	else
		return (IXOFF);
}

int cDataSize(int numbits) {
	switch (numbits) {
	case 5:
		return (CS5);
		break;
	case 6:
		return (CS6);
		break;
	case 7:
		return (CS7);
		break;
	case 8:
		return (CS8);
		break;
	default:
		return (CS8);
		break;
	}
}

int cStopSize(int numbits) {
	if (numbits == 2) {
		return (CSTOPB);
	} else {
		return (0);
	}
}

int cFlowControl(int flowcontrol) {
	if (flowcontrol) {
		return (CRTSCTS);
	} else {
		return (CLOCAL);
	}
}

int cParity(enum PARITY_TYPE par) {
	if (par != N) {
		if (par == O) {
			return (PARENB | PARODD);
		} else {
			return (PARENB);
		}
	} else {
		return (0);
	}
}

int cBaudrate(int baudrate) {
	switch (baudrate) {
	case 0:
		return (B0);
		break;
	case 300:
		return (B300);
		break;
	case 600:
		return (B600);
		break;
	case 1200:
		return (B1200);
		break;
	case 2400:
		return (B2400);
		break;
	case 4800:
		return (B4800);
		break;
	case 9600:
		return (B9600);
		break;
	case 19200:
		return (B19200);
		break;
	case 38400:
		return (B38400);
		break;
	case 57600:
		return (B57600);
		break;
	case 115200:
		return (B115200);
		break;
#ifdef B230400
	case 230400:
		return (B230400);
		break;
#endif

	default:
		return (B9600);
		break;
	}

}

void DEVICE_set_params(RFLEX_Device dev) {
	struct termios ctio;

	tcgetattr(dev.fd, &ctio); /* save current port settings */

	ctio.c_iflag = iSoftControl(dev.swf) | iParity(dev.parity);
	ctio.c_oflag = 0;
	ctio.c_cflag = CREAD | cFlowControl(dev.hwf || dev.swf)
			| cParity(dev.parity) | cDataSize(dev.databits)
			| cStopSize(dev.stopbits);

	ctio.c_lflag = 0;
	ctio.c_cc[VTIME] = 0; /* inter-character timer unused */
	ctio.c_cc[VMIN] = 0; /* blocking read until 0 chars received */

	cfsetispeed(&ctio, (speed_t) cBaudrate(dev.baud));
	cfsetospeed(&ctio, (speed_t) cBaudrate(dev.baud));

	tcflush(dev.fd, TCIFLUSH);
	tcsetattr(dev.fd, TCSANOW, &ctio);

}

int DEVICE_connect_port(RFLEX_Device *dev) {
	if ((dev->fd = open((dev->ttyport), (O_RDWR | O_NOCTTY), 0)) < 0) {
		return (-1);
	}
	DEVICE_set_params(*dev);
	return (dev->fd);
}
// /rflex-io.h

int rflex_open_connection(const char *device_name, int *fd) {
	RFLEX_Device rdev;

	strncpy(rdev.ttyport, device_name, MAX_NAME_LENGTH);
	rdev.baud = 115200;
	rdev.databits = 8;
	rdev.parity = N;
	rdev.stopbits = 1;
	rdev.hwf = 0;
	rdev.swf = 0;

	printf("trying port %s\n", rdev.ttyport);
	if (DEVICE_connect_port(&rdev) < 0) {
		fprintf(stderr, "Can't open device %s\n", rdev.ttyport);
		return -1;
	}

	*fd = rdev.fd;
	return 0;
}

#endif /* RFLEX_PLAYER_HH_ */
