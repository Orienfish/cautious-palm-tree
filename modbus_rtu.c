/*
 * This is the main file for modbus tcp communication
 * Modified by xfyu on Jan 25, 2018
 * Main Function：
 * a.using modbus rtu to activate gripper
 * b.close the gripper with full force and full speed
 * c.open the gripper with full force and full speed
 *
 * Note: some of the instruction is different from 
 * the Instruction Manual provided by Universal Robot.
 * I don't know why they are different, but I get these
 * from real robot test.
 */
 
/*
 * Libraries
 */
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#define MODBUS_DEV "/dev/ttyUSB0"
#define BAUDRATE B115200

/* already defined otherwhere */
#define BUF_SIZE 512

#define DEBUG

/*
 * Global Variables
 */
unsigned char activate[] = {
	0x09, 0x10,
	0x03, 0xe8,
	0x00, 0x03,
	0x06,
	0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00,
	0x73, 0x30
};

unsigned char read_gripper_status[] = {
	0x09, 0x03,
	0x07, 0xd0,
	0x00, 0x01,
	0x85, 0xcf
};

unsigned char activate_success[] = {
	0x09, 0x03,
	0x02,
	0x00, 0x00,
	0x59, 0x85
};

unsigned char close_with_full_speed_full_force[] = {
	0x09, 0x10,
	0x03, 0xe8,
	0x00, 0x03,
	0x06,
	0x09, 0x00,
	0x00, 0xff,
	0xff, 0xff,
	0x42, 0x29
};

unsigned char read_until_grip_completed[] = {
	0x09, 0x03,
	0x07, 0xd0,
	0x00, 0x03,
	0x04, 0x0e
};

unsigned char grip_is_completed[] = {
	0x09, 0x03,
	0x02, 0xf9,
	0x00, 0x1b, 0xd5
};

unsigned char open_with_full_speed_full_force[] = {
	0x09, 0x10,
	0x03, 0xe8,
	0x00, 0x03,
	0x06,
	0x09, 0x00,
	0x00, 0x00,
	0xff, 0xff,
	0x72, 0x19
};

unsigned char read_until_open_completed[] = {
	0x09, 0x03,
	0x07, 0xd0,
	0x00, 0x03,
	0x04, 0x0e
};

unsigned char open_is_completed[] = {
	0x09, 0x03,
	0x06,
	0xf9, 0x00,
	0x00, 0x00,
	0x03, 0x00,
	0x52, 0x2c
};

/*
 * Function Definitions
 */
int bufcmp(unsigned char *s1, unsigned char *s2);
int open_modbus();
int gripper_activate();
int gripper_close();
int gripper_open();

/*
 * bufcmp - Compare the recv buf with what we 
 * already have
 *
 * Input: s1 - addr of the first buf
 *        s2 - addr of the second buf
 * Return Value: 0 - same
 *               1 - different
 * We don't have to know which buf is smaller, we
 * only care whether they are same.
 */
int bufcmp(unsigned char *s1, unsigned char *s2) {
	int len1 = strlen((char *)s1);
	int len2 = strlen((char *)s2);
	if (len1 != len2)
		return 1; /* match fail */
	for (int i = 0; i < len1; ++i)
		if (s1[i] != s2[i])
			return 1;
	return 0;
}

/*
 * open_modbus - open the serial port
 *
 * Return Value: >0 - the fd. success
 *            	 <=0 - fail
 */
int open_modbus() {
	int fd;

	fd = open(MODBUS_DEV, O_RDWR);
	if (fd < 0) {
		perror("open tty error");
		return -1;
	}

	struct termios options;
	tcgetattr(fd, &options);
	memset(&options, 0, sizeof(options));
	// options.c_cflag |= CLOCAL | CREAD;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8; /* 8 data bit */

	options.c_cflag &= ~PARENB; /* no parity */
	options.c_cflag &= ~CSTOPB; /* 1 stop bit */

	/* set the baudrate */
	if (cfsetispeed(&options, BAUDRATE) < 0) {
		perror("baudrate seti error");
		return -1;
	}
	if (cfsetospeed(&options, BAUDRATE) < 0) {
		perror("baudrate seto error");
		return -1;
	}
	/* set the wait time */
	options.c_cc[VTIME] = 10;
	options.c_cc[VMIN] = 4;

	/* bind the options to fd */
	if (tcsetattr(fd, TCSANOW, &options) < 0) {
		perror("attr set error");
		return -1;
	}

	return fd;
}

/*
 * gripper_activate - activate the gripper
 *
 * Return Value: 0 - success
 *               -1 - fail
 */
int gripper_activate() {
	int fd;
	int read_cnt;
	unsigned char recv_buf[BUF_SIZE];

	if ((fd = open_modbus()) < 0)
		return -1;

	/* activate */
	if (write(fd, activate, sizeof(activate)) < 0) {
		perror("write error");
		return -1;
	}

#ifdef DEBUG	
	if ((read_cnt = read(fd, recv_buf, BUF_SIZE)) < 0) {
		perror("read error");
		return -1;
	}

	fprintf(stdout, "Activate Receive: ");
	for (int i = 0; i < read_cnt; ++i)
		fprintf(stdout, "0x%x ", recv_buf[i]);
	fprintf(stdout, "\n");
#endif

	while (1) {
		if (write(fd, read_gripper_status, sizeof(read_gripper_status)) < 0) {
			perror("write error");
			return -1;
		}
		/* recv gripper status */
		if ((read_cnt = read(fd, recv_buf, BUF_SIZE)) < 0) {
			perror("read error");
			return -1;
		}
		if (!bufcmp(activate_success, recv_buf))
			break; /* complete */
		else
			continue; /* not complete */
	}

	close(fd);
	return 0;
}

/*
 * gripper_close - close the gripper
 *
 * Return Value: 0 - success
 *               -1 - fail
 */
int gripper_close() {
	int fd;
	int read_cnt;
	unsigned char recv_buf[BUF_SIZE];

	if ((fd = open_modbus()) < 0)
		return -1;

	/* grip */
	if (write(fd, close_with_full_speed_full_force, 
			sizeof(close_with_full_speed_full_force)) < 0) {
		perror("write error");
		return -1;
	}
	while (1) {
		if (write(fd, read_gripper_status, 
				sizeof(read_gripper_status)) < 0) {
			perror("write error");
			return -1;
		}
		/* recv gripper status */
		if ((read_cnt = read(fd, recv_buf, BUF_SIZE)) < 0) {
			perror("read error");
			return -1;
		}
#ifdef DEBUG
	fprintf(stdout, "Close Receive: ");
	for (int i = 0; i < read_cnt; ++i)
		fprintf(stdout, "0x%x ", recv_buf[i]);
	fprintf(stdout, "\n");
#endif
		if (!bufcmp(grip_is_completed, recv_buf))
			break; /* complete */
		else
			continue; /* not complete */
	}

	close(fd);
	return 0;
}

/*
 * gripper_open - open the gripper
 *
 * Return Value: 0 - success
 *               -1 - fail
 */
int gripper_open() {
	int fd;
	int read_cnt;
	unsigned char recv_buf[BUF_SIZE];

	if ((fd = open_modbus()) < 0)
		return -1;

	/* open */
	if (write(fd, open_with_full_speed_full_force, 
			sizeof(open_with_full_speed_full_force)) < 0) {
		perror("write error");
		return -1;
	}
	while (1) {
		if (write(fd, read_until_open_completed, 
				sizeof(read_until_open_completed)) < 0) {
			perror("write error");
			return -1;
		}
		/* recv gripper status */
		if ((read_cnt = read(fd, recv_buf, BUF_SIZE)) < 0) {
			perror("read error");
			return -1;
		}
#ifdef DEBUG
	fprintf(stdout, "Open Receive: ");
	for (int i = 0; i < read_cnt; ++i)
		fprintf(stdout, "0x%x ", recv_buf[i]);
	fprintf(stdout, "\n");
#endif
		if (!bufcmp(open_is_completed, recv_buf))
			break; /* complete */
		else
			continue; /* not complete */
	}

	close(fd);
	return 0;
}

/*
 * main function
 */
int main() {
	int ins;
	
	/* activate first */
	if (gripper_activate() < 0)
		return 1;
		
	while (1) { /* a loop */
		printf("Please input instruction:\n");
		printf("1 for close, 2 for open\n");
	
		scanf("%d", &ins);
		if (ins == 1) {
			if (gripper_close() < 0)
				return 1;
		}
		else if (ins == 2) {
			if (gripper_open() < 0)
				return 1;
		}
		else
			printf("wrong type!\n");
	}
	return 0;
}