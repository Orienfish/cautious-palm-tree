/*
 * This is the main file for modbus tcp communication
 * Modified by xfyu on Jan 25, 2018
 * Main Function：
 * a.using modbus tcp to read ur3 TCP's position[x,y,z,rx,ry,rz], in base 
 *   coordinate system
 * b.read ur3's 6 joints' angle[base,shoulder,elbow,wrist1,wrist2,wrist3]
 *
 * Note: [x,y,z,rx,ry,rz] are signed
 * [base,shoulder,elbow,wrist1,wrist2,wrist3] are unsigned.
 * So you may consider limit the scope of each joint to make sure that
 * their sign is certain, either >0 or <0.
 */

/*
 * Libraries
 */
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

/*
 * Macro Definitions
 */
#define DEBUG

#define ROBOT_ADDR "192.168.0.1" /* server addr */
#define MODBUS_PORT 502 /* server port */
#define BUF_SIZE 1024 /* maximum rev length */
#define REG_NUM 6 /* num of reg to read */

/*
 * Global Variables
 */
/* request frame for the tcp pose */
unsigned char pos_req_frm[12] = { 
	0x00, 0x01, /* sequence number */
	0x00, 0x00, /* protocol identifier */
	0x00, 0x06, /* package length */
	0x00, 0x04, /* function code for read input registers */
	0x01, 0x90, /* addr of first reg: 400 */
	0x00, 0x06  /* total number of reg to read */
};

/* request frame for the wrist angle */
unsigned char wrist_req_frm[12] = {
	0x00, 0x01, /* sequence number */
	0x00, 0x00, /* protocol identifier */
	0x00, 0x06, /* package length */
	0x00, 0x04, /* function code for read input registers */
	0x01, 0x0e, /* addr of first reg: 400 */
	0x00, 0x06  /* total number of reg to read */
};

/*
 * Local Function Definitions
 */
int connect_modbus();
int read_pos(int modbus_fd, short * recv_value);
int read_wrist(int modbus_fd, short * recv_value);
void print_values(short * recv_value);
int main();

/*
 * Local Function Realizations
 */

/*
 * connect_modbus - establish the connection with modbus
 * Only open the connection, need the caller to close
 * the connection once finished.
 *
 * Parameter: none
 * Return value: >0 - success, the modbus_fd
 *               -1 - error
 */
int connect_modbus() {
	/* variables used in TCP connection */
	int clientSocket;
	struct sockaddr_in serverAddr;

	/* set up the socket of client */
	if ((clientSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		perror("socket error");
		return -1;
	}

	/* set the parameters */
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(MODBUS_PORT);
	serverAddr.sin_addr.s_addr = inet_addr(ROBOT_ADDR);
	/* try to connect to the server */
	if (connect(clientSocket, (struct sockaddr *)&serverAddr, 
			sizeof(serverAddr)) < 0) {
		perror("connect error");
		return -1;
	}
#ifdef DEBUG
	printf("connect to port %d suceeded\n", MODBUS_PORT);
#endif

	return clientSocket;
}

/*
 * read_pos - Read 6 pos value
 * p[x, y, z, rx, ry, rz]. The x, y, z are in 0.1mm base.
 * The rx, ry, rz are in 0.001rad base. 
 *
 * Parameter: recv_value - the space to store the data
 * Return value: 0 - succeed
 *               nonzero - trasaction error
 *               1 - invalid data
 */
int read_pos(int clientSocket, short * recv_value) {
	/* receive buffer */
	unsigned char recvbuf[BUF_SIZE];
	int res; /* ret num */

	/* Send request */
	if (write(clientSocket, pos_req_frm, 
			sizeof(pos_req_frm)) < 0) {
		perror("send error");
		return 1;
	}

	/* store the size that receive */
	if ((res = read(clientSocket, recvbuf, BUF_SIZE)) < 0) {
		perror("receive error");
		return 1;
	}
#ifdef DEBUG
	printf("Receive Bytes:");
	for (int i = 0; i < res; ++i) {
		printf("%x ", recvbuf[i]);
	}
	printf("\n");
#endif

	if (res >= 21) /* check if all 6 regs are read */
		for (int i = 0; i < REG_NUM; ++i) {
			int index = 9 + 2 * i; /* offset in the recvbuf */
			recv_value[i] = recvbuf[index] * 256 + 
				recvbuf[index + 1];
		}
	else
		return 1;

	return 0;
}

/*
 * read_wrist - Read 6 joint value
 * [base, shoulder, elbow, wrist1, wrist2, wrist3].
 * All values are in 0.001rad base. 
 *
 * Parameter: recv_value - the space to store the data
 * Return value: 0 - succeed
 *               nonzero - error
 *               1 - invalid data
 */
int read_wrist(int clientSocket, short * recv_value) {
	/* receive buffer */
	unsigned char recvbuf[BUF_SIZE];
	int res; /* ret num */

	/* write request */
	if ((write(clientSocket, wrist_req_frm, 
			sizeof(wrist_req_frm))) < 0) {
		perror("send error");
		return 1;
	}

	/* store the size that receive */
	if ((res = read(clientSocket, recvbuf, BUF_SIZE)) < 0) {
		perror("receive error");
		return 1;
	}
#ifdef DEBUG
	printf("Receive Bytes:");
	for (int i = 0; i < res; ++i) {
		printf("%x ", recvbuf[i]);
	}
	printf("\n");
#endif

	if (res >= 21) /* check if all 6 regs are read */
		for (int i = 0; i < REG_NUM; ++i) {
			int index = 9 + 2 * i; /* offset in the recvbuf */
			recv_value[i] = recvbuf[index] * 256 + recvbuf[index + 1];
		}
	else
		return 1; /* invalid data */

	/* close the transaction */
	close(clientSocket);
	return 0;
}

/*
 * print_values - print the received 6 data
 *
 * Parameters: recv_value - the data to print
 * No return value
 */
void print_values(short * recv_value) {
	printf("Receive Values:");
		for (int i = 0; i < REG_NUM; ++i)
			printf("%d ", recv_value[i]);
	printf("\n");
}

/*
 * main function
 */
int main() {
	/* 2 different fd used in connection */
	int modbus_fd, realtime_fd;
	/* init the space for receive value */
	short * recv_value = malloc(REG_NUM * sizeof(short));

	if ((modbus_fd = connect_modbus()) < 0)
		 return 1;

    /* read the pos successfully, store and print */
	if (!read_pos(modbus_fd, recv_value))
		print_values(recv_value);

	/* read the joint successfully, store and print */
	if (!read_wrist(modbus_fd, recv_value))
		print_values(recv_value);

    /* free the space */
	free(recv_value);
	/* close the transaction */
	close(modbus_fd);
}