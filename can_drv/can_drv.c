/**
*******************************************************************************
* @file           : can_drv.c
* @brief          : Description of C implementation module
* @author         : Gonzalo Rivera
* @date           : 29/03/2023
*******************************************************************************
* @attention
*
* Copyright (c) <date> grivera. All rights reserved.
*
*/
/******************************************************************************
    Includes
******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <pthread.h>
#include <semaphore.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_drv.h"
#include "ring_buffer.h"

#define LOG_LOCAL_LEVEL LOG_INFO
#define LOG_CONFIG_TIMESTAMP LOG_TIMESTAMP
#include "log.h"
/******************************************************************************
    Defines and constants
******************************************************************************/
#define CAN_LOOPBACK_DISABLE 0
/******************************************************************************
    Data types
******************************************************************************/

/******************************************************************************
    Local variables
******************************************************************************/
static const char *MODULE_NAME = "[CAN_DRIVER]";
static can_t can_buffer_tx[CAN_BUFFER_LENGHT] = {0};
static can_t can_buffer_rx[CAN_BUFFER_LENGHT] = {0};
static ring_buffer_t ring_buffer_rx = {0};
static ring_buffer_t ring_buffer_tx = {0};

/* can options */
static char *vcan = "vcan0";
static char *can = "can0";

/* socket can */
static int s; /* can raw socket */
static struct sockaddr_can addr = {0};
static struct ifreq ifr = {0};

static sem_t semaphore;
/******************************************************************************
    Local function prototypes
******************************************************************************/
static void *can_listener(void *params);
static void *can_transmitter(void *params);
/******************************************************************************
    Local function definitions
******************************************************************************/
static void *can_listener(void *params)
{
	struct can_frame frame = {0};
	int nbytes = 0; 
	can_t can_msg = {0};
	
	LOGI(MODULE_NAME, "can listener init");

	while(1)
	{
		nbytes = read(s, &frame, sizeof(struct can_frame));

		if (nbytes < 0)
		{
			perror("can raw socket read");
			break;
		}

		if(nbytes == CAN_MTU)	// can frame 
		{
			/* parse can frame and storage in buffer*/
			//can_msg.id = frame.can_id;
			can_msg.id = 0x080000000 | frame.can_id;	// see page 6 of simma manual.
			can_msg.buf_len = frame.can_dlc;
			memcpy(can_msg.buf, frame.data, frame.can_dlc);
        
			sem_wait(&semaphore);
			ring_buffer_queue(&ring_buffer_rx, &can_msg);
			sem_post(&semaphore);
		}

		usleep(100000);
	}

    close(s);
    pthread_exit(NULL);
}

static void *can_transmitter(void *params)
{
	can_t can_msg_tx = {0};
	uint8_t msgs = 0; 
	
	struct can_frame frame = {0};
	int nbytes = 0; 

	LOGI(MODULE_NAME, "can transmiter init");

	while(1)
	{
		sem_wait(&semaphore);
		msgs = ring_buffer_dequeue(&ring_buffer_tx, &can_msg_tx);
		sem_post(&semaphore);

		if(msgs > 0)
		{
			frame.can_dlc = can_msg_tx.buf_len;
			frame.can_id = can_msg_tx.id;
			memcpy(frame.data, can_msg_tx.buf, can_msg_tx.buf_len);
			nbytes = write(s, &frame, sizeof(struct can_frame));
			if(nbytes != CAN_MTU)
			{
				perror("can raw socket write");
			}
		}

		usleep(100000);
	}

    close(s);
    pthread_exit(NULL);	
}
/******************************************************************************
    Public function definitions
******************************************************************************/
int can_init(uint8_t port)
{
	int ret = 0; 
	pthread_t can_rx_thread;	// hilo para escuchar el puerto can 
	pthread_t can_tx_thread;	// hilo para transmitir mensajes can existentes en el buffer de tx

    /* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;	// ?
	}

#if CAN_LOOPBACK_DISABLE
	/*
	 * disable default receive filter on this RAW socket
	 * This is obsolete as we do not read from the socket at all, but for
	 * this reason we can remove the receive list in the Kernel to save a
	 * little (really a very little!) CPU usage.
	 */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
	const int loopback = 0;
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
#endif

	/* set name and index device */
	if(0 == port)
	{
		strncpy(ifr.ifr_name, can, strlen(can));	
	}

	if(1 == port)
	{
		strncpy(ifr.ifr_name, vcan, strlen(vcan));
	}
	
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	/*  bind socket */
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
		perror("Error in socket bind");
		return -2;	// ?
	}
	
	LOGI(MODULE_NAME, "%s at index %d", ifr.ifr_name, ifr.ifr_ifindex);
    LOGI(MODULE_NAME, "socket can initializate");
	
	/* ring buffer init */
	ret = ring_buffer_init(&ring_buffer_tx, &can_buffer_tx[0], sizeof(can_buffer_tx), sizeof(can_t));
	if(0 != ret)
	{
		perror("\"ring buffer rx\"\n");
		return EXIT_FAILURE;
	}

	ret = ring_buffer_init(&ring_buffer_rx, &can_buffer_rx[0], sizeof(can_buffer_rx), sizeof(can_t));
	if(0 != ret)
	{
		perror("\"ring buffer rx\"\n");
		return EXIT_FAILURE;
	}
	
	sem_init(&semaphore, 0, 1);

	ret = pthread_create(&can_rx_thread, NULL, can_listener, NULL);
	if(ret != 0)
	{
		perror("\"can_rx_thread\"");
	}

	ret = pthread_create(&can_tx_thread, NULL, can_transmitter, NULL);
	if(ret != 0)
	{
		perror("\"can_tx_thread\"");
	}
	
	return EXIT_SUCCESS;
}

uint8_t can_rx(uint8_t port, can_t *frame)
{
	uint8_t ret = 0;

	sem_wait(&semaphore);
	ret = ring_buffer_dequeue(&ring_buffer_rx, frame);
	sem_post(&semaphore);

	return !ret;
}

uint8_t can_tx(uint8_t port, can_t *frame)
{
	uint8_t ret = CAN_FRAME_OK;

	sem_wait(&semaphore);
	ring_buffer_queue(&ring_buffer_tx, frame);
	sem_post(&semaphore);

	return ret;
}