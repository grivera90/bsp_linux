/**
*******************************************************************************
* @file           : can_drv.h
* @brief          : Description of header file
* @author         : Gonzalo Rivera
* @date           : dd/mm/aaaa
*******************************************************************************
* @attention
*
* Copyright (c) <date> grivera. All rights reserved.
*
*/
#ifndef __CAN_DRV_H__
#define __CAN_DRV_H__
/******************************************************************************
        Includes
 ******************************************************************************/
#include <stdint.h>

/******************************************************************************
        Constants
 ******************************************************************************/
#define CAN_BUFFER_LENGHT 16

/******************************************************************************
        Data types
 ******************************************************************************/
typedef enum
{
	CAN_FRAME_OK = 0,
	CAN_FRAME_ERROR
} can_frame_sts_t;

typedef struct  __attribute__ ((__packed__))
{
	uint32_t id;
	uint8_t buf[8];
	uint8_t buf_len;
} can_t;
/******************************************************************************
        Public function prototypes
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function to initialize all can refered.
 *
 * @param uint8_t port: can port.
 *
 * @return 0: Ok, 1: Error.
 * */
int can_init(uint8_t port);

/**
 * @brief Function to receive a frame can from buffer.
 * 
 * @param uint8_t port: can port. 
 * @param can_t *frame: pointer to storage data received.
 *
 * @return 0: Ok, 1: Error.
 * */
uint8_t can_rx(uint8_t port, can_t *frame);

/**
 * @brief Function to transmit a frame can.
 *
 * @param uint8_t port: can port.
 * @param can_t *frame: pointer to data to transmit
 *
 * @return 0: Ok, 1: Error.
 * */
uint8_t can_tx(uint8_t port, can_t *frame);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // EOF __CAN_DRV_H__
