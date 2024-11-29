/**
*******************************************************************************
* @file           : bsp_linux.h
* @brief          : Description of header file
* @author         : Gonzalo Rivera
* @date           : 29/03/2023
*******************************************************************************
* @attention
*
* Copyright (c) <date> grivera. All rights reserved.
*
*/
#ifndef __BSP_LINUX_H__
#define __BSP_LINUX_H__
/******************************************************************************
        Includes
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

/******************************************************************************
        Constants
 ******************************************************************************/
#define YIELD(x)                        usleep(1000)
#define FLASH_PAGE_SIZE                 (0x800)
/******************************************************************************
        Data types
 ******************************************************************************/
typedef enum
{
	BSP_OK,
	BSP_ERROR = -1
} bsp_ret_t;

/******************************************************************************
        Public function prototypes
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function to bsp init.
 *
 * @return bsp_ret_t. See \p bsp_ret_t in Data types section.
 */
bsp_ret_t bsp_init(void);

/**
 * @brief Function to set a callback to provide a tick of the system.
 *
 * @param int (*cb_systick)(void): function pointer.
 * 
 * @return bsp_ret_t. See \p bsp_ret_t in Data types section.
 */
bsp_ret_t bsp_set_cb_systick(int (*cb_systick)(void));

/**
 * @brief Function to init the system tick in milli seconds.
 *
 * @param uint32_t period_ms: period.
 * 
 * @return bsp_ret_t. See \p bsp_ret_t in Data types section.
 */
bsp_ret_t bsp_systick_init(uint32_t period_ms);

/**
 * @brief Function to get the current tick counter.
 * 
 * @return uint32_t: current tick.
 */
uint32_t bsp_get_systick(void);

/**
 * @brief Function test.
 *
 * @param bool state: state.
 * 
 * @return none.
 */
void bsp_led_test_write(bool state);

/**
 * @brief Function to cmd line init.
 *
 * @return bsp_ret_t. See \p bsp_ret_t in Data types section.
 */
bsp_ret_t bsp_cmd_init(void);

/**
 * @brief Function to read a cmd entry by user in CLI mode.
 * 
 * @return char *: pointer to cmd.
 */
char *bsp_read_cmd(void);

/**
 * @brief Function to write some to the CLI terminal.
 *
 * @param char *s: pointer to string to write.
 * 
 * @param int s_len: string length.
 * 
 * @return bsp_ret_t. See \p bsp_ret_t in Data types section.
 */
bsp_ret_t bsp_write_cmd(char *s, int s_len);

/**
 * @brief Function to clear internal buffer used by CLI.
 * 
 * @return bsp_ret_t. See \p bsp_ret_t in Data types section.
 */
bsp_ret_t bsp_clear_cmd(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // EOF __BSP_LINUX_H__