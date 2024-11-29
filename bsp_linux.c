/**
*******************************************************************************
* @file           : bsp_linux.c
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

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/wait.h>
#include <stdbool.h>
#include <ctype.h>
#include <pthread.h>
#include <semaphore.h>

/* timer need */
#include <signal.h>           /* Definition of SIGEV_* constants */
#include <time.h>

#include "bsp.h"
#include "can_drv.h"
#include "conf_parser.h"
#define LOG_LOCAL_LEVEL LOG_INFO
#define LOG_CONFIG_TIMESTAMP LOG_TIMESTAMP
#include "log.h"
/******************************************************************************
    Defines and constants
******************************************************************************/
#define CMD_BUFFER_SIZE 128
/******************************************************************************
    Data types
******************************************************************************/

/******************************************************************************
    Local variables
******************************************************************************/
static const char *MODULE_NAME = "[BSP_LINUX]";
static int (*local_systick_handler)(void) = NULL;
static sem_t bsp_ticks_semaphore;
static pthread_t cmd_input_thread;
static char line[CMD_BUFFER_SIZE];
static char rx_pipe_buff[CMD_BUFFER_SIZE] = {0};
static bool read_usr_flag = true;
/******************************************************************************
    Local function prototypes
******************************************************************************/
/**
 * @brief Function to get tick of internal timer of a this module. 
 * 
 * @return uint32_t: ticks.
 */
static uint32_t get_ticks(void);

/**
 * @brief Function to config the internal timer to systick provide.
 *
 * @param uint16_t: period in milli seconds.
 * 
 * @return int: 0 to OK 1 for Error.
 */
static int timer_config(uint16_t msec);

/**
 * @brief Function to handle the callback timer.
 *
 * @param union sigval arg:
 * 
 * @return none.
 */
static void timer_handler(union sigval arg);

/**
 * @brief Task to read the pipe line.
 *
 * @param void *params:
 * 
 * @return void *: 
 */
static void *read_pipe(void *params);
/******************************************************************************
    Local function definitions
******************************************************************************/
static uint32_t get_ticks(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (1000 * ts.tv_sec + ts.tv_nsec / 1000000);
}

static int timer_config(uint16_t msec)
{
    timer_t timer_id;
    int status;
    struct itimerspec ts;
    struct sigevent se;
    long long nanosecs = msec * 1000000;

    se.sigev_notify = SIGEV_THREAD;
    se.sigev_value.sival_ptr = &timer_id;
    se.sigev_notify_function = timer_handler;
    se.sigev_notify_attributes = NULL;

    ts.it_value.tv_sec = nanosecs / 1000000000;
    ts.it_value.tv_nsec = nanosecs % 1000000000;
    ts.it_interval.tv_sec = ts.it_value.tv_sec;
    ts.it_interval.tv_nsec = ts.it_value.tv_nsec;

    status = timer_create(CLOCK_REALTIME, &se, &timer_id);
    if(status == -1)
    {
        perror("create timer\n");
        return EXIT_FAILURE;
    }
        

    status = timer_settime(timer_id, 0, &ts, 0);
    if(status == -1)
    {
        perror("set timer\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS; 
}

static void timer_handler(union sigval arg)
{
    if(NULL != local_systick_handler)
    {
        local_systick_handler();
    }
}

static void *read_pipe(void *params)
{
    int fd;
    int n = 0; 

    while(1)
    {
        fd = open("/tmp/pipe_tx", O_RDONLY);
        n = read(fd, line, sizeof(line));
        
        if(n > 0)
        {
            if(true == read_usr_flag)
            {
                read_usr_flag = false;
                memcpy(rx_pipe_buff, line, n);
            }
            
            memset(line, 0, sizeof(line));
        }

        close(fd);
        usleep(1000);
    }

    pthread_exit(NULL);
}

/******************************************************************************
    Public function definitions
******************************************************************************/
bsp_ret_t bsp_init(void)
{
	bsp_ret_t bsp_ret = BSP_OK;
    int pid = -1;
    int wstatus = -1;
    char *can;
    char *can_baudrate;

    sem_init(&bsp_ticks_semaphore, 0, 1);
    
    log_set_timestamp(bsp_get_systick);

    /* read a configuration file and init */
    // 1) read and parser bsp_config.conf file in "bsp_linux/" dir.
    // LOGI(MODULE_NAME, "reading bsp_config.conf");
    FILE* conf_file = fopen("can_configuration.conf", "r");
    if(NULL == conf_file)
    {
        perror("file open error\n");
    }
    conf_entries_t* entries = configuration_parse_file(conf_file);
    fclose(conf_file);
    can = configuration_read_value(entries, "can_port");
    can_baudrate = configuration_read_value(entries, "can_baud"); 

    // 2) run cmd to file chmod +x <my_script_can_set_up>.sh 
    pid = fork();

    if(0 == pid)
    {
        /* child process */
        if(0 != execlp("/bin/sh","/bin/sh", "-c", "chmod +x can_setup.sh",(char *)NULL))
        {
            perror("chmod +x can_setup.sh error\n");
        }
    }
    else
    {   
        /* parent process */
        wait(&wstatus); // espero a que el hijo termine. 
        printf("change permisess done\n");
    }

    // 3) run <my_script_can_set_up>.sh <param1: "can0" | "vcan0"> <param2: 250000>
    pid = fork();
    
    if(0 == pid)
    {
        /* child process */
        char cmd_str[64];
        sprintf(cmd_str, "./can_setup.sh %s %s", can, can_baudrate);
        if(0 != execlp("/bin/sh","/bin/sh", "-c", cmd_str, (char *)NULL))
        {
            perror("can_setup.sh error\n");
        }
    }
    else
    {
        /* parent process */
        wait(&wstatus);
        printf("can device config done\n");
    }
    
    // 4) initialize "bsp_can_init()" with selected param.
    if(0 == strcmp(can, "vcan0"))
    {
        can_init(1); // 1 = "vcan0"
    }
    else if(0 == strcmp(can, "can0"))
    {
        can_init(0); // 0 = "can0"
    }
    else
    {
        perror("can param error\n");
    }
    
    bsp_systick_init(1);
    LOGI(MODULE_NAME, "bsp init");

	return bsp_ret;
}

bsp_ret_t bsp_set_cb_systick(int (*cb_systick)(void))
{
    bsp_ret_t ret = BSP_OK;

    if(NULL !=  cb_systick)
    {
        local_systick_handler = cb_systick;
    }

    return ret;
}

bsp_ret_t bsp_systick_init(uint32_t period_ms)
{
    bsp_ret_t ret = BSP_OK;

    if(EXIT_SUCCESS != timer_config(period_ms))
    {
        perror("timer_config\n");    
    }

    return ret;
}

uint32_t bsp_get_systick(void)
{
    uint32_t ticks = 0;
    sem_wait(&bsp_ticks_semaphore);
    ticks = (uint32_t) get_ticks();
    sem_post(&bsp_ticks_semaphore);
    return ticks;
}

void bsp_led_test_write(bool state)
{

}

bsp_ret_t bsp_cmd_init(void)
{
    if(0 != pthread_create(&cmd_input_thread, NULL, read_pipe, NULL))
    {
        perror("read_usr_cmd error\n");
    }

    return BSP_OK;
}

char *bsp_read_cmd(void)
{
    return rx_pipe_buff;
}

bsp_ret_t bsp_write_cmd(char *s, int s_len)
{
    bsp_ret_t ret = BSP_OK;
    int fd = 0;

    if(NULL == s)
    {
        return BSP_ERROR;
    }

    fd = open("/tmp/pipe_rx", O_WRONLY);
    write(fd, s, s_len + 1);
    close(fd);

    return ret;
}

bsp_ret_t bsp_clear_cmd(void)
{
    memset(rx_pipe_buff, 0, sizeof(rx_pipe_buff));
    read_usr_flag = true;
    
    return BSP_OK;
}
