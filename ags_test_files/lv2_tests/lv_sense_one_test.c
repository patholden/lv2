/*************************************************************
 *                                                           *
 * Assembly Guidance Systems new laser device tests          *
 * Copyright 2015.                                           *
 *                                                           *
 * Test Name:  lv_box_tests                                  *
 * Command  :  "lv_box_tests"                                *
 * Input    :    startX, startY, step, count.                *
 * Description:  This test exercises the new kernel laser    *
 *               device driver.  It attempts to do a line    *
 *               along X axis, line along Y axis, then       *
 *               sense on X & Y lines.                       *
 *                                                           *
 ************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <stddef.h>
#include <errno.h>
#include <sys/io.h>
#include <math.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include <string.h>
#include <sys/time.h>
#include "BoardComm.h"
#include "Laser_lv2_If.h"

// Some people don't know how to use structs or to do abstraction layers,
// so I had to add the following globals below just to get things compiling
// for test code.

int gVideoCount = 0;
int gVideoCheck = 0;

struct lg_master *pConfigMaster=0;

static void do_write_dark_xydata(int16_t x, int16_t y)
{
    struct lv2_xypoints   xydata;

    xydata.xPoint = x;
    xydata.yPoint = y;
    lv_setpoints_dark(pConfigMaster, &xydata);
    return;
}
static void do_write_sense_one_x(int16_t point, uint32_t index)
{
    struct lv2_sense_one_info   sense_data;

    sense_data.data = point;
    sense_data.index = index;
    lv_sense_oneX_cmd(pConfigMaster, (struct lv2_sense_one_info *)&sense_data);
    return;
}    
static void do_write_sense_one_y(int16_t point, uint32_t index)
{
    struct lv2_sense_one_info   sense_data;

    sense_data.data = point;
    sense_data.index = index;
    lv_sense_oneY_cmd(pConfigMaster, (struct lv2_sense_one_info *)&sense_data);
    return;
}    
int main( int argc, char ** argv )
{
    struct timeval             start;
    struct timeval             end;
    double                     elapsedTime;
    uint8_t                    *pData;
    struct write_sense_cs_data    *pWriteSenseInput;
    int                        error;
    int16_t                    i;
    int16_t                    inputX;
    int16_t                    inputY;
    int16_t                    point;
    uint16_t                   step;
    uint16_t                   data_size;
    uint16_t                   numPoints;
    uint16_t                   op_delay;
    
    // Make sure user entered x & y coords
    if(argc < 6)
      {
	printf("Syntax: box_test <x> <y> <step> <numPoints> <sense-delay(us)>\n");
	exit(EXIT_FAILURE);
      }

    // Get input data
    inputX = atoi(argv[1]);
    inputY = atoi(argv[2]);
    step = atoi(argv[3]);
    numPoints = atoi(argv[4]);
    op_delay   = atoi(argv[5]);
    
    openlog("agsd", 0, LOG_USER);
    syslog(LOG_NOTICE, "CoarseScan Box Test");

    pConfigMaster = malloc(sizeof(struct lg_master));
    if (!pConfigMaster)
      {
	syslog(LOG_ERR, "LG_MASTER_MALLOC Failed\n");
	closelog();
	exit(EXIT_FAILURE);
      }
    memset((char *)pConfigMaster, 0, sizeof(struct lg_master));
    pConfigMaster->fd_lv2 = open("/dev/lv2", O_RDWR);
    if (pConfigMaster->fd_lv2 < 0)
      {
	syslog(LOG_ERR,"laser device %d not opened,errno %d", pConfigMaster->fd_lv2, errno);
	free(pConfigMaster);
	closelog();
	exit(EXIT_FAILURE);
      }
    data_size = numPoints  * sizeof(struct write_sense_cs_data);
    pData = malloc(data_size);
    if (!pData)
      {
	syslog(LOG_ERR,"ERROR trying to malloc buffer for reading sense data");
	exit(EXIT_FAILURE);
      }
    pWriteSenseInput = (struct write_sense_cs_data *)pData;

    do_write_dark_xydata(inputX,inputY);
    usleep(1000 * 1000);
    // Sense-X (loop here), driver does single point sense-operation
    for (i = 0; i < numPoints; i++)
      {
	point = inputX+ (i * step);
	gettimeofday(&start, NULL);
	do_write_sense_one_x(point, i);
	gettimeofday(&end, NULL);
	usleep(op_delay);
	elapsedTime = end.tv_usec - start.tv_usec;
	syslog(LOG_NOTICE, "SENSE_ONE_TEST:  write-senseX operation takes %f usec", elapsedTime);
      }
    do_write_dark_xydata(inputX,inputY);
    error = read(pConfigMaster->fd_lv2, pData, data_size);
    if (error < 0)
      {
	syslog(LOG_ERR,"\nError reading from LV2 device\n");
	free(pData);
	exit(EXIT_FAILURE);
      }
    // Display data
    syslog(LOG_NOTICE, "BOX_TEST:  sense_data for X (local loop):");
    for (i = 0; i < numPoints; i++)
      {
	syslog(LOG_NOTICE,"DEV-READ   xpoint[%d]=%x;sense_data=%x",
	       i,pWriteSenseInput[i].point, pWriteSenseInput[i].sense_val);
      }
    usleep(1000 * 1000);
    // Sense-Y (loop here), driver does single point sense-operation
    for (i = 0; i < numPoints; i++)
      {
	point = inputY+(i*step);
	gettimeofday(&start, NULL);
	do_write_sense_one_y(point, i);
	gettimeofday(&end, NULL);
	elapsedTime += end.tv_usec - start.tv_usec;
	syslog(LOG_NOTICE, "SENSE_ONE_TEST:  write-senseY operation takes %f usec", elapsedTime);
	usleep(op_delay);
      }
    do_write_dark_xydata(inputX,inputY);
    error = read(pConfigMaster->fd_lv2, pData, data_size);
    if (error < 0)
      {
	syslog(LOG_ERR,"\nError reading from LV2 device\n");
	free(pData);
	exit(EXIT_FAILURE);
      }
    // Display data
    syslog(LOG_NOTICE, "BOX_TEST:  sense_data for Y (local loop):");
    for (i = 0; i < numPoints; i++)
      {
	syslog(LOG_NOTICE,"DEV-READ   ypoint[%d]=%x;sense_data=%x",
	       i,pWriteSenseInput[i].point, pWriteSenseInput[i].sense_val);
      }
    free(pData);
    exit(EXIT_SUCCESS);
}
