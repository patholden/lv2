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
#include <sys/time.h>
#include <math.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include <string.h>
#include "BoardComm.h"
#include "Laser_lv2_If.h"

// Some people don't know how to use structs so following globals are here
// just to get things compiling for test code.
// CODE REVIEWS & DATA STRUCTURE COURSE WOULD GO A LONG WAY

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
static void do_write_sensedata_x(int16_t point, int8_t step, uint16_t loop_count, uint8_t op_delay)
{
    struct lv2_sense_info   sense_data;

    sense_data.xData = point;
    sense_data.step = step;
    sense_data.numPoints = loop_count;
    sense_data.point_delay = op_delay;

    lv_senseX_cmd(pConfigMaster, (struct lv2_sense_info *)&sense_data);
    return;
}
static void do_write_sensedata_y(int16_t point, int8_t step, uint16_t loop_count, uint8_t op_delay)
{
    struct lv2_sense_info   sense_data;

    sense_data.yData = point;
    sense_data.step = step;
    sense_data.numPoints = loop_count;
    sense_data.point_delay = op_delay;

    lv_senseY_cmd(pConfigMaster, (struct lv2_sense_info *)&sense_data);
    return;
}
int main( int argc, char ** argv )
{
    struct timeval             start;
    struct timeval             end;
    struct write_sense_cs_data   *pWriteSenseData;
    double                     elapsedTime;
    int                        error;
    int16_t                    i;
    int16_t                    inputX;
    int16_t                    inputY;
    uint16_t                   step;
    uint16_t                   data_size;
    uint16_t                   num_points;
    uint16_t                   op_delay;

    // Make sure user entered x & y coords
    if(argc < 6)
      {
	printf("Syntax: box_test <x> <y> <step> <numPoints> <usec-delay>\n");
	exit(EXIT_FAILURE);
      }

    // Get input data
    inputX = atoi(argv[1]);
    inputY = atoi(argv[2]);
    step = atoi(argv[3]);
    num_points = atoi(argv[4]);
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
    data_size = num_points  * sizeof(struct write_sense_cs_data);
    pWriteSenseData = (struct write_sense_cs_data *)malloc(data_size);
    if (!pWriteSenseData)
      syslog(LOG_ERR,"\nCoarseScanFindMatch: ERROR trying to malloc buffer\n");

    // Sense-X (loop in driver) unit test
    syslog(LOG_NOTICE, "Start SenseX-Driver-Loop unit test");
    do_write_dark_xydata(inputX,inputY);
    gettimeofday(&start, NULL);
    do_write_sensedata_x(inputX, step, num_points, op_delay);
    error = read(pConfigMaster->fd_lv2, (uint8_t *)pWriteSenseData, data_size);
    gettimeofday(&end, NULL);
    if (error < 0)
      {
	syslog(LOG_ERR,"\nError reading from LV2 device\n");
	free((uint8_t *)pWriteSenseData);
	exit(EXIT_FAILURE);
      }
    elapsedTime = end.tv_usec - start.tv_usec;
    syslog(LOG_NOTICE, "SENSE_LOOP_TEST:  write-senseX operation takes %f usec per point", elapsedTime/num_points);
    syslog(LOG_NOTICE, "SENSE_LOOP_TEST:  write-senseX operation takes %f usec, #points=%d", elapsedTime, num_points);
    // Display data
    syslog(LOG_NOTICE, "SenseX-Driver-Loop Read-Data:");
    for (i = 0; i < num_points; i++)
      {
	syslog(LOG_NOTICE,"   drv_xpoint[%d]=%x;sense_data=%x",
	       i,pWriteSenseData[i].point, pWriteSenseData[i].sense_val);
      }
    usleep(10 * 1000);
    do_write_dark_xydata(inputX,inputY);
    // Sense-Y (loop in driver) unit test
    syslog(LOG_NOTICE, "Start SenseY-Driver-Loop unit test");
    gettimeofday(&start, NULL);
    do_write_sensedata_y(inputY, step, num_points, op_delay);
    error = read(pConfigMaster->fd_lv2, (uint8_t *)pWriteSenseData, data_size);
    gettimeofday(&end, NULL);
    if (error < 0)
      {
	syslog(LOG_ERR,"\nError reading from LV2 device\n");
	free((uint8_t *)pWriteSenseData);
	exit(EXIT_FAILURE);
      }
    elapsedTime = end.tv_usec - start.tv_usec;
    syslog(LOG_NOTICE, "SENSE_LOOP_TEST:  write-senseY operation takes %f usec per point", elapsedTime/num_points);
    syslog(LOG_NOTICE, "SENSE_LOOP_TEST:  write-senseY operation takes %f usec, #points=%d", elapsedTime, num_points);
    // Display data
    syslog(LOG_NOTICE, "SenseY-Driver-Loop Read_Data:");
    for (i = 0; i < num_points; i++)
      {
	syslog(LOG_NOTICE,"   drv_ypoint[%d]=%x;sense_data=%x",
	       i,pWriteSenseData[i].point, pWriteSenseData[i].sense_val);
      }
    free((uint8_t *)pWriteSenseData);
    exit(EXIT_SUCCESS);
}
