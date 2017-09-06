/*************************************************************
 *                                                           *
 * Aligned Vision, Inc.      new laser device tests          *
 * Copyright 2015.                                           *
 *                                                           *
 * Test Name:  lv_box_tests                                  *
 * Command  :  "lv_box_tests"                                *
 * Input    :    startX, startY, step, count.                *
 * Description:  This test exercises the new kernel laser    *
 *               device driver.  It attempts to do a line    *
 *               along X axis, line along Y axis, then       *
 *               create a box along X & Y axis.              *
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
#include "BoardComm.h"
#include "Laser_If.h"

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
static void do_write_xydata(int16_t x, int16_t y)
{
    struct lv2_xypoints   xydata;

    xydata.xPoint = x;
    xydata.yPoint = y;
    lv_setpoints_lite(pConfigMaster, &xydata);
    return;
}
int main( int argc, char ** argv )
{
    int16_t                    inputX;
    int16_t                    inputY;
    int16_t                    currentX;
    int16_t                    currentY;
    uint16_t                   i;
    uint16_t                   step;
    uint16_t                   num_points;
    
    // Make sure user entered x & y coords
    if(argc < 5)
      {
	printf("Syntax: box_test <x> <y> <step> <numPoints>\n");
	exit(EXIT_FAILURE);
      }

    // Get input data
    inputX = atoi(argv[1]);
    inputY = atoi(argv[2]);
    step = atoi(argv[3]);
    num_points = atoi(argv[4]);
    
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

    // Traverse Y axis, direction=up (positive step)
    do_write_dark_xydata(inputX,inputY);
    usleep(100);
    while (1)
      {
	currentX = inputX;
	currentY = inputY;
	for (i = 0; i < num_points; i+=step)
	  {
	    currentY += step;
	    do_write_xydata(currentX, currentY);
	    //	    usleep(100);
	  }
	syslog(LOG_NOTICE, "Y-Axis-Up Line Test Complete: StartX=%x, StartY=%x, Step=%d\n", currentX,currentY, step);
	// currentX = inputX + (num_points * step), currentY=inputY
	currentX = inputX;
	currentY = inputY + (num_points * step);
	// Traverse right-side X, direction=right (positive step)
	for (i = 0; i < num_points; i+=step)
	  {
	    currentX += step;
	    do_write_xydata(currentX, currentY);
	    usleep(100);
	  }
	syslog(LOG_NOTICE, "X-Axis-Right Line Test Complete: StartX=%x, StartY=%x, Step=%d\n", currentX,currentY, step);
	// currentX = inputX + (num_points * step), currentY=inputY + (num_points * step)
	currentX = inputX + (num_points * step);
	currentY = inputY + (num_points * step);
	// Traverse Y, direction=down (negative step)
	for (i = 0; i < num_points; i+=step)
	  {
	    currentY -= step;
	    do_write_xydata(currentX, currentY);
	    usleep(100);
	  }
	syslog(LOG_NOTICE, "Y-Axis-Down Line Test Complete: StartX=%x, StartY=%x, Step=%d\n", currentX,currentY, ~step);
	// currentX = inputX + (num_points * step), currentY=inputY
	currentX = inputX + (num_points * step);
	currentY = inputY;
	// Traverse X, direction=left (negative step)
	for (i = 0; i < num_points; i+=step)
	  {
	    currentX -= step;
	    do_write_xydata(currentX, currentY);
	    usleep(100);
	  }
	syslog(LOG_NOTICE, "X-Axis-Left Line Test Complete:  StartX=%x, StartY=%x, Step=%d\n", inputX,inputY, ~step);
	usleep(1000);
      }
    exit(EXIT_SUCCESS);
}
