/*************************************************************
 *                                                           *
 * Aligned Vision new laser device tests                     *
 * Copyright 2017.                                           *
 *                                                           *
 * Test Name:  lv_coarse_scan_test                           *
 * Command  :  "lv_coarse_scan_test"                         *
 * Input    :    startX, startY, step, count, and            *
 *               write-wait-delay are set by user.           *
 * Description:  This test exercises the new target-find     *
 *               coarse-scan function.                       *
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
#include "Laser_lv2_If.h"
#include "TargetFind.h"

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
/****************************************************************
 *
 *  CoarseScan()
 *
 *  Description:   Sends command to LV2 device driver to perform an expanding
 *                 box algorithm to search for a target around startX/startY
 *                 arguments.
 *  Arguments:     <pLgMaster> (pointer to master structure)
 *                 <startX> <startY>  (x,y pair that start search)
 *
 *  Returns:       -1 if no target is found.
 *                  0 if target found; <*foundX> <*foundY>
 *                          (x,y pair where target is found at)
 *
 ***************************************************************/
static int CoarseScanDark(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
		   int16_t *foundX, int16_t *foundY)
{
    struct lv2_sense_info   sense_data;
    struct lv2_xypoints     xydata;
    int                     ret;
    uint32_t                box_loop_count;
    int16_t                 currentX=startX;
    int16_t                 currentY=startY;
    
    *foundX = kMaxUnsigned;
    *foundY = kMaxUnsigned;

    // Set up starting point
    box_loop_count = 0;
    
    // Starting arguments to box algorithm
    sense_data.point_delay = DEFAULT_SENSE_DELAY; 
    sense_data.sense_delay = 0; 
    sense_data.step = COARSE_SCAN_STEP;
    sense_data.numPoints = COARSE_SCAN_NUM_POINTS;

    // Set up starting x,y points
    currentX = startX - sense_data.step;
    currentY = startY - sense_data.step;
    sense_data.xData = currentX;
    sense_data.yData = currentY;


    // Move mirrors to starting XY in the dark on each loop to avoid ghost/tail
    xydata.xPoint = startX; 
    xydata.yPoint = startY; 
    lv_setpoints_dark(pLgMaster, (struct lv2_xypoints *)&xydata);
    usleep(250);

    // loop until out of grid points or target is found
    while (box_loop_count < COARSE_SCAN_MAX_LOOPS)
      {
	if (isOutOfBounds(sense_data.xData, sense_data.step, sense_data.numPoints))
	  {
	    syslog(LOG_NOTICE, "BOX_LOOP %d: OUT-OF-BOUNDS step=%d, data=%x, numPoints=%d",
		   box_loop_count, sense_data.step, sense_data.xData, sense_data.numPoints);
	    break;
	  }
	if (isOutOfBounds(sense_data.yData, sense_data.step, sense_data.numPoints))
	  {
	    syslog(LOG_NOTICE, "BOX_LOOP %d: OUT-OF-BOUNDS step=%d, data=%x, numPoints=%d",
		   box_loop_count, sense_data.step, sense_data.yData, sense_data.numPoints);
	    break;
	  }

	syslog(LOG_NOTICE, "BOX_LOOP %d: X,Y=%x,%x; step=%d, numPoints=%d",
	       box_loop_count, currentX, currentY, sense_data.step, sense_data.numPoints);
	lv_box_dark_sense_cmd(pLgMaster, (struct lv2_sense_info *)&sense_data);
    
	ret = CoarseScanFindMatch(pLgMaster, sense_data.numPoints, startX, startY, sense_data.step, foundX, foundY);
	if (ret == 0)
	  {
	    syslog(LOG_NOTICE, "CoarseScan() END:  found x=%x; found y=%x; numLoops=%d",*foundX, *foundY, box_loop_count);
	    return(0);
	  }
	// Adjust X & Y back to bottom left corner of box
	sense_data.numPoints += ((box_loop_count * 2) + 2);
	box_loop_count++;
	currentX = startX - (COARSE_SCAN_STEP * (box_loop_count - 1));
	currentY = startY - (COARSE_SCAN_STEP * (box_loop_count - 1));
	sense_data.xData = currentX;
	sense_data.yData = currentY;
      }

    syslog(LOG_NOTICE, "CoarseScan() END:  NO target found, numLoops=%d", box_loop_count);
    return(-1);
}

int main( int argc, char ** argv )
{
    int        error;
    int16_t    foundX;
    int16_t    foundY;
    int16_t    inputX;
    int16_t    inputY;
    
    // Make sure user entered x & y coords
    if(argc < 3)
      {
	printf("Syntax: box_test <x> <y>\n");
	exit(EXIT_FAILURE);
      }

    // Get input data
    inputX = atoi(argv[1]);
    inputY = atoi(argv[2]);
    
    openlog("agsd", 0, LOG_USER);
    syslog(LOG_NOTICE, "LV2 CoarseScan Test");

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
    syslog(LOG_ERR,"laser device %d opened successfully", pConfigMaster->fd_lv2);
    do_write_dark_xydata(inputX,inputY);
    usleep(1000);

    foundX = 0xFFFF;
    foundY = 0xFFFF;
    error = CoarseScanDark(pConfigMaster, inputX, inputY, &foundX, &foundY);
    if (error)
      {
	syslog(LOG_NOTICE, "LV_BOX_TEST: CoarseScan target NOT FOUND");
	exit(EXIT_FAILURE);
      }
    if ((foundX == 0xFFFF) &&  (foundY == 0xFFFF))
      {
	syslog(LOG_NOTICE, "LV_BOX_TEST: CoarseScan target NOT FOUND");
	return(-1);
      }
    do_write_dark_xydata(inputX,inputY);
    syslog(LOG_NOTICE, "LV_BOX_TEST: CoarseScan Found target at %x,%x", foundX, foundY);
    exit(EXIT_SUCCESS);
}
