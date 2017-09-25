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
int main( int argc, char ** argv )
{
    int        error;
    uint32_t   numLines;
    uint32_t   numPoints;
    int16_t    SuperScanX;
    int16_t    SuperScanY;
    int16_t    inputX;
    int16_t    inputY;
    
    // Make sure user entered x & y coords
    if(argc < 3)
      {
	//	printf("Syntax: box_test <x> <y> <step> <count> <write-delay>\n");
		printf("Syntax: box_test <x> <y>\n");
	exit(EXIT_FAILURE);
      }

    // Get input data
    inputX = atoi(argv[1]);
    inputY = atoi(argv[2]);
    
    openlog("agsd", 0, LOG_USER);
    syslog(LOG_NOTICE, "LV2_SS_COORD_TEST");

    pConfigMaster = malloc(sizeof(struct lg_master));
    if (!pConfigMaster)
      {
	syslog(LOG_ERR, "LV_SS_COORD_TEST:  LG_MASTER_MALLOC Failed\n");
	closelog();
	exit(EXIT_FAILURE);
      }
    memset((char *)pConfigMaster, 0, sizeof(struct lg_master));
    pConfigMaster->fd_lv2 = open("/dev/lv2", O_RDWR);
    if (pConfigMaster->fd_lv2 < 0)
      {
	syslog(LOG_ERR,"LV_SS_COORD_TEST:  laser device %d not opened,errno %d", pConfigMaster->fd_lv2, errno);
	free(pConfigMaster);
	closelog();
	exit(EXIT_FAILURE);
      }
    syslog(LOG_NOTICE,"LV_SS_COORD_TEST:  laser device %d opened successfully", pConfigMaster->fd_lv2);
    do_write_dark_xydata(inputX,inputY);
    usleep(1000);
    error = FindSuperScanCoords(pConfigMaster, inputX, inputY, &SuperScanX, &SuperScanY, &numLines, &numPoints);
    if (error)
      {
	syslog(LOG_NOTICE, "LV_SS_COORD_TEST: Coords NOT FOUND");
	exit(EXIT_FAILURE);
      }
    if ((SuperScanX == 0xFFFF) &&  (SuperScanY == 0xFFFF))
      {
	syslog(LOG_NOTICE, "LV_SS_COORD_TEST: Coords NOT FOUND(XY=%x,%x)", SuperScanX, SuperScanY);
	return(-1);
      }
    if (numLines <= 0)
      {
	syslog(LOG_NOTICE, "LV_SS_COORD_TEST: invalid number of lines (%d)", numLines);
	return(-1);
      }
    syslog(LOG_NOTICE,"LV_FIND_SS_COORD_TEST: SuperScan XY=%x,%x; NumLines=%d",SuperScanX, SuperScanY, numLines);
    syslog(LOG_NOTICE,"LV_FIND_SS_COORD_TEST: Test complete for /dev/lv2, fd %d",pConfigMaster->fd_lv2);
    exit(EXIT_SUCCESS);
}
