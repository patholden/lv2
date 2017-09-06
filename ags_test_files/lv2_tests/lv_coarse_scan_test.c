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
#include "Laser_If.h"
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
    int        error;
    int16_t    i;
    int16_t    foundX;
    int16_t    foundY;
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
    error = CoarseScan(pConfigMaster, inputX, inputY, &foundX, &foundY);
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
    syslog(LOG_NOTICE, "LV_BOX_TEST: CoarseScan Found target at %x,%x", foundX, foundY);
    // blink target location on & off 3 times
    for (i=0; i < 3; i++)
      {
	do_write_dark_xydata(foundX, foundY);
	sleep(1);
	do_write_xydata(foundX, foundY);
	sleep(1);
	do_write_dark_xydata(foundX, foundY);
	sleep(1);
	do_write_xydata(foundX, foundY);
	sleep(1);
      }
    
    syslog(LOG_NOTICE,"LV_BOX_TEST: CoarseScan Test complete for /dev/lv2, fd %d",pConfigMaster->fd_lv2);
    exit(EXIT_SUCCESS);
}
