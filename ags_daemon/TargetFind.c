/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "TargetFind.h"
#include "Laser_lv2_If.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "parse_data.h"

// Static prototypes for local functions

static int isOutOfBounds(int16_t point, int16_t step, uint16_t count);
int CoarseScanFindMatch(struct lg_master *pLgMaster, uint32_t numPoints, int16_t startX, int16_t startY, int16_t *foundX, int16_t *foundY, uint32_t read_sense_sz);

//  Local functions
/****************************************************************
 *
 *  isOutOfBounds()
 *
 *  Description:   Checks to make sure points don't go off the
 *                 grid.
 *  Arguments:     <point> (x or y coord to test)
 *                 <step>  (value used to increment x or y by)
 *                 <numPoints> (number of points that will be used
 *                 in target find search)
 *
 *  Returns:       -1 if <point> is out of range in either direction.
 *                  0 if <point> is safe.
 *
 ***************************************************************/
int isOutOfBounds(int16_t point, int16_t step, uint16_t count)
{
    if ((point - (step * count)) <= kMinSigned)
      return(-1);
    if ((point + (step * count)) >= kMaxSigned)
      return(-1);
    return(0);
}

/****************************************************************
 *
 *  CoarseScanFindMatch()
 *
 *  Description:   Reads sense buffer at end of each box scan to
 *                 see if a target is found.  Criteria is 5 points
 *                 minimum.
 *  Arguments:     <pLgMaster> (pointer to master structure)
 *                 <numPoints> (number of write-sense operations done
 *                             during this search)
 *                 <startX> <startY>  (x,y pair that started search)
 *                 <read_sense_sz>  (size of sense read buffer) 
 *
 *  Returns:       -1 if no target is found.
 *                  0 if target found; <*foundX> <*foundY>
 *                          (x,y pair where target is found at)
 *
 ***************************************************************/
int CoarseScanFindMatch(struct lg_master *pLgMaster, uint32_t numPoints, int16_t startX, int16_t startY, int16_t *foundX, int16_t *foundY, uint32_t read_sense_sz)
{
    struct write_sense_cs_data    *pSenseFound;
    int        rc;
    uint32_t   low_index;
    uint16_t   i;
    uint8_t   found_count;

    *foundX = kMaxUnsigned;
    *foundY = kMaxUnsigned;

    if (read_sense_sz > MAX_TF_BUFFER)
      {
	syslog(LOG_ERR, "CoarseScanFindMatch: length %d too long for sensing, max=%ld", read_sense_sz, MAX_TF_BUFFER);
	return(-1);
      }
    // Sense Buffer is filled by driver
    pSenseFound = (struct write_sense_cs_data *)malloc(read_sense_sz);
    if (!pSenseFound)
      {
	syslog(LOG_ERR,"CoarseScanFindMatch: ERROR trying to malloc buffer");
	return(-1);
      }
    // Need to memset because typically the amount of sense data read in is much smaller than the actual
    // allocated size in the driver
    memset(pSenseFound, 0, read_sense_sz);
    rc = read(pLgMaster->fd_lv2, (uint8_t *)pSenseFound, read_sense_sz);
    if (rc < 0)
      {
	syslog(LOG_ERR,"CoarseScanFindMatch: ERROR reading from LV2 device #%d, errno=%x", pLgMaster->fd_lv2, errno);
	free((uint8_t *)pSenseFound);
	return(-1);
      }
    // Analyze data, look for at least 3 hits
    found_count = 0;
    low_index = 0;
    for (i = 0; i < numPoints; i++)
      {
	if (pSenseFound[i].sense_val <= COARSE_SCAN_THRESHOLD2)
	  {
	    syslog(LOG_NOTICE,"CoarseScanFindMatch: found good sense data[%d]=%x", i, pSenseFound[i].sense_val);
	    found_count++;
	    if (pSenseFound[i].sense_val < pSenseFound[low_index].sense_val)
	      low_index = i;
	  }
      }
    if (found_count < 5)
      {
	syslog(LOG_NOTICE,"CoarseScanFindMatch: NO sense data found, match-count = %d", found_count);
	free(pSenseFound);
	return(-1);
      }
    // Lowest sense value wins
    *foundX = startX + low_index;
    *foundY = startX + low_index;
    free(pSenseFound);
    return(0);
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
int CoarseScan(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
	       int16_t *foundX, int16_t *foundY)
{
    struct lv2_sense_info   sense_data;
    struct lv2_xypoints     xydata;
    int                     ret;
    uint32_t                box_loop_count;
    uint32_t                sense_read_sz;
    int16_t                 currentX=startX;
    int16_t                 currentY=startY;
    
    *foundX = kMaxUnsigned;
    *foundY = kMaxUnsigned;

    // Set up starting point
    box_loop_count = 0;
    
    // loop until out of grid points or target is found
    currentX = startX;
    currentY = startY;
    sense_data.numPoints = COARSE_SCAN_COUNT;
    
    while (box_loop_count != COARSE_SCAN_MAX_LOOPS)
      {
	sense_data.step = COARSE_SCAN_STEP;
	sense_data.numPoints += box_loop_count * 2;
	sense_data.point_delay = DEFAULT_SENSE_DELAY;
	sense_read_sz = sense_data.step * sense_data.numPoints * 4;
	sense_data.sense_buf_sz = sense_read_sz;
	// Move mirrors to starting XY in the dark on each loop to avoid ghost/tail
	currentX = currentX - COARSE_SCAN_STEP - ((box_loop_count * COARSE_SCAN_COUNT)/2 - 1);
	currentY = currentY - COARSE_SCAN_STEP - ((box_loop_count * COARSE_SCAN_COUNT)/2 - 1);
	syslog(LOG_NOTICE, "BOX_LOOP: START X,Y=%x,%x; step=%d, numPoints=%d, loop_count=%d",
	       currentX, currentY, sense_data.step, sense_data.numPoints, box_loop_count);
	xydata.xPoint = currentX; 
	xydata.yPoint = currentY; 
	lv_setpoints_dark(pLgMaster, (struct lv2_xypoints *)&xydata);
	usleep(250);

	sense_data.xData = currentX;
	sense_data.yData = currentY;
	if (isOutOfBounds(sense_data.xData, sense_data.step, sense_data.numPoints))
	  {
	    syslog(LOG_NOTICE, "BOX_LOOP: OUT-OF-BOUNDS step=%d, data=%x, loop_count=%d",
		   sense_data.step, sense_data.xData, sense_data.numPoints);
	    break;
	  }
	if (isOutOfBounds(sense_data.yData, sense_data.step, sense_data.numPoints))
	  {
	    syslog(LOG_NOTICE, "BOX_LOOP: OUT-OF-BOUNDS step=%d, data=%x, loop_count=%d",
		   sense_data.step, sense_data.yData, sense_data.numPoints);
	    break;
	  }

	syslog(LOG_NOTICE,"BOX_LOOP %d START: Sending currentX=%x,currentY=%x", box_loop_count, currentX, currentY);
	lv_box_sense_cmd(pLgMaster, (struct lv2_sense_info *)&sense_data);
    
	ret = CoarseScanFindMatch(pLgMaster, sense_data.numPoints, startX, startY, foundX, foundY, sense_read_sz);
	if (ret == 0)
	  {
	    syslog(LOG_NOTICE, "CoarseScan():  found x=%x; found y=%x",*foundX, *foundY);
	    return(0);
	  }
	// Adjust X & Y back to bottom left corner of box
	syslog(LOG_NOTICE,"BOX LOOP %d END: currentX=%x,currentY=%x", box_loop_count, currentX, currentY);
	box_loop_count++;
      }
	
    return(-1);
}

/****************************************************************
 *
 *  FindSuperScanCoords()
 *
 *  Description:   Sends command to LV2 device driver to perform a cross search
 *                 algorithm to find the startX, endX, and number of lines to
 *                 perform super-scan search on.
 *  Arguments:     <pLgMaster> (pointer to master structure)
 *                 <startX> <startY>  (x,y pair that start search)
 *
 *  Returns:       -1 if no target is found.
 *                  0 if target found; <*foundX> <*foundY>
 *                          (x,y pair where target is found at)
 *
 ***************************************************************/
int FindSuperScanCoords(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
		      int16_t *foundX, int16_t *foundY)
{
    struct lv2_sense_info   sense_data;
    struct lv2_xypoints     xydata;
    
    // Starts out similar to coarse-scan function.
    *foundX = kMaxUnsigned;
    *foundY = kMaxUnsigned;

    // numPoints not used for this type of search
    sense_data.step = FSSC_SCAN_STEP;
    sense_data.numPoints = 0;
   
    sense_data.xData = startX;
    sense_data.yData = startY;

    // First move mirrors to start x,y location
    xydata.xPoint = startX; 
    xydata.yPoint = startY; 
    lv_setpoints_dark(pLgMaster, (struct lv2_xypoints *)&xydata);
    usleep(250);

    lv_find_ss_coords_sense_cmd(pLgMaster, (struct lv2_sense_info *)&sense_data);
    
    return(0);
}

int ScanEnd(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
		      int16_t *foundX, int16_t *foundY)
{
    // FIXME---PAH---TBD
    // show targets, write out log files
    return(0);
}
/****************************************************************
 *
 *  SuperScan()
 *
 *  Description:   Sends command to LV2 device driver to perform a line search
 *                 algorithm to find target dimensions & saves to log.
 *  Arguments:     <pLgMaster> (pointer to master structure)
 *                 <startX> <startY>  (x,y pair that start search)
 *
 *  Returns:       -1 if no target is found.
 *                  0 if target found; <*foundX> <*foundY>
 *                          (x,y pair where target is found at)
 *
 *  Logs:           TBD logs will be saved for post-processing.
 *
 ***************************************************************/
int SuperScan(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
		      int16_t *foundX, int16_t *foundY)
{
    // FIXME---PAH---TBD
    return(0);
}

int FindTarget(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
	       int16_t *foundX, int16_t *foundY)
{
    struct   lv2_xypoints  xyPoints;
    int                    rc;

    rc = CoarseScan(pLgMaster, startX, startY, foundX, foundY);
    // Move mirrors to starting XY in the dark on each loop to avoid ghost/tail
    xyPoints.xPoint = startX; 
    xyPoints.yPoint = startY; 
    lv_setpoints_dark(pLgMaster, (struct lv2_xypoints *)&xyPoints);
    usleep(250);
    if (rc)
      return(rc);
    if ((*foundX == kMaxUnsigned) && (*foundY == kMaxUnsigned))
      return(-1);

    // Got a target
    startX = *foundX;
    startY = *foundY;
    xyPoints.xPoint = *foundX;
    xyPoints.yPoint = *foundY;
    syslog(LOG_NOTICE, "Found target at x=%x,y=%x", *foundX, *foundY);

    // Next phase, find endpoints for super-fine scan
    rc = FindSuperScanCoords(pLgMaster, startX, startY, foundX, foundY);
    if (rc)
      return(rc);
    startX = *foundX;
    startY = *foundY;
    rc = SuperScan(pLgMaster, startX, startY, foundX, foundY);
    if (rc)
      return(rc);
    startX = *foundX;
    startY = *foundY;
    rc = ScanEnd(pLgMaster, startX, startY, foundX, foundY);
    return(rc);
}
