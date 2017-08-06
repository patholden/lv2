/*
static char rcsid[] = "$Id: BoardComm.c,v 1.36 2007/03/30 20:13:58 pickle Exp pickle $";
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <stddef.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/ioctl.h>
#include <linux/laser_api.h>
#include "AppCommon.h"
#include "BoardCommDefines.h"
#include "AppErrors.h"
#include "AppStrListIDs.h"
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "L3DTransform.h"
#include "3DTransform.h"
#include "SensorSearch.h"
#include "SensorRegistration.h"
#include "LaserInterface.h"
#include "Video.h"
#include "Init.h"
#include "ROI.h"
#include "Net.h"
#include "Hobbs.h"
#include "QuickCheckManager.h"



static time_t start_QC_timer;
static time_t end_QC_timer;
static struct lg_xydata *tmp_pattern=0;
static struct lg_xydata *out_pattern=0;
uint32_t  gRespondToWhom = kDoNotRespond;
static  int32_t     gQuickCheckCounterStart = -1;
static  uint32_t    cur_num_disp_points=0;

enum
{
  kInitializingBoardMsg = 1,
  kInitUnrecognizedMemModeMsg,
  kInitBoardNotFoundMsg,
  kInitOBCodeNotFound,
  kInitCannotLoadCode,
  kInitCannotResetBoard,
  kInitCannotFindSerialPack
};

enum
{
  kGSOffLineMsg = 1,
  kGSInitializingMsg,
  kGSOnLineMsg
};
 
static  void  DoRespond (struct lg_master *pLgMaster, struct k_header *pHdr);
static void RestoreBeamPosition(struct lg_master *pLgMaster);
static int doSetQCFlag(struct lg_master *pLgMaster, uint32_t qcflag);
static int doSensor(struct lg_master *pLgMaster, struct lg_move_data *pSensor);
static int doDarkMove(struct lg_master *pLgMaster, struct lg_move_data *pDarkMove);
static int doLiteMove(struct lg_master *pLgMaster, struct lg_move_data *pLiteMove);
static void SaveBeamPosition(struct lg_master *pLgMaster, char *data);
 
static int FillDispBuff(struct lg_master *pLgMaster, uint32_t *ptn_len)
{
    struct lg_xydata first_xypos;
    struct lg_xydata cur_xypos;
    struct lg_xydata last_xypos;
    struct lg_xydata *pXYout;
    struct lg_xydata *pXYtmp;
    struct cmd_rw    *cmd_buff;
    double           n,dx,dy,dsqr,dlen;
    int              rc;
    int32_t         i,count, Npoints;
    int16_t         k,xstep, ystep;

    // Set up buffer for laser-dev data  
    cmd_buff = (struct cmd_rw *)malloc(sizeof(struct cmd_rw));
    if (!cmd_buff)
      return(-1);

    // Start with a clean slate
    memset((char *)&cur_xypos, 0, sizeof(struct lg_xydata));
    memset((char *)&first_xypos, 0, sizeof(struct lg_xydata));
    memset((char *)&last_xypos, 0, sizeof(struct lg_xydata));
    memset((char *)cmd_buff, 0, sizeof(struct cmd_rw));

    // Send delta for first xy val
    pXYtmp = tmp_pattern;
    SaveBeamPosition(pLgMaster, (char *)tmp_pattern);


    move_dark(pLgMaster, pXYtmp);


    usleep(250);         // let mirrors settle
    // Start with first & last pointing to initial XY points
    first_xypos.xdata = last_xypos.xdata = pXYtmp->xdata;
    first_xypos.ydata = last_xypos.ydata = pXYtmp->ydata;
    first_xypos.ctrl_flags = last_xypos.ctrl_flags = pXYtmp->ctrl_flags;
    count = 0;
    Npoints = *ptn_len / sizeof(struct lg_xydata);
    for (i=0; i < Npoints; i++)
      {
	pXYtmp = (struct lg_xydata *)((char *)tmp_pattern + (sizeof(struct lg_xydata) * i));
	pXYout = (struct lg_xydata *)((char *)out_pattern + (sizeof(struct lg_xydata) * count));
	cur_xypos.xdata = pXYtmp->xdata;
	cur_xypos.ydata = pXYtmp->ydata;
	cur_xypos.ctrl_flags = pXYtmp->ctrl_flags;
	pXYout->xdata = last_xypos.xdata;
	pXYout->ydata = last_xypos.ydata;
	pXYout->ctrl_flags = last_xypos.ctrl_flags;
	count++;
	dx = (double)cur_xypos.xdata - (double)last_xypos.xdata;
	dy = (double)cur_xypos.ydata - (double)last_xypos.ydata;
	dsqr = dx*dx + dy*dy;
	dlen = sqrt(dsqr);
	if (dlen > pLgMaster->dmax)
	  {
	    n = ((dlen / pLgMaster->dmax) + 0.001);
	    n += 2.0;
	    xstep =  (int16_t)(dx / n);
	    ystep =  (int16_t)(dy / n);
	    for (k = 0; k < n; k++)
	      {
		pXYout = (struct lg_xydata *)((char *)out_pattern + (sizeof(struct lg_xydata) * count));
		pXYout->xdata = last_xypos.xdata + (k * xstep);
		pXYout->ydata = last_xypos.ydata + (k * ystep);
		pXYout->ctrl_flags = last_xypos.ctrl_flags;
		count++;
	      }
	  }
	last_xypos.xdata = cur_xypos.xdata;
	last_xypos.ydata = cur_xypos.ydata;
	last_xypos.ctrl_flags = cur_xypos.ctrl_flags;
      }
    // Set up last position
    pXYout = (struct lg_xydata *)((char *)out_pattern + (sizeof(struct lg_xydata) * count));
    pXYout->xdata = last_xypos.xdata;
    pXYout->ydata = last_xypos.ydata;
    pXYout->ctrl_flags = last_xypos.ctrl_flags;
    count++;
    dx = (double)first_xypos.xdata - (double)last_xypos.xdata;
    dy = (double)first_xypos.ydata - (double)last_xypos.ydata;
    dsqr = dx*dx + dy*dy;
    if (dsqr > (pLgMaster->dmax * pLgMaster->dmax))
      {
	n = (sqrt(dsqr / pLgMaster->dmax));
	n += 2.0;
	xstep =  (int16_t)(dx / n);
	ystep =  (int16_t)(dy / n);
	for (k = 0; k < n ; k++)
	  {
	    pXYout = (struct lg_xydata *)((char *)out_pattern + (sizeof(struct lg_xydata) * count));
	    pXYout->xdata = last_xypos.xdata + (k * xstep);
	    pXYout->ydata = last_xypos.ydata + (k * ystep);
	    pXYout->ctrl_flags = last_xypos.ctrl_flags;
	    count++;
	  }
      }
    *ptn_len = count * sizeof(struct lg_xydata);
    
    SaveBeamPosition(pLgMaster, (char *)out_pattern);
    // Don't send if too much data for laser-dev to handle
    if (*ptn_len > MAX_LG_BUFFER)
      {
	syslog(LOG_ERR,"%d length is too much data for laser device to handle",*ptn_len);
	return(-1);
      }
    memcpy((char *)&cmd_buff->xydata[0], (char *)out_pattern, *ptn_len);
    cmd_buff->base.hdr.cmd = CMDW_BUFFER;
    cmd_buff->base.hdr.length = *ptn_len;
syslog(LOG_ERR, "Write %d length, count %d ",*ptn_len, count);
    rc = write(pLgMaster->fd_laser, cmd_buff, sizeof(struct cmd_rw));
    if (rc < 0)
      {
	syslog(LOG_ERR, "Write command to laser device failed for %d length, rc=%x,errno=%x",*ptn_len,rc,errno);
	return(-1);
      }
    cur_num_disp_points = Npoints;
    free(cmd_buff);
    return(0);
}
void PostCmdDisplay(struct lg_master *pLgMaster, struct displayData *p_dispdata, int32_t do_response, uint32_t respondToWhom)
{
    struct k_header      pRespHdr;
    uint32_t             ptn_len;
    int                  rc;
    struct lg_xydata     *pXYtmp;
    
    // Prepare for response back to PC host
    memset((void *)&pRespHdr, 0, sizeof(struct k_header));

    // Check input data
    if (!p_dispdata)
      return;

    ptn_len = pLgMaster->gBuiltPattern;
    if (!p_dispdata->pattern || !ptn_len || (ptn_len > MAX_LG_BUFFER))
      {
	ROIoff(pLgMaster);
	initQCcounter(pLgMaster);
	// Only fail if trying to display more than driver can handle.
	// Can get here to display nothing too for some reason
	if (!p_dispdata->pattern || !ptn_len)
	  pRespHdr.status = RESPGOOD;
	else
	  pRespHdr.status = RESPFAIL;
	if ((do_response == SENDRESPONSE) ||  (pRespHdr.status == RESPFAIL))
	  DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
	return;
      }

    // Initialize all buffers to be used
    memset(tmp_pattern, 0, MAX_LG_BUFFER);
    memset(out_pattern, 0, MAX_LG_BUFFER);

    // Send initial commands to laser-dev
    ROIoff(pLgMaster);
    setROIlength(pLgMaster, 0);
    if ( gVideoCheck == 0 )
      initQCcounter(pLgMaster);

    

    // Get pattern data
    memcpy (tmp_pattern, (char *)p_dispdata->pattern, ptn_len);
    rc = FillDispBuff(pLgMaster, &ptn_len);
    if (rc)
      {
	syslog(LOG_ERR, "Unable to send display command to driver, rc %x,errno %x",rc,errno);
	pRespHdr.status = RESPFAIL;
	DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
	return;
      }

    // do a dark move to start of display
    pXYtmp = p_dispdata->pattern;
    move_dark(pLgMaster, pXYtmp);

    // Send command to start display
    if ( gVideoCheck == 0 )
      SaveAnglesForQuickCheck(p_dispdata, kRespondExtern );
    StartHobbs(pLgMaster);
    pLgMaster->gDisplayFlag = 1;
    rc = doDevDisplay(pLgMaster, ptn_len, 0);
    if (rc < 0)
      {
	syslog(LOG_ERR, "Unable to send display command to driver, rc %x,errno %x",rc,errno);
	pRespHdr.status = RESPFAIL;
	DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
	return;
      }
    else
      pRespHdr.status = RESPGOOD;
    if (pLgMaster->gOutOfRange.errtype1 || pLgMaster->gOutOfRange.errtype2)
      {
	pRespHdr.status = RESPFAIL;
	pRespHdr.errtype1 = pLgMaster->gOutOfRange.errtype1;
	pRespHdr.errtype2 = pLgMaster->gOutOfRange.errtype2;
      }
    if ((do_response == SENDRESPONSE) && (pRespHdr.status == RESPGOOD))
      DoRespond (pLgMaster, (struct k_header *)&pRespHdr);
    return;
}
void PostCmdEtherAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData)
{
    pLgMaster->gDisplayFlag = 0;
    SetLowBeam(pAngleData);
    move_lite(pLgMaster, pAngleData);
    doWriteDevPoints(pLgMaster, pAngleData);
    return;
}
void PostCmdGoAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData, uint32_t respondToWhom)
{
  struct k_header gResponseBuffer;

  memset((char *)&gResponseBuffer, 0, sizeof(struct k_header));
  
  gResponseBuffer.status = RESPGOOD;
  SetLowBeam(pAngleData);
  move_lite(pLgMaster, pAngleData);
  doWriteDevPoints(pLgMaster, pAngleData);
  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
    DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
  usleep(1000000);
  return;
}
void PostCmdDarkAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData)
{
    struct k_header gResponseBuffer;

    memset((char *)&gResponseBuffer, 0, sizeof(struct k_header));
    pLgMaster->gDisplayFlag = 0;
    move_dark(pLgMaster, pAngleData);
    doWriteDevPoints(pLgMaster, pAngleData);
    gResponseBuffer.status = RESPGOOD;
    DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
    return;
}
void PostCommand(struct lg_master *pLgMaster, uint32_t theCommand, char *data, uint32_t respondToWhom )
{
    struct k_header    gResponseBuffer;
    struct displayData *dispData;
    struct cmd_rw      *cmd_buff;
    uint32_t           ptn_len;
    int                rc;
    
    if (!pLgMaster)
      {
	syslog(LOG_ERR,"POSTCMD:  BAD POINTER %d",theCommand);
	return;
      }
    memset((char *)&gResponseBuffer, 0, sizeof(gResponseBuffer));

    //  assume display is off or being turned off
    pLgMaster->gDisplayFlag = 0;
    gRespondToWhom = respondToWhom;
    switch ( theCommand )
      {
      case kStop:
	// stop, slow down and turn off Ready LED
	doSTOPCMD(pLgMaster);
	g_FODflag = 0;
	gQuickCheckCounterStart = -1;
	gVideoCount = 0;
	gVideoCheck = 0;
	setROIlength(pLgMaster, 0);
	gResponseBuffer.status = RESPSTOPOK;
	EndHobbs(pLgMaster);
	
	if ( !(pLgMaster->gHeaderSpecialByte & 0x80 ) ) {
	  DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
	}
        break;
      case kDisplayVideoCheck:
	dispData = (struct displayData *)data;
	if (!dispData->pattern)
	  return;
	ptn_len = pLgMaster->gBuiltPattern;
	if (ptn_len > MAX_LG_BUFFER)
	  {
	    syslog(LOG_ERR,"\nKDISP-VCHK:  BAD LENGTH %d", ptn_len);
	    return;
	  }
	cmd_buff = (struct cmd_rw *)malloc(sizeof(struct cmd_rw));
	if (!cmd_buff)
	  return;
	memset((char *)cmd_buff, 0, sizeof(struct cmd_rw));
	ROIoff(pLgMaster);
	setROIlength(pLgMaster, 0);
	gQuickCheckCounterStart = -1;
	initQCcounter(pLgMaster);   /* never quick check */
	SaveBeamPosition(pLgMaster, (char *)dispData->pattern);
	memcpy((char *)&cmd_buff->xydata[0], (char *)dispData->pattern, ptn_len);
	cmd_buff->base.hdr.cmd = CMDW_BUFFER;
	cmd_buff->base.hdr.length = ptn_len;
	write( pLgMaster->fd_laser, cmd_buff, sizeof(struct cmd_rw));
	free(cmd_buff);
#ifdef AGS_DEBUG
	syslog(LOG_DEBUG,"\nPostCmdVideo: writing to device, len %d",ptn_len);
#endif
	StartHobbs(pLgMaster);
	pLgMaster->gDisplayFlag = 1;
	rc = doDevDisplay(pLgMaster, ptn_len, 0);
	if (rc < 0)
	  {
	    syslog(LOG_ERR,"Unable to send display command to driver, rc=%x,errno=%x", rc, errno);
	    gResponseBuffer.status1 = RESPFAIL;
	  }
      if (pLgMaster->gOutOfRange.errtype1 || pLgMaster->gOutOfRange.errtype2)
	{
	  gResponseBuffer.status1 = RESPFAIL;
	  gResponseBuffer.errtype1 = pLgMaster->gOutOfRange.errtype1;
	  gResponseBuffer.errtype2 = pLgMaster->gOutOfRange.errtype2;
	}
      else
	{
	  if (rc == 0)
	    gResponseBuffer.status = RESPGOOD;
	} 
      DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
      break;
    case kSegmentDisplay:
      DoRespond (pLgMaster, (struct k_header *)&gResponseBuffer);
      break;
    case kSearchForASensor:
      break;
    case kDisplayNoQuickCheck:
      if (!data)
	return;
      dispData = (struct displayData *)data;
      if (!dispData->pattern)
	return;
      ptn_len = pLgMaster->gBuiltPattern;
      if (ptn_len > MAX_LG_BUFFER)
	{
	  syslog(LOG_ERR,"\nKDISP-NOQCHK:  BAD LENGTH %d", ptn_len);
	  return;
	}
      cmd_buff = (struct cmd_rw *)malloc(sizeof(struct cmd_rw));
      if (!cmd_buff)
	return;
      
      memset((char *)cmd_buff, 0, sizeof(struct cmd_rw));
      ROIoff(pLgMaster);
      setROIlength(pLgMaster, 0);
      gQuickCheckCounterStart = -1;
      initQCcounter(pLgMaster);   /* never quick check */
      SaveBeamPosition(pLgMaster, (char *)dispData->pattern);
      // Write new xy data to driver
      memcpy((char *)&cmd_buff->xydata[0], (char *)dispData->pattern, ptn_len);
      cmd_buff->base.hdr.cmd = CMDW_BUFFER;
      cmd_buff->base.hdr.length = ptn_len;
      write(pLgMaster->fd_laser, cmd_buff, sizeof(struct cmd_rw));
      free(cmd_buff);
      // Now tell driver to start displaying new pattern
      StartHobbs(pLgMaster);
      pLgMaster->gDisplayFlag = 1;
      rc = doDevDisplay(pLgMaster, ptn_len, 0);
      if (rc < 0)
	{
	  syslog(LOG_ERR, "Unable to send display command to driver, rc=%x,errno=%x",rc, errno);
	  gResponseBuffer.status1 = RESPFAIL;
	}
      if (pLgMaster->gOutOfRange.errtype1 || pLgMaster->gOutOfRange.errtype2)
	{
	  gResponseBuffer.status1 = RESPFAIL;
	  gResponseBuffer.errtype1 = pLgMaster->gOutOfRange.errtype1;
	  gResponseBuffer.errtype2 = pLgMaster->gOutOfRange.errtype2;
	}
      else
	{
	  if (rc == 0)
	    gResponseBuffer.status = RESPGOOD;
	}
      DoRespond (pLgMaster, (struct k_header*)&gResponseBuffer);
      break;

      case kQuickCheckSensor:
      break;
    default:
      break;
    }
  return;
}

static void DoRespond (struct lg_master *pLgMaster, struct k_header *pHdr)
{
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;

  memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
  memcpy((char *)pResp, (char *)pHdr, sizeof(struct k_header));
  pResp->hdr.status = pHdr->status1;
  pResp->hdr.errtype1 = pHdr->errtype1;
  pResp->hdr.errtype2 = htons(pHdr->errtype2);
  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), gRespondToWhom);
  return;
}

void ReleaseBoard (struct lg_master *pLgMaster)
{
  if (pLgMaster->fd_laser > 0)
    close(pLgMaster->fd_laser);
  if (tmp_pattern)
    free(tmp_pattern);
  if (out_pattern)
    free(out_pattern);
  return;
}

int InitBoard (struct lg_master *pLgMaster)
{
    int        rc;
    uint32_t   fpga_version=0;

    pLgMaster->fd_laser = open("/dev/laser", O_RDWR);
    if (pLgMaster->fd_laser < 0)
      {
	syslog(LOG_ERR,"laser device %d not opened,errno %d", pLgMaster->fd_laser, errno);
	return(-1);
      }
    
#if 0
    pLgMaster->fd_lv2 = open("/dev/lv2", O_RDWR);
    if (pLgMaster->fd_lv2 < 0)
      {
	syslog(LOG_ERR,"lv2 device %d not opened,errno %d", pLgMaster->fd_lv2, errno);
	return(-1);
      }
#endif
    
    // Check FPGA version
    rc = ioctl(pLgMaster->fd_laser, LGGETFPGAVERSION, &fpga_version);
    if (rc < 0)
      {
	syslog(LOG_ERR,"Cannot get FPGA version information,errno %d", errno);
	return(-1);
      }
    if ((fpga_version & 0xffff) < CURRENT_FPGA_VERSION)
      {
	syslog(LOG_ERR,"Invalid FPGA version, Got %x, Expected at least %x", fpga_version, CURRENT_FPGA_VERSION);
	return(-1);
      }
      
    // Initialize driver & state machine to IDLE
    doLGSTOP(pLgMaster);
    
    // initialize display-pattern buffers
    tmp_pattern = (struct lg_xydata *)malloc(MAX_LG_BUFFER);
    if (!tmp_pattern)
      return(-2);
    out_pattern = (struct lg_xydata *)malloc(MAX_LG_BUFFER);
    if (!out_pattern)
      return(-3);
    syslog(LOG_NOTICE,"LaserVision laser device initialized, FPGA version is %x", fpga_version);
    return(0);
}

int DoLineSearch(struct lg_master *pLgMaster, struct lg_xydata *pSrchData,
		 struct lg_xydelta *pDeltaData, uint32_t nPoints)
{
    struct lg_move_data lg_sensor;
    char       *c_out;
    int        rc,j, count, blank;
    int        optic_status=0;
    uint32_t   time_out, num, buff_len;

    buff_len = nPoints * sizeof(int16_t);
    c_out = malloc(buff_len);
    if (!c_out)
      return(kStopWasDone);

    memset(c_out, 0, buff_len);
    // Move mirrors to XY in the dark to avoid ghost/tail
    move_dark(pLgMaster, pSrchData);
    usleep(250);   // Wait for driver to write & mirrors to settle.
    count = 10000;
    blank = 500;
    ioctl(pLgMaster->fd_laser, LGGETCTL2STAT, &optic_status);
    while (count--  && blank)
      {
	if (optic_status & PHTEDGEDTCT)
	  blank = 500;
	else
	  blank--;
      }
    // Send SENSOR command to driver
    memset((char *)&lg_sensor, 0, sizeof(struct lg_move_data));
    lg_sensor.poll_freq = pLgMaster->gSrchStpPeriod;
    lg_sensor.xy_curpt.xdata = pSrchData->xdata;
    lg_sensor.xy_curpt.ydata = pSrchData->ydata;
    lg_sensor.xy_curpt.ctrl_flags = BRIGHTBEAMISSET | BEAMONISSET | LASERENBISSET;
    lg_sensor.xy_delta.xdata = pDeltaData->xdata;
    lg_sensor.xy_delta.ydata = pDeltaData->ydata;
    lg_sensor.nPoints =  nPoints;
    lg_sensor.do_coarse = 1;
    rc = doSensor(pLgMaster, &lg_sensor);
    if (rc < 0)
      {
	syslog(LOG_ERR, "Unable to send sense command to driver, rc %x,errno %x",rc,errno);
	return(kStopWasDone);
      }

    // Need to give driver time to complete the SENSE sequence
    // read is being used to determine when the sequence finishes
    time_out = SENSOR_READ_FREQ * nPoints;
    usleep(time_out);
    memset((void *)c_out, 0, buff_len);
    num = 0;
    for (j=0; j < MAX_DOSENSE_RETRIES && (int)num <= 0; j++ )
      {
        num = read(pLgMaster->fd_laser, c_out, (nPoints * sizeof(int16_t)));

        usleep(500);
      }
    free(c_out);

    return(0);
}

int DoLevelSearch(struct lg_master *pLgMaster, struct lg_xydata *pSrchData,
		  struct lg_xydelta *pDeltaData, uint32_t nPoints, int16_t *c_out, uint32_t do_coarse)
{
    struct lg_move_data lg_sensor;
    size_t              num;
    uint32_t            senseThreshold;
    uint32_t            time_out;
    uint16_t            i, j, sense_count;
    int                 rc;

    // Move mirrors to XY in the dark to avoid ghost/tail
    move_dark(pLgMaster, pSrchData);
    usleep(1000);
    // usleep(250);
    // Set up SENSOR command info for driver
    memset((char *)&lg_sensor, 0, sizeof(struct lg_move_data));
    // lg_sensor.poll_freq = SENSOR_WRITE_FREQ;
    // lg_sensor.poll_freq = SENSOR_WRITE_FREQ;
 
    lg_sensor.poll_freq = pLgMaster->gSrchStpPeriod;  // default
    if ( pLgMaster->gSearchType == FINESEARCH )
        lg_sensor.poll_freq = pLgMaster->gFinePeriod;
    if ( pLgMaster->gSearchType == COARSESEARCH )
        lg_sensor.poll_freq = pLgMaster->gCoarsePeriod;

#ifdef ZDEBUG
    if ( pLgMaster->gSearchType == COARSESEARCH ) {
syslog(LOG_NOTICE, "level_search  period %d", lg_sensor.poll_freq );
    }
#endif

    lg_sensor.xy_curpt.xdata = pSrchData->xdata;
    lg_sensor.xy_curpt.ydata = pSrchData->ydata;
    lg_sensor.xy_curpt.ctrl_flags = BRIGHTBEAMISSET | BEAMONISSET | LASERENBISSET;
    lg_sensor.xy_delta.xdata = pDeltaData->xdata;
    lg_sensor.xy_delta.ydata = pDeltaData->ydata;
    lg_sensor.nPoints =  nPoints;
    lg_sensor.start_index =  0;   // start_index seems to be a problem
    lg_sensor.do_coarse = do_coarse;


    // Send SENSOR command to driver
    rc = doSensor(pLgMaster, &lg_sensor);
    if (rc < 0)
      {
	syslog(LOG_ERR, "Unable to send sense command to driver, rc %x,errno %x",rc,errno);
	return(kStopWasDone);
      }
    // Need to give driver time to complete the SENSE sequence
    // Should be only event handler WRITE+READ period * number of points
    // Adding fudge factor of times-2 to complete
    time_out = SENSOR_READ_FREQ * nPoints;
    usleep(time_out);
    memset((void *)c_out, 0, 3 * nPoints * sizeof(int16_t));
    j = 0;
    num = 0;
    time_out = 100;
    while ( j < MAX_DOSENSE_RETRIES && (int)num <= 0 ) 
      {
        j++;
	num = read(pLgMaster->fd_laser, (char *)c_out, (3 * nPoints * sizeof(int16_t)));
        usleep(time_out);
        time_out *= 2;
      }

    // set senseThreshold based on search type
    if ( pLgMaster->gSearchType == COARSESEARCH )
      senseThreshold = pLgMaster->gSenseThresholdCoarse;
    if ( pLgMaster->gSearchType == FINESEARCH )
      senseThreshold = pLgMaster->gSenseThresholdFine;
    if ( pLgMaster->gSearchType == SUPERSEARCH )
      senseThreshold = pLgMaster->gSenseThresholdSuperFine;

    syslog(LOG_DEBUG, "SearchType: %d, SenseThreshold: %d", pLgMaster->gSearchType, senseThreshold);

    // num = read(pLgMaster->fd_laser, (char *)c_out, (nPoints * sizeof(int16_t)));
    // hardware issue
    //  occasionally there is a bright spike
    //  at the start;  
    //  if the first value is past the threshold
    //  and there are more than 40 values
    //  reset the first three values
    // if ((c_out[0] < senseThreshold) && (num > 40))
    //   {
    //      for (i = 0; i < 3; i++)
    //        {
    //           c_out[3*i] = 1023;  // maximum for 10-bit DAC
    //        }
    //   }

    sense_count = 0;
    if (num > 0)
      {
	for (i = 0; i < nPoints; i++)
	  {
	    if ((c_out[3*i] > 0) && (c_out[3*i] < senseThreshold))
	      {
		sense_count++;
		if (sense_count > 1)
		  {
#ifdef ZDEBUG
		    syslog(LOG_NOTICE,"Driver found %d valid points", sense_count);
#endif
		    return(0);
		  }
	      }
	  }
      }
    return(kCoarseNotFound);
}
int SearchBeamOn(struct lg_master *pLgMaster)
{
  doSetSearchBeam(pLgMaster);
  return 0;
}

int SearchBeamOff(struct lg_master *pLgMaster)
{
  int rc=0;

  // FIXME---PAH---I don't think we need this next call
  // doClearSearchBeam(pLgMaster);
  // attempt to turn off beam
  // Just go back to idle state in driver state machine
  //  rc = doLGSTOP(pLgMaster);
  //  doClearSearchBeam(pLgMaster);
  return(rc);
}

int doWriteDevCmdNoData(struct lg_master *pLgMaster, uint32_t command)
{
  int    rc=0;

  rc = write(pLgMaster->fd_laser, (char *)&command, sizeof(uint32_t));
  if (rc < 0)
    syslog(LOG_ERR,"\nCMDW-NODATA: cmd %d, ERROR rc%d, errno %d\n", command, rc, errno);
  return(rc);
}
int doLGSTOP(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_STOP));
}
int doSTOPCMD(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_STOPCMD));
}
int doROIOff(struct lg_master *pLgMaster)
{
    usleep(250000);
    return(doWriteDevCmdNoData(pLgMaster, CMDW_ROIOFF));
}
int doDevDisplay(struct lg_master *pLgMaster, uint32_t ptn_len, uint32_t do_restart)
{
    struct lg_disp_data    lg_display;
    struct cmd_rw_dispdata *p_cmd_data;
    int                    rc=0;

    memset((char *)&lg_display, 0, sizeof(struct lg_disp_data));
    lg_display.is_restart = do_restart;
    lg_display.poll_freq = pLgMaster->gPeriod;
    lg_display.nPoints  = ptn_len/sizeof(struct lg_xydata);
    if (lg_display.nPoints > MAX_XYPOINTS)
      {
	syslog(LOG_ERR, "Unable to send display command to driver, %d too mand points",lg_display.nPoints);
	return(-1);
      }
    p_cmd_data = (struct cmd_rw_dispdata *)malloc(sizeof(struct cmd_rw_dispdata));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->hdr.cmd = CMDW_DISPLAY;
  p_cmd_data->hdr.length = sizeof(struct lg_disp_data);
   // poll_freq is the first member of a lg_disp_data struct
  memcpy((char *)&p_cmd_data->dispdata.poll_freq, (char *)&lg_display.poll_freq, sizeof(struct lg_disp_data));
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_dispdata));
  if (rc < 0)
    syslog(LOG_ERR,"\nCMDW-DISPLAY: ERROR rc %d, errno %d\n", rc, errno);
  free(p_cmd_data);
  return(rc);
}
int doStartPulse(struct lg_master *pLgMaster, struct lg_pulse_data *lg_pulsedata)
{
  int    rc=0;
  struct cmd_rw_pulsedata *p_cmd_data;

  if (!lg_pulsedata)
    return(-1);

  p_cmd_data = (struct cmd_rw_pulsedata *)malloc(sizeof(struct cmd_rw_pulsedata));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->hdr.cmd = CMDW_STARTPULSE;
  p_cmd_data->hdr.length = sizeof(struct lg_disp_data);
  memcpy((char *)&p_cmd_data->pulsedata.xy_curpt, (char *)&lg_pulsedata->xy_curpt, sizeof(struct lg_pulse_data));
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_pulsedata));
  if (rc < 0)
    syslog(LOG_ERR,"\nCMDW-STARTPULSE: ERROR rc %d, errno %d\n", rc, errno);
  free(p_cmd_data);
  return(rc);
}
int doSetSearchBeam(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_SEARCHBEAMON));
}
int doClearSearchBeam(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_SEARCHBEAMON));
}
int doSetReadyLED(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_READYLEDON));
}
int doClearReadyLED(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_READYLEDOFF));
}
void doClearLinkLED(struct lg_master *pLgMaster)
{
  doWriteDevCmdNoData(pLgMaster, CMDW_LINKLEDOFF);
  return;
}
void doSetLinkLED(struct lg_master *pLgMaster)
{
  doWriteDevCmdNoData(pLgMaster, CMDW_LINKLEDON);
  return;
}
int doClearShutterENB(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_CLEARSHUTENB));
}
int doSetShutterENB(struct lg_master *pLgMaster)
{
  return(doWriteDevCmdNoData(pLgMaster, CMDW_SETSHUTENB));
}
int doWriteDevCmd32(struct lg_master *pLgMaster, uint32_t command, uint32_t write_val)
{
  int    rc=0;
  struct cmd_rw_base *p_cmd_data;

  p_cmd_data = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->hdr.cmd = command;
  p_cmd_data->cmd_data.dat32.val32 = write_val;
  p_cmd_data->hdr.length = sizeof(uint32_t);
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_base));
  if (rc < 0)
    syslog(LOG_ERR,"\nCMDW-VAL32: ERROR cmd %d, val %d, rc %d, errno %d\n", command, write_val, rc, errno);
  free(p_cmd_data);
  return(rc);
}
int doWriteDevDelta(struct lg_master *pLgMaster, struct lg_xydelta *pDelta)
{
  int    rc=0;
  struct cmd_rw_base *p_cmd_data;

  p_cmd_data = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->hdr.cmd = CMDW_SETDELTA;
  p_cmd_data->hdr.length = sizeof(struct lg_xydata);
  p_cmd_data->cmd_data.xydelta.xdata = pDelta->xdata;
  p_cmd_data->cmd_data.xydelta.ydata = pDelta->ydata;
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_base));
  if (rc < 0)
    syslog(LOG_ERR,"\nCMDW-SETDELTA: ERROR xval %d, yval %d, rc %d, errno %d\n", pDelta->xdata, pDelta->ydata, rc, errno);
  free(p_cmd_data);
  return(rc);
}
int doWriteDevPoints(struct lg_master *pLgMaster, struct lg_xydata *pPoints)
{
  int    rc=0;
  struct cmd_rw_base *p_cmd_data;

  p_cmd_data = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
  if (!p_cmd_data)
    return(-1);
  
  p_cmd_data->hdr.cmd = CMDW_GOANGLE;
  p_cmd_data->hdr.length = sizeof(struct lg_xydata);
  p_cmd_data->cmd_data.xydata.ctrl_flags = pPoints->ctrl_flags;
  p_cmd_data->cmd_data.xydata.xdata = pPoints->xdata;
  p_cmd_data->cmd_data.xydata.ydata = pPoints->ydata;
  rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_base));
  if (rc < 0)
    syslog(LOG_ERR,"\nCMDW-GOANGLE: ERROR xval %d, yval %d, rc %d, errno %d\n", pPoints->xdata, pPoints->ydata, rc, errno);
  free(p_cmd_data);
  return(rc);
}
int doWriteCmdMove(struct lg_master *pLgMaster, struct lg_move_data *pMove, uint32_t command)
{
    int    rc=0;
    struct cmd_rw_movedata *p_cmd_data;

    if (!pMove)
      return(-1);

    p_cmd_data = (struct cmd_rw_movedata *)malloc(sizeof(struct cmd_rw_movedata));
    if (!p_cmd_data)
      return(-1);

    p_cmd_data->hdr.cmd = command;
    p_cmd_data->hdr.length = sizeof(struct lg_move_data);
    p_cmd_data->movedata.xy_curpt.xdata = pMove->xy_curpt.xdata;
    p_cmd_data->movedata.xy_curpt.ydata = pMove->xy_curpt.ydata;
    p_cmd_data->movedata.xy_curpt.ctrl_flags = pMove->xy_curpt.ctrl_flags;
    p_cmd_data->movedata.xy_delta.xdata = pMove->xy_delta.xdata;
    p_cmd_data->movedata.xy_delta.ydata = pMove->xy_delta.ydata;
    p_cmd_data->movedata.poll_freq = pMove->poll_freq;
    p_cmd_data->movedata.nPoints = pMove->nPoints;
    p_cmd_data->movedata.do_coarse = pMove->do_coarse;
    rc = write(pLgMaster->fd_laser, (char *)p_cmd_data, sizeof(struct cmd_rw_movedata));
    if (rc < 0)
      syslog(LOG_ERR,"\nCMDW-MOVEDATA: ERROR cmd %d, rc %d, errno %d\n", command, rc, errno);
    free(p_cmd_data);
    return(rc);
}
static int doSensor(struct lg_master *pLgMaster, struct lg_move_data *pMove)
{
  return(doWriteCmdMove(pLgMaster, pMove, CMDW_DOSENSOR));
}
static int doDarkMove(struct lg_master *pLgMaster, struct lg_move_data *pMove)
{
  return(doWriteCmdMove(pLgMaster, pMove, CMDW_DODARKMOVE));
}
static int doLiteMove(struct lg_master *pLgMaster, struct lg_move_data *pMove)
{
  return(doWriteCmdMove(pLgMaster, pMove, CMDW_DOLITEMOVE));
}
void SetHighBeam(struct lg_xydata *pDevXYData)
{
  // Set beam HIGH, set laser ENABLED
  pDevXYData->ctrl_flags = BRIGHTBEAMISSET | BEAMONISSET | LASERENBISSET;
  return;
}
void SetLowBeam(struct lg_xydata *pDevXYData)
{
    // Set beam LOW, keep laser enabled
    pDevXYData->ctrl_flags = BEAMONISSET | LASERENBISSET;
    return;
}
void SetDarkBeam(struct lg_xydata *pDevXYData)
{
    // No beam, keep laser enabled
    pDevXYData->ctrl_flags = LASERENBISSET;
    return;
}
static int ApplyQCCounter(struct lg_master *pLgMaster, uint32_t qcCount)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETQCCOUNTER, qcCount));
}
static int doSetQCFlag(struct lg_master *pLgMaster, uint32_t qcflag)
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETQCFLAG, qcflag));
}
int initQCcounter(struct lg_master *pLgMaster)
{
  gQuickCheckCounterStart = pLgMaster->gQCcount;
  return(ApplyQCCounter(pLgMaster, gQuickCheckCounterStart));
}
int resetQCcounter(struct lg_master *pLgMaster)
{
  return(ApplyQCCounter(pLgMaster, gQuickCheckCounterStart));
}
int SetQCcounter(struct lg_master *pLgMaster, int count)
{
  int   itest;
  
  itest = ApplyQCCounter(pLgMaster, count);
#if defined(KITDEBUG)
  syslog(LOG_NOTICE, "SetCQ qcCounter %d\n", count);
#endif
  if (itest)
    return(itest);
  return 0;
}
int stopQCcounter(struct lg_master *pLgMaster)
{
  int       itest;
  uint32_t  qcflag=0;
  uint32_t  qc_value = -1;
  
  itest = (ApplyQCCounter(pLgMaster, qc_value));
  if (itest)
    return itest;

  itest = doSetQCFlag(pLgMaster, qcflag);
  if (itest)
    return itest;
  return 0;
}
int32_t GetQCflag(struct lg_master *pLgMaster)
{
  int itest;
  uint32_t qcFlag=0;
  
  itest = ioctl( pLgMaster->fd_laser, LGGETQCFLAG, &qcFlag);
  if (itest)
    return(-1);
	 
  return(qcFlag);
}

int32_t InitQCtimer(void)
{
  start_QC_timer = time(NULL);
  end_QC_timer = time(NULL);

  return( 0 );
}

int32_t GetQCtimer( void )
{
            int32_t diff;

            end_QC_timer = time(NULL);

            diff = (int32_t)(end_QC_timer - start_QC_timer);
            return( diff );
}

int32_t GetQCcounter(struct lg_master *pLgMaster)
{
  int itest;
  uint32_t  qcCounter=0;

  itest = ioctl( pLgMaster->fd_laser, LGGETQCCOUNTER, &qcCounter);
  if (itest)
    return(-1);

  return(qcCounter);
}

void ResumeDisplay(struct lg_master *pLgMaster)
{
    int                   rc;
    
#ifdef AGS_DEBUG
    syslog(LOG_DEBUG, "Attempting to resume display");
#endif
    RestoreBeamPosition(pLgMaster);
    StartHobbs(pLgMaster);
    pLgMaster->gDisplayFlag = 1;
    rc = doDevDisplay(pLgMaster, 0, 1);
    if (rc)
      syslog(LOG_ERR, "Unable to send display command to driver, rc=%x,errno=%x",rc,errno);
  return;
}

/*  intended to save initial position */
static void SaveBeamPosition(struct lg_master *pLgMaster, char *data)
{
  memcpy((char *)&pLgMaster->gSaveXY, data, sizeof(struct lg_xydata));
  return;
}

void RestoreBeamPosition(struct lg_master *pLgMaster)
{

    move_dark(pLgMaster, (struct lg_xydata *)&pLgMaster->gSaveXY);
#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"\nRESTOREBEAM: x=%d,y=%d",pLgMaster->gSaveXY.xdata, pLgMaster->gSaveXY.ydata);
#endif
    doWriteDevPoints(pLgMaster, (struct lg_xydata *)&pLgMaster->gSaveXY);
    usleep(10000);
    return;
}

void GoToRaw(struct lg_master *pLgMaster, struct lg_xydata *pRawData)
{
  move_dark(pLgMaster, pRawData);
#ifdef AGS_DEBUG
  syslog(LOG_DEBUG,"\nGOTORAW: x=%d,y=%d",pRawData->xdata, pRawData->ydata);
#endif
  doWriteDevPoints(pLgMaster, pRawData);
  return;
}

void GoToPulse(struct lg_master *pLgMaster, struct lg_xydata *pPulseData,
	       uint32_t pulseoffvalue, uint32_t pulseonvalue)
{
    struct lg_pulse_data   lg_gopulse;

    memset((char *)&lg_gopulse, 0, sizeof(lg_gopulse));
    move_lite(pLgMaster, pPulseData);
    usleep(250);     // wait for driver to settle
    lg_gopulse.poll_freq = KETIMER_50U;  // for now, hard-code to 50U
    lg_gopulse.xy_curpt.xdata = pPulseData->xdata;
    lg_gopulse.xy_curpt.ydata = pPulseData->ydata;
    lg_gopulse.xy_curpt.ctrl_flags = BRIGHTBEAMISSET | BEAMONISSET | LASERENBISSET;
    lg_gopulse.off_val = pulseoffvalue;
    lg_gopulse.on_val = pulseonvalue;
    doStartPulse(pLgMaster, &lg_gopulse);
    return;
}
void StopPulse (struct lg_master *pLgMaster)
{
    doLGSTOP(pLgMaster);
}

int ROIoff (struct lg_master *pLgMaster)
{
  int rc=0;
  
  SearchBeamOff(pLgMaster);
  rc = doROIOff(pLgMaster);
  return(rc);
}
int setROIlength(struct lg_master *pLgMaster, int32_t half_pattern )
{
  return(doWriteDevCmd32(pLgMaster, CMDW_SETROI, half_pattern));
}
void FlashLed(struct lg_master *pLgMaster, int numFlash )
{
  int i;

  for (i = 0; i < numFlash; i++)
    {
      usleep(250000);
      doClearReadyLED(pLgMaster);
      usleep(250000);
      doSetReadyLED(pLgMaster);
    }
  return;
}
void SlowDownAndStop(struct lg_master *pLgMaster)
{
  // stop display
  // check and update hobbs meters, if necessary
  doLGSTOP(pLgMaster);
  EndHobbs(pLgMaster);
  return;
}

int JustDoDisplay(struct lg_master *pLgMaster, char *wr_ptr, int pattern_len)
{ 
    uint32_t ptn_len;
    int      rc=0;
  
    if (!wr_ptr)
      return(-1);

    // Initialize all buffers about to be used
    memset(tmp_pattern, 0, MAX_LG_BUFFER);
    memset(out_pattern, 0, MAX_LG_BUFFER);

    // Set pattern length established in command arguments
    ptn_len = pattern_len;
    if (ptn_len > MAX_LG_BUFFER)
      {
	syslog(LOG_ERR,"Unable to send display command, %d bad length!",ptn_len);
	return(-1);
      }
    // Send new settings to laser-dev
    ROIoff(pLgMaster);
    setROIlength(pLgMaster, 0);
    stopQCcounter(pLgMaster);   /* never quick check */
    // Set the data up for driver
    memcpy((char *)tmp_pattern, wr_ptr, ptn_len);
    rc = FillDispBuff(pLgMaster, &ptn_len);
    if (rc)
      return(rc);
    // Send display command to driver
    StartHobbs(pLgMaster);
    pLgMaster->gDisplayFlag = 1;
    rc = doDevDisplay(pLgMaster, ptn_len, 0);
    if (rc < 0)
      syslog(LOG_ERR, "Unable to send display command, rc=%x,errno=%x",rc,errno);
    return(rc);
}

int move_lite(struct lg_master *pLgMaster, struct lg_xydata *pNewData)
{
    struct lg_move_data lg_litemove;
    struct lg_xydata    xydata;
    double              n, dx, dy, dsqr, dlen;
    int                 rc;
    double              max_lite;  // try to vary maximum lit step size

    memset((char *)&lg_litemove, 0, sizeof(struct lg_move_data));
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    lg_litemove.poll_freq = KETIMER_30U;   // for now, hard-code to 30U

    // make sure the projector is IDLE
    // before getting current scanner angles
    doLGSTOP(pLgMaster);
    // Get current xy spot to see if need to travel to move to new point
    ioctl(pLgMaster->fd_laser, LGGETANGLE, &xydata);

          // need to start at current point
    lg_litemove.xy_curpt.xdata = xydata.xdata;
    lg_litemove.xy_curpt.ydata = xydata.ydata;
    lg_litemove.xy_curpt.ctrl_flags = pNewData->ctrl_flags;

    lg_litemove.nPoints = 1; // Going to write at least 1 point

    // Figure out delta point
    dx = (double)pNewData->xdata - (double)xydata.xdata;
    dy = (double)pNewData->ydata - (double)xydata.ydata;
    dsqr = dx*dx + dy*dy;
    dlen = sqrt(dsqr);
      // if start and end are more than a degree apart
      // take intermediate steps
      // otherwise, go straight to end point

    max_lite = 0.1 * pLgMaster->dmax;
    if (dlen > (max_lite))
      {
	n = dlen / max_lite;
	n += 5;
	lg_litemove.xy_delta.xdata =  (int16_t)(dx / n);
	lg_litemove.xy_delta.ydata =  (int16_t)(dy / n);
	lg_litemove.nPoints =  (uint32_t)n;
      }
    else
      {
        lg_litemove.xy_curpt.xdata = pNewData->xdata;
        lg_litemove.xy_curpt.ydata = pNewData->ydata;
        lg_litemove.xy_curpt.ctrl_flags = pNewData->ctrl_flags;
      }
    rc = doLiteMove(pLgMaster, &lg_litemove);

      // need to sleep after the move
      // if there are intermediate points
    if (dlen > max_lite)
      {
         usleep(2 * (uint32_t)n * 50  + 2000);
      }
    if (rc < 0)
      syslog(LOG_ERR, "Unable to send litemove command to driver %x,errno %x",rc,errno);
    /* end of lite move */
  return(0);
}

void move_one_dark(struct lg_master *pLgMaster, struct lg_xydata *pDarkData)
{
    pDarkData->ctrl_flags = 0;
    doWriteDevPoints(pLgMaster, pDarkData);
    return;
}

int move_dark(struct lg_master *pLgMaster, struct lg_xydata *pNewData)
{
    struct lg_move_data lg_darkmove;
    struct lg_xydata    xydata;
    double              n, dx, dy, dsqr, dlen;
    int                 rc;
    useconds_t          dark_wait;

    // before trying to LGGETANGLE,
    // make sure that the projector is not displaying
    doLGSTOP( pLgMaster );
    
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&lg_darkmove, 0, sizeof(struct lg_move_data));

    lg_darkmove.poll_freq = KETIMER_50U;   // for now, hard-code to 50U
    lg_darkmove.nPoints = 1; // Going to write at least 1 point
    // Get current xy spot and calculate delta to move to new point.
    ioctl( pLgMaster->fd_laser, LGGETANGLE, &xydata);
       
    lg_darkmove.xy_curpt.xdata = xydata.xdata;
    lg_darkmove.xy_curpt.ydata = xydata.ydata;
    lg_darkmove.xy_curpt.ctrl_flags = 0;

    dx = (double)pNewData->xdata - (double)xydata.xdata;
    dy = (double)pNewData->ydata - (double)xydata.ydata;
    dsqr = dx*dx + dy*dy;
    dlen = sqrt(dsqr);
      // if start and end are more than a degree apart
      // take intermediate steps
      // otherwise, go straight to end point
    if ( dlen > pLgMaster->dmax )
     {
        n = dlen / pLgMaster->dmax;
        n += 2;
        lg_darkmove.xy_delta.xdata =  (int16_t)(dx / n);
        lg_darkmove.xy_delta.ydata =  (int16_t)(dy / n);
        lg_darkmove.nPoints = n;
     }
    else
     {
        lg_darkmove.xy_curpt.xdata = pNewData->xdata;
        lg_darkmove.xy_curpt.ydata = pNewData->ydata;
        lg_darkmove.xy_curpt.ctrl_flags = 0;
     }
    rc = doDarkMove(pLgMaster, &lg_darkmove);
    if (rc < 0)
      syslog(LOG_ERR, "Unable to send darkmove command to driver, rc %x,errno %x",rc,errno);

      // need to wait for darkmove to complete
    if ( dlen > pLgMaster->dmax )
     {
       dark_wait = (100 * (int)n * KETIMER_50U ) + 20000;
       usleep( dark_wait );
     }
    else
     {
       dark_wait = 2000;
       usleep( 2000 );
     }

  /* end of dark move */
  return(0);
}

int CDRHflag(struct lg_master *pLgMaster)
{
  int  optic_status=0;

  ioctl(pLgMaster->fd_laser, LGGETCTL2STAT, &optic_status);
  if (optic_status & CDRHBITMASK)
    { 
      doSetShutterENB(pLgMaster);
      return(0);
    }

  syslog(LOG_NOTICE, "\nCDRHFlag clear, try to CLOSE shutter\n");
  doClearShutterENB(pLgMaster);
  return(1); 
}
double ArrayToDouble(double inp_data)
{
   union
   {
      double ull;
      int8_t  c[8];
   } x;

   int8_t c = 0;
   x.ull = inp_data;
   c = x.c[0]; x.c[0] = x.c[7]; x.c[7] = c;
   c = x.c[1]; x.c[1] = x.c[6]; x.c[6] = c;
   c = x.c[2]; x.c[2] = x.c[5]; x.c[5] = c;
   c = x.c[3]; x.c[3] = x.c[4]; x.c[4] = c;
   return(x.ull);
}
void limitXY(int16_t currentX, int16_t currentY, int16_t *eolXNeg, int16_t *eolXPos,
		    int16_t *eolYNeg, int16_t * eolYPos, int16_t delta)
{

    if ((currentX + delta) >= kMaxSigned)
      *eolXPos = kMaxSigned;
    else
      *eolXPos = currentX + delta;
    if ((currentX - delta) <= kMinSigned)
      *eolXNeg = kMinSigned;
    else
      *eolXNeg = currentX - delta;
    if ((currentY + delta) >= kMaxSigned)
      *eolYPos = kMaxSigned;
    else
      *eolYPos = currentY + delta;
    if ((currentY - delta) <= kMinSigned)
      *eolYNeg = kMinSigned;
    else
      *eolYNeg = currentY - delta;
      
    return;
}
uint32_t  get_num_disp_points(void)
{
    return(cur_num_disp_points);
}  
