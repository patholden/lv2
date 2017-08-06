//static char rcsid[] = "$Id: LaserCmds.c,v 1.18 2003/04/25 10:40:04 ags-sw Exp ags-sw $";

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stddef.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <endian.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include <syslog.h>
#include <poll.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "AppStrListIDs.h"
#include "LaserCmds.h"
#include "AppErrors.h"
#include "L3DTransform.h"
#include "3DTransform.h"
#include "SensorRegistration.h"
#include "LaserInterface.h"
#include "LaserPattern.h"
#include "APTParser.h"
#include "Video.h"
#include "QuickCheckManager.h"
#include "Protocol.h"
#include "Web.h"


#define _SIKORSKY_DEMO_		0

#if _SIKORSKY_DEMO_
#define	kTheHardCodedHeight	-108.5L
#endif

enum
{
	kInitDataBufferErr = 1,
	kInitializingLaserCmd
};



#define MAXLINE 1024
#define BUFFSIZE  (2048+3*512*512)

static char FromVideo[BUFFSIZE];
static char ToVideo[MAXLINE];


#ifdef __unix__
#define gSameEndian false
#else
#define gSameEndian true
#endif

void ResetPlyCounter(struct lg_master *pLgMaster)
{
    // Resets all relevant data for a display to initial values
    pLgMaster->gPlysToDisplay = 0;
    pLgMaster->gPlysReceived = 0;
    return;
}
void DoDisplayChunksStart (struct lg_master *pLgMaster, struct parse_chunkstart_parms *pInp, uint32_t respondToWhom )
{
    struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
    uint32_t                dataLength;

    // Note this input is in BIG endian format on the wire
    dataLength = ntohl(pInp->apt_len);
    memset(pResp, 0, sizeof(struct parse_basic_resp));
    if (!dataLength || (dataLength > kMaxDataLength))
      {
	pResp->hdr.status = RESPFAIL;
	if (dataLength)
	  pResp->hdr.errtype = htons(RESPDATATOOLARGE);
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }
    if (!pLgMaster->gDataChunksBuffer)
      {
	pResp->hdr.status = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }

    // Ready to go, initialize variables & buffers
    memset(pLgMaster->gDataChunksBuffer, 0, kMaxDataLength);
    pLgMaster->gDataChunksLength = dataLength;
    pLgMaster->gTransmitLengthSum = 0;
    pResp->hdr.status = RESPGOOD;

    // If no-response flag is set, don't send response
    if (!(pLgMaster->gHeaderSpecialByte & 0x80))
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
    return;
}

void DoDisplayChunks(struct lg_master *pLgMaster, struct parse_chunksdo_parms *pInp, uint32_t respondToWhom)
{
  struct displayData  dispData;
  double tmpDoubleArr[MAX_NEW_TRANSFORM_ITEMS];
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  char *tmpPtr;
  double *p_transform;
  int index;
  int i;
  int numberOfTargets;
  int checkQC;
  int error;

  // Do some validations before continuing
  if (pLgMaster->gTransmitLengthSum < pLgMaster->gDataChunksLength)
    {
      pResp->hdr.status1 = RESPFAIL;
      pResp->hdr.errtype1 = RESPE1APTERROR;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter(pLgMaster);
      return;
    }
  if (!pLgMaster->gSensorBuffer)
    {
      pResp->hdr.status1 = RESPFAIL;
      pResp->hdr.errtype1 = RESPE1BOARDERROR;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter(pLgMaster);
      return;
    }
	
  pLgMaster->gAbortDisplay = false;
  memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
  memset((char *)&dispData, 0, sizeof(struct displayData));
  
  index  = pLgMaster->gPlysReceived * kNumberOfRegPoints * 2 * sizeof(uint32_t);
  tmpPtr = (char *)((char *)pLgMaster->gSensorBuffer + index);
  numberOfTargets = MAX_TARGETSOLD;
  gQuickCheckTargetNumber[pLgMaster->gPlysReceived] = numberOfTargets;

  // Get angle pairs
  memcpy(tmpPtr, pInp->chunk_anglepairs, (kNumberOfRegPoints * 2 * sizeof(uint32_t)));

  //  KLUDGE to prevent quickcheck
  checkQC = 0;
  for (i = 0; i < numberOfTargets; i++)
    {
      // Check pair
      if ((pInp->chunk_anglepairs[i] != 0) || (pInp->chunk_anglepairs[i+1] != 0))
	checkQC++;
    }
  if (checkQC == 0)
    {
      pLgMaster->gQCcount = -1;
      initQCcounter(pLgMaster);
    }

  p_transform = (double *)((char *)pInp + offsetof(struct parse_chunksdo_parms,chunk_transform));
  error = 0;
  for (i=0; i<12; i++)
    {
      tmpDoubleArr[i] = p_transform[i];
      if (isnan(tmpDoubleArr[i]))
	error = 1;
      if (isinf(tmpDoubleArr[i]))
	error = 1;
      if ((i < 9) && (fabs(tmpDoubleArr[i]) > 10.0))
	error = 1;
    }
  if (error)
    {
      pResp->hdr.status = RESPFAIL;
      pResp->hdr.errtype = RESPOTHERERROR; 
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp) -kCRCSize), respondToWhom);
      ResetPlyCounter(pLgMaster);
      return;
    }
  if (pLgMaster->gPlysReceived++)
    ChangeTransform((double *)&tmpDoubleArr);
  else
    {
      dispData.numberOfSensorSets = pLgMaster->gPlysToDisplay;
      dispData.sensorAngles = (int32_t *)pLgMaster->gSensorBuffer;
      dispData.pattern = SetUpLaserPattern(pLgMaster, tmpDoubleArr);
    }
  pResp->hdr.errtype = ProcessPatternData(pLgMaster, pLgMaster->gDataChunksBuffer, pLgMaster->gDataChunksLength);
  if (pResp->hdr.errtype)
    {
      pResp->hdr.status = RESPFAIL;
      pResp->hdr.errtype = pResp->hdr.errtype;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter(pLgMaster);
      return;
    }
  
  if (pLgMaster->gPlysReceived < pLgMaster->gPlysToDisplay)
    {
      SetPenUp();
      PendPenDown();
      pResp->hdr.errtype = 0;
    }
  else
    pResp->hdr.errtype = (uint16_t)FinishPattern(pLgMaster);

  if (pResp->hdr.errtype)
    {
      pResp->hdr.status  = RESPFAIL;
      pResp->hdr.errtype = htons(pResp->hdr.errtype);
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter(pLgMaster);
      return;
    }
  if (pLgMaster->gAbortDisplay || (pLgMaster->gPlysReceived < pLgMaster->gPlysToDisplay))
    {
      pResp->hdr.status  = RESPGOOD;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      ResetPlyCounter(pLgMaster);
      return;
    }
  else
    {
      if ((CDRHflag(pLgMaster)))
	{
	  pResp->hdr.status1 = RESPFAIL;
	  pResp->hdr.errtype1 = RESPE1BOARDERROR;
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
	  ResetPlyCounter(pLgMaster);
	  return;
	}
      gVideoCheck = 0;
      SearchBeamOff(pLgMaster);
      if ((CDRHflag(pLgMaster)))
	{
	  pResp->hdr.status1 = RESPFAIL;
	  pResp->hdr.errtype1 = RESPE1BOARDERROR; 
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  ResetPlyCounter(pLgMaster);
	  return;
	}
      PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, SENDRESPONSE, respondToWhom);
      ResetPlyCounter(pLgMaster);
    }
  return;
}
	
void AddDisplayChunksData(struct lg_master *pLgMaster, uint32_t dataLength,
			  uint32_t dataOffset, char *patternData, uint32_t respondToWhom)
{
    struct displayData     dispData;
    struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
#ifdef ZDEBUG
    int i;
    char * ptr;
#endif
  
    memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
    memset((char *)&dispData, 0, sizeof(struct displayData));

    // dataLength = num bytes in this packet
    // dataOffset = offset into entire payload
    // pLgMaster->gTransmitLengthSum = how many bytes received so far
    // 
    if ((dataLength + dataOffset) > pLgMaster->gDataChunksLength)
      {
	pResp->hdr.status = RESPFAIL;
	pResp->hdr.errtype1 = RESPDATATOOLARGE;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }
    pLgMaster->gTransmitLengthSum += dataLength;
    if (pLgMaster->gTransmitLengthSum > pLgMaster->gDataChunksLength)
      {
	pResp->hdr.status = RESPFAIL;
	pResp->hdr.errtype1 = RESPDATATOOLARGE;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }
#ifdef ZDEBUG
      ptr = patternData;
      for ( i=0; i < dataLength; i += 16 ) {
        syslog( LOG_DEBUG ,
                "chunkdata %5d " 
                " %2x %2x %2x %2x"
                " %2x %2x %2x %2x"
                " %2x %2x %2x %2x"
                " %2x %2x %2x %2x"
               , i
               , ptr[i +  0]
               , ptr[i +  1]
               , ptr[i +  2]
               , ptr[i +  3]
               , ptr[i +  4]
               , ptr[i +  5]
               , ptr[i +  6]
               , ptr[i +  7]
               , ptr[i +  8]
               , ptr[i +  9]
               , ptr[i + 10]
               , ptr[i + 11]
               , ptr[i + 12]
               , ptr[i + 13]
               , ptr[i + 14]
               , ptr[i + 15]
               );
      }

#endif
    memcpy((char *)(pLgMaster->gDataChunksBuffer + dataOffset), patternData, dataLength);
    pResp->hdr.status = RESPGOOD;
    if (!(pLgMaster->gHeaderSpecialByte & 0x80))
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
    return;
}

int IfStopThenStopAndNeg1Else0 (struct lg_master *pLgMaster)
{
    struct pollfd poll_set[4];  // 4 is somewhat arbitary
    int numfds = 0;
    int status;

    // use poll function to see if any data
    // came in on either serial or ether (if socket open)
    // if so, return a "1"
    // otherwise return "0"

    numfds = 0;
    if ( pLgMaster->datafd >= 0 ) {
        poll_set[0].fd = pLgMaster->datafd;
        poll_set[0].events = POLLIN;
        numfds++;
    }
        // poll_set[1].fd = pLgMaster->pc_serial;
        // poll_set[1].events = POLLIN;
        // numfds++;
    status = poll( poll_set, numfds, 0 );
   
    if (  (poll_set[0].revents & POLLIN) && status >= 1 ) {
        syslog(LOG_NOTICE, "IfStopThenStopAndNeg1Else0 stopping (ethernet)" );
	pLgMaster->rcvdStopCmd = 0;
	return(1);
    }
    // if (  (poll_set[1].revents & POLLIN) && status >= 1 ) {
    //     syslog(LOG_NOTICE, "IfStopThenStopAndNeg1Else0 stopping (serial)" );
    // 	   pLgMaster->rcvdStopCmd = 0;
    //     return(1);
    // }

    return(0);
}

void DoStopCmd(struct lg_master *pLgMaster, uint32_t respondToWhom)
{
    PostCommand(pLgMaster, kStop, 0, respondToWhom);
    pLgMaster->rcvdStopCmd = 1;
    return;
}


void DoGoAngle (struct lg_master *pLgMaster, struct parse_goangle_parms *pInp, uint32_t respondToWhom )
{
    struct lg_xydata xydata;
    double Xin, Yin;
    int      return_val = 0;
    struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;

    memset(pResp, 0, sizeof(struct parse_basic_resp));

    // Check input parms pointer & get x/y data
    if (!pInp)
      return;
    
    // both X & Y inputs come in reversed
    Xin = ArrayToDouble(pInp->x.xData);
    Yin = ArrayToDouble(pInp->y.yData);
  if (pLgMaster->gHeaderSpecialByte & 0x10)
     {
 	syslog(LOG_NOTICE, "using DACs %f %f\n", Xin, Yin);
 	xydata.xdata = Xin;
        xydata.ydata = Yin;
  	syslog(LOG_NOTICE, "using DACs %d %d\n", xydata.xdata, xydata.ydata);
     }
  else
    {
        return_val =  ConvertExternalAnglesToBinary(pLgMaster, Xin, Yin, &xydata.xdata, &xydata.ydata);
        if (return_val)
        {
           pResp->hdr.status1 = RESPFAIL;
           pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
           HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
           return;
        }
    }
  if ((CDRHflag(pLgMaster)))
    {
	  pResp->hdr.status1 =RESPFAIL;
	  pResp->hdr.errtype1 = RESPE1BOARDERROR; 
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  return;
    }
  PostCmdGoAngle(pLgMaster, (struct lg_xydata *)&xydata, respondToWhom );
  return;
}

void DoEtherAngle (struct lg_master *pLgMaster, struct parse_ethangle_parms *pInp, uint32_t respondToWhom )
{
    struct lg_xydata xydata;
    struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
    double Xin, Yin;
    int    return_val;
	
    memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));

    if (!pInp)
      return;

    // both X & Y inputs come in reversed
    Xin = ArrayToDouble(pInp->x.xData);
    Yin = ArrayToDouble(pInp->y.yData);

    if (pLgMaster->gHeaderSpecialByte & 0x10)
      {
 	syslog(LOG_NOTICE, "using DACs %f %f\n", Xin, Yin);
 	xydata.xdata = Xin;
        xydata.ydata = Yin;
  	syslog(LOG_NOTICE, "using DACs %d %d\n", xydata.xdata, xydata.ydata);
      }
    else
      {
	return_val =  ConvertExternalAnglesToBinary(pLgMaster, Xin, Yin, &xydata.xdata, &xydata.ydata);

	if (return_val)
	  {
	    pResp->hdr.status1 = RESPFAIL;
	    pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
	    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	    return;
	  }
      }

    if ((CDRHflag(pLgMaster)))
      {
	pResp->hdr.status1 = RESPFAIL;
	pResp->hdr.errtype1 = RESPE1BOARDERROR;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }

    PostCmdEtherAngle(pLgMaster, (struct lg_xydata *)&xydata);
    
    return;
}

void DarkAngle(struct lg_master *pLgMaster, struct parse_dkangle_parms *pInp, uint32_t respondToWhom)
{
    struct lg_xydata xydata;
    struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
    double Xin, Yin;
    int    return_val;

    memset((char *)pResp, 0, sizeof(struct parse_basic_resp));
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));

    if (!pInp)
      return;

    // both X & Y inputs come in reversed
    Xin = ArrayToDouble(pInp->x.xData);
    Yin = ArrayToDouble(pInp->y.yData);

    if (pLgMaster->gHeaderSpecialByte & 0x10)
      {
 	syslog(LOG_NOTICE, "using DACs %f %f\n", Xin, Yin);
 	xydata.xdata = Xin;
        xydata.ydata = Yin;
  	syslog(LOG_NOTICE, "using DACs %d %d\n", xydata.xdata, xydata.ydata);
      }
    else
      {
	return_val =  ConvertExternalAnglesToBinary(pLgMaster, Xin, Yin, &xydata.xdata, &xydata.ydata);

	if (return_val)
	  {
	    pResp->hdr.status1 = RESPFAIL;
	    pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
	    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	    return;
	  }
      }

    PostCmdDarkAngle(pLgMaster, &xydata);
      
    return;
}

void DimAngle(struct lg_master *pLgMaster, struct parse_dimangle_parms *pInp, uint32_t respondToWhom)
{
    struct lg_xydata xydata;
    struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
    double      Xin, Yin;
    int         return_val;
    uint32_t    pulseoffvalue;
    uint32_t    pulseonvalue;

    Xin = pInp->xData;
    Yin = pInp->yData;
    pulseoffvalue = pInp->pulseoff;
    pulseonvalue = pInp->pulseon;
    return_val =  ConvertExternalAnglesToBinary(pLgMaster, Xin, Yin, &xydata.xdata, &xydata.ydata);
    if (return_val)
      {
	pResp->hdr.status1 = RESPFAIL;
	pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }
    GoToPulse(pLgMaster, &xydata, pulseoffvalue, pulseonvalue);
    return;
}

void DoDisplayKitVideo (struct lg_master *pLgMaster, uint32_t dataLength,
			unsigned char *otherParameters, char *patternData,
			uint32_t respondToWhom)
{
    short i;
    double tmpDoubleArr[12];
    struct displayData dispData;
    struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
    int index;
    unsigned char * tmpPtr;
    int32_t   tmpLong;
    transform   invTransform, CurrentTransform;
    int count;
    double inputPoint[3], outputPoint[3];

    // Init all buffers
    memset((char *)&inputPoint, 0, sizeof(inputPoint));
    memset((char *)&outputPoint, 0, sizeof(outputPoint));
    memset((char *)&tmpDoubleArr, 0, sizeof(tmpDoubleArr));
    memset(pResp, 0, sizeof(struct parse_basic_resp));
    memset((char *)&dispData, 0, sizeof(struct displayData));
    
    index = 0;
    tmpPtr = &otherParameters[index];
    ArrayIntoTransform ( (double *)tmpPtr, &CurrentTransform  );
    InvertTransform ( &CurrentTransform, &invTransform );

    index =
                   12 * kSizeOldLongDouble
                 ;
    tmpPtr = &otherParameters[index];
    tmpLong = *(int32_t *)tmpPtr;
    if ( tmpLong > 0 ) {
       gVideoPreDwell = tmpLong;
    } else {
       gVideoPreDwell = 1000;
    }
#if defined(SDEBUG) || defined(KITDEBUG)
    syslog(LOG_NOTICE, "gVideoPreDwell %d\n", gVideoPreDwell );
#endif


    index =   
                   12 * kSizeOldLongDouble
              +         sizeof ( int32_t )
                 ;
    tmpPtr = &otherParameters[index];
    tmpLong = *(int32_t *)tmpPtr;
    if ( tmpLong > 0 ) {
       gVideoCount = tmpLong;
    } else {
       gVideoCount = 5000;
    }
#if defined(SDEBUG) || defined(KITDEBUG)
    syslof(LOG_NOTICE, "gVideoCount %d\n", gVideoCount );
#endif
    

    index =     12 * kSizeOldLongDouble
              +      sizeof ( int32_t )
              +      sizeof ( int32_t )
              ;
    tmpPtr = &otherParameters[index];

    inputPoint[0] = ((double *)tmpPtr)[0];
    inputPoint[1] = ((double *)tmpPtr)[1];
    inputPoint[2] = ((double *)tmpPtr)[2];
#if defined(SDEBUG) || defined(KITDEBUG)
    syslog(LOG_NOTICE, "XYZ %.5lf %.lf %.lf\n",((double *)tmpPtr)[0],
	   ((double *)tmpPtr)[1], ((double *)tmpPtr)[3]);
#endif
    TransformPoint ( &CurrentTransform, inputPoint, outputPoint );
#if defined(SDEBUG) || defined(KITDEBUG)
    double Xpoint, Ypoint, Zpoint;
    Xpoint = outputPoint[0];
    Ypoint = outputPoint[1];
    Zpoint = outputPoint[2];
    syslog(LOG_NOTICE, "check XYZ  %.5lf %.5lf %.5lf",outputPoint[0],
	     outputPoint[1], outputPoint[2]);
#endif

    PointToBinary(pLgMaster, outputPoint, &pLgMaster->gXcheck, &pLgMaster->gYcheck );

#if defined(SDEBUG) || defined(KITDEBUG)
    syslog(LOG_NOTICE, "check XY raw %x %x", pLgMaster->gXcheck, pLgMaster->gYcheck);
#endif

	pLgMaster->gAbortDisplay = false;
	index  = 0;
	tmpPtr = &otherParameters[index];
	for (i=0; i<12; i++)
	  tmpDoubleArr[i] = (double)(((double *)tmpPtr)[i]);
	ChangeTransform((double *)&tmpDoubleArr);
	
	dispData.numberOfSensorSets = 0;
	dispData.sensorAngles = (int32_t *)pLgMaster->gSensorBuffer;
	dispData.pattern = SetUpLaserPattern(pLgMaster, tmpDoubleArr);
	pResp->hdr.errtype = ProcessPatternData(pLgMaster, patternData, dataLength);	
 	if (pResp->hdr.errtype)
	  {
	    pResp->hdr.status = RESPFAIL;
	    pResp->hdr.errtype = htons(pResp->hdr.errtype);
	    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	    ResetPlyCounter(pLgMaster);
	    return;
	  }
 	
#ifdef KITDEBUG
	syslog(LOG_NOTICE, "1555 plys rcvd %d  disp %d", pLgMaster->gPlysReceived, pLgMaster->gPlysToDisplay);
#endif
 	if (pLgMaster->gPlysReceived < pLgMaster->gPlysToDisplay)
	  {
	    SetPenUp();
	    pResp->hdr.errtype = 0;
	  }
 	else
	  pResp->hdr.errtype = (uint16_t)FinishPattern(pLgMaster);
 
 	if (pResp->hdr.errtype)
 	{
	  pResp->hdr.status = RESPFAIL; 
	  pResp->hdr.errtype = htons(pResp->hdr.errtype);
	  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	  ResetPlyCounter(pLgMaster);
 	}
 	else
 	{
	  if (pLgMaster->gAbortDisplay || (pLgMaster->gPlysReceived < pLgMaster->gPlysToDisplay))
	    {
#ifdef KITDEBUG
	      syslog(LOG_NOTICE, "1574 plys rcvd %d  disp %d", pLgMaster->gPlysReceived, pLgMaster->gPlysToDisplay);
#endif
	      pResp->hdr.status = RESPGOOD;
	      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	    }
	  else
	    {
                        gQuickCheck = 0;
                        gVideoCheck = 1;
 			SearchBeamOff(pLgMaster);

			count = sprintf( ToVideo,
				      "GET  /cgi-bin/reset"
				      "   HTTP/1.0\n\n"
				       );
#ifdef KITDEBUG
			int slen;
			slen = GetWebVideo(pLgMaster, ToVideo, count, FromVideo );
			syslog(LOG_NOTICE, "1634 getweb slen %d gVideoCount %d\n", slen, gVideoCount);
#else
			GetWebVideo(pLgMaster, ToVideo, count, FromVideo );
#endif

                        if ( (CDRHflag(pLgMaster) )  ) {
			  pResp->hdr.status1 = RESPFAIL;
			  pResp->hdr.errtype1 = RESPE1BOARDERROR;
			  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
			  ResetPlyCounter(pLgMaster);
			  return;
                        }
#if defined(KITDEBUG)
			syslog(LOG_NOTICE, "LC1651 setVideoCount %d\n", gVideoCount );
#endif
			SetQCcounter(pLgMaster, gVideoCount );
                        gQCtimer = -1;
			PostCmdDisplay(pLgMaster, (struct displayData *)&dispData, SENDRESPONSE, respondToWhom );
 			ResetPlyCounter(pLgMaster);
 		}
  	}
}

void SetDisplaySeveral(struct lg_master *pLgMaster, uint32_t number, uint32_t respondToWhom )
{
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;

  memset(pResp, 0, sizeof(struct parse_basic_resp));
  if (number > (kMaxNumberOfPlies * kNumberOfFlexPoints * 2 * sizeof(uint32_t)))
    {
      pResp->hdr.status = RESPFAIL;
      HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
      return;
    }
  memset((char *)&pLgMaster->gOutOfRange, 0, sizeof(struct k_header)); 
  ResetPlyCounter(pLgMaster);
  pLgMaster->gPlysToDisplay = number;

  // Clear buffer for display data
  memset(pLgMaster->gSensorBuffer, 0, (kNumberOfFlexPoints * 2 * pLgMaster->gPlysToDisplay * sizeof(uint32_t)));
  // Try to open shutter for display operation here, optic status is checked later
  // during actual display operation
  doSetShutterENB(pLgMaster);

  pResp->hdr.status = RESPGOOD;
  if (!(pLgMaster->gHeaderSpecialByte & 0x80))
    HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
  return;
}

void DoQuickCheck (struct lg_master *pLgMaster, struct parse_qkcheck_parms *pInp, uint32_t respondToWhom )
{

    gRespondToWhom = respondToWhom;
      // must now set number of targets, hard-coded to max val for old max.
    PerformAndSendQuickCheck (pLgMaster, pInp->anglepairs, MAX_TARGETSOLD);
    return;
}
