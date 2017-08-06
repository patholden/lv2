
/*
static char rcsid[] = "$Id: RightOnFullReg.c,v 1.2 2000/05/05 23:57:15 ags-sw Exp $";
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
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "AppCommon.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "3DTransform.h"
#include "Protocol.h"
#include "RightOnFullReg.h"

#define	kNumberOfSensorSearchAttempts 3

#ifdef AGS_DEBUG
void LogRightOnFullRegCommand(struct parse_rightondofullreg_parms *param, struct lg_master *pLgMaster);

void LogRightOnFullRegResponse(struct parse_rightondofullreg_resp *pRespBuf, uint32_t respLen);
#endif

void RightOnFullReg(struct lg_master *pLgMaster,
		    struct parse_rightondofullreg_parms *param,
		    uint32_t respondToWhom)
{
    struct parse_rightondofullreg_resp *pRespBuf;
    struct lg_xydata theAngleBuffer[MAX_TARGETSOLD];
    transform        foundTransform;
    double           foundAngles[MAX_TARGETSOLD * 2];
    double           theCoordinateBuffer[MAX_TARGETSOLD * 3];
    double           theTransformTolerance;
    double           XExternalAngles[MAX_TARGETSOLD];
    double           YExternalAngles[MAX_TARGETSOLD];
    double           XfoundAngles[MAX_TARGETSOLD];
    double           YfoundAngles[MAX_TARGETSOLD];
    int32_t          RawGeomFlag;
    int32_t          tmpAngle;
    int              numberOfFoundTargets = 0;
    int              searchResult;
    int              useTarget[MAX_TARGETSOLD];
    uint32_t         respLen = (sizeof(struct parse_rightondofullreg_resp)-kCRCSize);
    uint32_t         rc = 0;
    uint32_t         lostSensors;
    int16_t          Xarr[MAX_TARGETSOLD];
    int16_t          Yarr[MAX_TARGETSOLD];
    int16_t          ptX, ptY, fndX, fndY;
    unsigned short   i, j;
    unsigned char    theResult;
				
#ifdef AGS_DEBUG
    syslog(LOG_DEBUG, "Entered Routine: RighOnFullReg");

    LogRightOnFullRegCommand(param, pLgMaster);
#endif

    SlowDownAndStop(pLgMaster);
 
    pRespBuf = (struct parse_rightondofullreg_resp *)pLgMaster->theResponseBuffer;

    memset(pRespBuf, 0, respLen);
				
    theTransformTolerance  = pLgMaster->gArgTol;

    for (i = 0; i < MAX_TARGETSOLD; i++)
      {
	pLgMaster->foundTarget[i] = 0;
	useTarget[i] = 1;
      }
				
    gWorstTolReg = 1.0;
				
    RawGeomFlag = param->angleflag;  // 1: raw binary angles, 2: geometric angles

    if (RawGeomFlag == 1)
      {
	for (i = 0; i < MAX_TARGETSOLD; i++ )
	  {
	    theAngleBuffer[i].xdata = (int16_t)param->target_raw_angle[i].xangle & kMaxUnsigned;
	    theAngleBuffer[i].ydata = (int16_t)param->target_raw_angle[i].yangle & kMaxUnsigned;
	  }
      } else if (RawGeomFlag == 2)
      {
	for ( i = 0; i < MAX_TARGETSOLD; i++ )
	  {
	    rc = ConvertExternalAnglesToBinary (pLgMaster,
					        param->target_geo_angle[i].Xangle,
					        param->target_geo_angle[i].Yangle,
					        &theAngleBuffer[i].xdata,
					        &theAngleBuffer[i].ydata);
	  }
      }
				
    // save target coordinates
    for (i = 0; i < MAX_TARGETSOLD; i++)
      {
	theCoordinateBuffer[(3*i)+0] = param->target[i].Xtgt;
	theCoordinateBuffer[(3*i)+1] = param->target[i].Ytgt;
	theCoordinateBuffer[(3*i)+2] = param->target[i].Ztgt;
      }

    SaveFullRegCoordinates(MAX_TARGETSOLD, theCoordinateBuffer);

    // check for duplicates
    for (i = 0; i < MAX_TARGETSOLD; i++)
      {
	for (j = i; j < MAX_TARGETSOLD; j++)
	  {
	    if (i != j)
	      {
		if ((gX[i]==gX[j]) && (gY[i]==gY[j]) && (gZ[i]==gZ[j]))
		  useTarget[j] = 0;
	      }
	  }
      }
				
    lostSensors = 0;
				
    if (rc == 0)
      {
	for (i = 0; i < MAX_TARGETSOLD; i++)
	  {
	    j = gNumberOfSensorSearchAttempts;

	    gSearchCurrentSensor = i;

	    /*
	     *  allow for a variable speed search, in needed
	     */
            pLgMaster->gCoarse2Factor = pLgMaster->saveCoarse2Factor;
            pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;


	    while (j--)
	      {
		ptX = theAngleBuffer[i].xdata;
		ptY = theAngleBuffer[i].ydata;

		if (useTarget[i] == 1) {
#ifdef ZDEBUG
                    pLgMaster->debug_count++;
#endif
                    pLgMaster->current_target = i;
		    searchResult = SearchForASensor(pLgMaster, ptX, ptY, &fndX, &fndY);
		  }
		else
		  {
		    searchResult = 99;
		    fndX = 0;
		    fndY = 0;
		  }

		Xarr[i] = fndX;
		Yarr[i] = fndY;

		if (searchResult == kStopWasDone)
		  {
		    SearchBeamOff(pLgMaster);
		    return;
		  }

		if (!searchResult)
		  break;

		pLgMaster->gCoarse2SearchStep /= 2;
		pLgMaster->gCoarse2Factor     /= 2; 
		if (pLgMaster->gCoarse2SearchStep <= 1)
		  {
		    pLgMaster->gCoarse2SearchStep = 1;
		    pLgMaster->gCoarse2Factor     = 1;
		  }
	      }
	    
	    if (searchResult)
	      lostSensors += 1U << i;
	    else
	      {
		pLgMaster->foundTarget[i] = 1;
		ConvertBinaryToGeometricAngles(pLgMaster, fndX, fndY, &(XfoundAngles[i]), &(YfoundAngles[i]));
		ConvertBinaryToExternalAngles(pLgMaster, fndX, fndY,  &(XExternalAngles[i]), &(YExternalAngles[i]));
	      }
	  }
	
	for (i = 0; i < MAX_TARGETSOLD; i++)
	  {
	    foundAngles[2*i+0] = XfoundAngles[i];
	    foundAngles[2*i+1] = YfoundAngles[i];
	    if (pLgMaster->foundTarget[i] == 1)
	      numberOfFoundTargets ++;
          }

	if (numberOfFoundTargets >= 4)
	  {
	    theResult = FindTransformMatrix(pLgMaster, MAX_TARGETSOLD,
					    gDeltaMirror, theTransformTolerance, foundAngles,
					    (double *)&foundTransform );
          }
        }
    else
      theResult = 0;

    if (theResult)
      {
	pRespBuf->hdr.status3 = RESPGOOD;
	pRespBuf->hdr.numTransforms = (unsigned char)(0xFF & GnOfTrans);
	
	if (gTargetDrift)
	  InitDrift(Xarr, Yarr);
	
	memset(pRespBuf->transform, 0, OLDTRANSFORMLEN);
	TransformIntoArray(&foundTransform, (double *)&pRespBuf->transform);

	for (i = 0; i < MAX_TARGETSOLD; i++)
	  {
	    tmpAngle = Xarr[i];
	    memcpy(&pRespBuf->anglepairs[8*i+0], &tmpAngle, sizeof(int32_t));
	    tmpAngle = Yarr[i];
	    memcpy(&pRespBuf->anglepairs[8*i+4], &tmpAngle, sizeof(int32_t));
	  }
	     
	for (i = 0; i < MAX_TARGETSOLD; i++)
	  {
	    pRespBuf->target_geo_angle[i].Xangle = XExternalAngles[i];
	    pRespBuf->target_geo_angle[i].Yangle = YExternalAngles[i];
	  }

#ifdef AGS_DEBUG
	LogRightOnFullRegResponse(pRespBuf, respLen);
#endif
	
	HandleResponse (pLgMaster, respLen, respondToWhom );
      }
    else
      {
	pRespBuf->hdr.status3 = RESPFAIL;

	pRespBuf->hdr.hdr |= htonl(lostSensors);

	HandleResponse (pLgMaster, sizeof(pRespBuf->hdr.hdr), respondToWhom );

	return;
      }
				
    return;
}

#ifdef AGS_DEBUG
void LogRightOnFullRegCommand(struct parse_rightondofullreg_parms *param, struct lg_master *pLgMaster)
{
    int32_t          i;
    int32_t          RawGeomFlag;

    syslog(LOG_DEBUG, "CMD: HeaderSpecialByte: %02x", pLgMaster->gHeaderSpecialByte);

    RawGeomFlag = param->angleflag;

    syslog(LOG_DEBUG, "CMD: angleflag: %d", param->angleflag);

    for (i = 0; i < MAX_TARGETSOLD; i++)
      {
        syslog(LOG_DEBUG, "CMD: target #%d: target[%d].Xtgt: %f   target[%d].Ytgt: %f   target[%d].Ztgt: %f",
               i + 1,
	       i,
  	       param->target[i].Xtgt,
	       i,
	       param->target[i].Ytgt,
	       i,
	       param->target[i].Ztgt);
	}

    if (RawGeomFlag == 2)
      {
	for (i = 0; i < MAX_TARGETSOLD; i++)
	  {
	    syslog(LOG_DEBUG, "CMD: target #%d: target_geo_angle[%d].Xangle: %f   target_geo_angle[%d].Yangle: %f",
		   i + 1,
		   i,
		   param->target_geo_angle[i].Xangle,
		   i,
		   param->target_geo_angle[i].Yangle);
	  }
      }
    
    if (RawGeomFlag == 1)
      {
	for (i = 0; i < MAX_TARGETSOLD; i++)
	  {
	    syslog(LOG_DEBUG, "CMD: target #%d: target_raw_angle[%d].xangle: %d   target_raw_angle[%d].yangle: %d",
		   i + 1,
		   i,
		   param->target_raw_angle[i].xangle & kMaxUnsigned,
		   i,
		   param->target_raw_angle[i].yangle & kMaxUnsigned);
	  }
      }

    return;    
}


void LogRightOnFullRegResponse(struct parse_rightondofullreg_resp *pRespBuf, uint32_t respLen)
{
    double           transform[MAX_NEW_TRANSFORM_ITEMS];
    int32_t          i;
    
    syslog(LOG_DEBUG, "RSP: hdr: %08x", htonl(pRespBuf->hdr.hdr));

    if (respLen <= sizeof(pRespBuf->hdr.hdr))
	return;

    memcpy(transform, pRespBuf->transform, sizeof(transform));
    for (i = 0; i < MAX_NEW_TRANSFORM_ITEMS; i++)
      {
	syslog(LOG_DEBUG, "RSP: transform[%d]: %f", i, transform[i]);
      }

    for (i = 0; i < MAX_TARGETSOLD; i++)
      {
        syslog(LOG_DEBUG, "RSP: target #%d: anglepairsX[%d]: %d   anglepairsY[%d]: %d",
	       i + 1,
	       2*i+0,
	       ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+0],
	       2*i+1,
  	       ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+1]);
      }

    for (i = 0; i < MAX_TARGETSOLD; i++)
      {
        syslog(LOG_DEBUG, "RSP: target #%d: target_geo_angle[%d].Xangle: %f   target_geo_angle[%d].yangle: %f",
	       i + 1,
	       i,
	       pRespBuf->target_geo_angle[i].Xangle,
	       i,
  	       pRespBuf->target_geo_angle[i].Yangle);
      }

    return;
}
#endif
