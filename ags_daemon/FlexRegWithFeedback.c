/*
static char rcsid[] = "$Id$";

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
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "FlexFullRegWithFeedback.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "3DTransform.h"
#include "L3DTransform.h"
#include "FOM.h"
#include "parse_data.h"
#ifdef NEW_TFIND
#include "TargetFind.h"
#endif

#ifdef AGS_DEBUG
void LogFlexRightOnDoFullRegWithFeedbackCommand(struct parse_flexrightondofullregwithfeedback_parms *param, struct lg_master *pLgMaster);

void LogFlexRightOnDoFullRegWithFeedbackResponse(struct parse_flexrightondofullregwithfeedback_resp *pRespBuf, uint32_t respLen);
#endif

void FlexFullRegWithFeedback(struct lg_master *pLgMaster,
			     struct parse_flexrightondofullregwithfeedback_parms *param,
			     uint32_t respondToWhom)
{
    struct parse_flexfail_resp *pFlexFailResp;
    struct parse_flexrightondofullregwithfeedback_resp *pRespBuf;
    struct lg_xydata theAngleBuffer[MAX_TARGETSFLEX];
    transform        foundTransform;
    double           foundAngles[MAX_TARGETSFLEX * 2];
    double           theCoordinateBuffer[MAX_TARGETSFLEX * 3];
    double           theTransformTolerance;
    double           XExternalAngles[MAX_TARGETSFLEX];
    double           YExternalAngles[MAX_TARGETSFLEX];
    double           XfoundAngles[MAX_TARGETSFLEX];
    double           YfoundAngles[MAX_TARGETSFLEX];
    double           useTol;
    int32_t          intColinear;
    int32_t          intPlanar;
    int32_t          RawGeomFlag;
    int              numberOfFoundTargets;
    int              searchResult;
    int              useTarget[MAX_TARGETSFLEX];
    uint32_t         flexFailRespLen;
    uint32_t         lostSensors;
    uint32_t         numberOfTargets;
    uint32_t         respLen;
    uint32_t         rc = 0;
    uint32_t         target_status[MAX_TARGETSFLEX];
    uint16_t         i, j;
    int16_t          fndX, fndY;
    int16_t          Xarr[MAX_TARGETSFLEX];
    int16_t          Yarr[MAX_TARGETSFLEX];
    int16_t          ptX, ptY;
    unsigned char    saveHeaderSpecialByte;
    unsigned char    theResult;

#ifdef AGS_DEBUG
    syslog(LOG_DEBUG, "Entered Routine: FlexFullRegWithFeedback");

    LogFlexRightOnDoFullRegWithFeedbackCommand(param, pLgMaster);
#endif
    
    SlowDownAndStop(pLgMaster);

    // Initialize variables
    flexFailRespLen = (sizeof(struct parse_flexfail_resp));
    respLen = (sizeof(struct parse_flexrightondofullregwithfeedback_resp)-kCRCSize);
    pFlexFailResp = (struct parse_flexfail_resp *)pLgMaster->theResponseBuffer;
    pRespBuf = (struct parse_flexrightondofullregwithfeedback_resp *)pLgMaster->theResponseBuffer;
    saveHeaderSpecialByte = pLgMaster->gHeaderSpecialByte;
    theTransformTolerance  = pLgMaster->gArgTol;
    intColinear = 0;
    intPlanar = 0;
    pLgMaster->gBestTargetNumber = 0;
    gWorstTolReg = 1.0;

#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"respLen1 %d  2Who %x  status %02x  failsize %d", respLen, respondToWhom, pRespBuf->hdr.status, flexFailRespLen );
#endif

    //Initialize buffers
    memset(pRespBuf, 0, respLen);
    memset(pLgMaster->gBestTargetArray, 0, sizeof(pLgMaster->gBestTargetArray));
    memset((char *)&target_status, 0, sizeof(target_status));
    memset((char *)&pLgMaster->foundTarget, 0, sizeof(pLgMaster->foundTarget));
    memset((char *)&savePoint, 0, sizeof(savePoint));
    memset((char *)&XExternalAngles, 0, sizeof(XExternalAngles));
    memset((char *)&YExternalAngles, 0, sizeof(YExternalAngles));
    memset((char *)&Xarr, 0, sizeof(Xarr));
    memset((char *)&Yarr, 0, sizeof(Yarr));
    memset((char *)&pLgMaster->gColinear, 0, sizeof(pLgMaster->gColinear));
    memset((char *)&pLgMaster->gCoplanar, 0, sizeof(pLgMaster->gCoplanar));

    for (i = 0; i < MAX_TARGETSFLEX; i++)
      {
        useTarget[i] = 1;
        theAngleBuffer[i].xdata = 0;
        theAngleBuffer[i].ydata = 0;
      }
				      
    RawGeomFlag = param->angleflag;  // 1: raw binary angles, 2: geometric angles

    numberOfTargets = param->num_targets;
     
    if (RawGeomFlag == 1)
      {
        for (i = 0; i < numberOfTargets; i++)
	  {
	    theAngleBuffer[i].xdata = param->target_raw_angle[i].xangle & kMaxUnsigned;
	    theAngleBuffer[i].ydata = param->target_raw_angle[i].yangle & kMaxUnsigned;

	    if (numberOfTargets == 4)
	      {
	        theAngleBuffer[i+4].xdata = theAngleBuffer[i].xdata;
	        theAngleBuffer[i+4].ydata = theAngleBuffer[i].ydata;
	      }
	  }
      } else if (RawGeomFlag == 2)
      {
        for (i = 0; i < numberOfTargets; i++)
	  {
  	    rc = ConvertExternalAnglesToBinary(pLgMaster,
               				       param->target_geo_angle[i].Xangle,
					       param->target_geo_angle[i].Yangle,
					       &theAngleBuffer[i].xdata,
					       &theAngleBuffer[i].ydata);
#ifdef AGS_DEBUG
	    syslog(LOG_DEBUG, "FRWF binar %lf %lf  i %d",param->target_geo_angle[i].Xangle,param->target_geo_angle[i].Yangle,i);
	    syslog(LOG_DEBUG, "FRWF binar %d %d  i %d",theAngleBuffer[i].xdata,theAngleBuffer[i].ydata,i );
#endif
	    
	    if (numberOfTargets == 4)
	      {
		theAngleBuffer[i+4].xdata = theAngleBuffer[i].xdata;
		theAngleBuffer[i+4].ydata = theAngleBuffer[i].ydata;
	      }
           }
      }


    if (numberOfTargets > 4)
      {
	for (i = 0; i < numberOfTargets; i++)
          {
	    theCoordinateBuffer[(3*i)+0] = param->target[i].Xtgt;
	    theCoordinateBuffer[(3*i)+1] = param->target[i].Ytgt;
	    theCoordinateBuffer[(3*i)+2] = param->target[i].Ztgt;
          }

        SaveFullRegCoordinates (numberOfTargets, theCoordinateBuffer);
      }
    else
      {
	for (i = 0; i < 4; i++)
  	  {
	    theCoordinateBuffer[(3*i)+0] = param->target[i].Xtgt;
	    theCoordinateBuffer[(3*i)+1] = param->target[i].Ytgt;
	    theCoordinateBuffer[(3*i)+2] = param->target[i].Ztgt;

	    theCoordinateBuffer[(3*i)+12] = param->target[i].Xtgt;
	    theCoordinateBuffer[(3*i)+13] = param->target[i].Ytgt;
	    theCoordinateBuffer[(3*i)+14] = param->target[i].Ztgt;
	  }

        SaveFullRegCoordinates (8, theCoordinateBuffer);
      }
    
	

    // *****************************************************
    // if numberOfTargets is 4, change it to 8 at this point
    // *****************************************************
    
    if (numberOfTargets == 4)
      numberOfTargets = 8;

    lostSensors = 0;

    if (!rc)
      {
	for (i = 0; i < numberOfTargets; i++)
	  {
	    j = gNumberOfSensorSearchAttempts;
	    
	    gSearchCurrentSensor = i;

	    /*
	     *  allow for a variable speed search, if needed
	     */
            pLgMaster->gCoarse2Factor = pLgMaster->saveCoarse2Factor;
            pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;

            ClearSensorBuffers();  // Make sure all the buffers used by SearchForASensor are zeroed to start

	    while (j--)
	      {
                ptX = theAngleBuffer[i].xdata;
                ptY = theAngleBuffer[i].ydata;
                fndX = 0;
                fndY = 0;

#ifdef AGS_DEBUG
		syslog(LOG_DEBUG, "ptXY %x %x", ptX, ptY );
#endif
			
                if (useTarget[i] == 1)
		  {

#ifdef ZDEBUG
                    pLgMaster->debug_count++;
#endif
                    pLgMaster->current_target = i;
#ifdef NEW_TFIND
		    searchResult = FindTarget(pLgMaster, ptX, ptY, &fndX, &fndY);
#else
		    searchResult = SearchForASensor(pLgMaster, ptX, ptY, &fndX, &fndY);
#endif
		    if (searchResult == kStopWasDone)
		      {
		        SearchBeamOff(pLgMaster);
			fndX = 0;
			fndY = 0;
                      }
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
		  return;
		
		if (!searchResult)
		  break;

		pLgMaster->gCoarse2SearchStep /= 2;
		pLgMaster->gCoarse2Factor     /= 2;
		
		if (pLgMaster->gCoarse2SearchStep < kCoarseSrchStpMin)
		  {
		    pLgMaster->gCoarse2SearchStep = kCoarseSrchStpMin;
		    pLgMaster->gCoarse2Factor     = kCoarseFactorMin;
		  }
	      }
	    
	      pLgMaster->gCoarse2Factor     = kCoarseFactorDef;
	      pLgMaster->gCoarse2SearchStep = kCoarseSrchStpDef;
		
	      if (searchResult)
		{
		  lostSensors += 1U << i;
                  XfoundAngles[i] = 0;
                  YfoundAngles[i] = 0;
		}
	      else
		{
                  target_status[i] = 1;

		  pLgMaster->foundTarget[i]   = 1;

		  ConvertBinaryToGeometricAngles(pLgMaster, fndX, fndY, &(XfoundAngles[i]), &(YfoundAngles[i]));

		  ConvertBinaryToExternalAngles(pLgMaster, fndX, fndY,  &(XExternalAngles[i]), &(YExternalAngles[i]));
                }
	  }
      
	  numberOfFoundTargets = 0;

	  for (i = 0; i < numberOfTargets; i++)
	    {
	      foundAngles[2*i+0] = XfoundAngles[i];
	      foundAngles[2*i+1] = YfoundAngles[i];
	      
	      if (pLgMaster->foundTarget[i] == 1)
		numberOfFoundTargets ++;
	    }
	  
          if (numberOfFoundTargets >= 4)
	    {
	      theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror, theTransformTolerance,
					      foundAngles, (double *)&foundTransform);
            }
	  
          for (i = 0; i < numberOfTargets; i++)
	    {
              if (savePoint[i] > 0)
		{
                  target_status[i] = 2;
                }
            }
        }
    else
      {
        theResult = false;
      }

    if (theResult == true)
      {
	rc = kOK;
      }
    else
      {
        gWorstTolReg = 0.0;
        rc = kFlexFail;
      }

    /* 
     *  last desperate attempt to find transform
     *  provided that at least four targets have been found
     */
    if (gSaved == 0 && numberOfFoundTargets >= 4 && gForceTransform)
      {
        rc = kFlexFail;
	
        theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror, 0.001,
					foundAngles, (double *)&foundTransform);

	// redo with the suggested tolerance
        // now in variable gWorstTolReg
        if (theResult == true)
	  {
            useTol = gWorstTolReg;

	    theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror, useTol,
					    foundAngles, (double *)&foundTransform);
          }
	
        for (i = 0; i < numberOfTargets ; i++)
	  {
            if (savePoint[i] > 0)
	      {
                target_status[i] = 2;
              }
          }
      }
    
    if (gSaved == 0 && numberOfFoundTargets >= 4 && (saveHeaderSpecialByte & 0x20))
      {
        rc = kFlexFail;
	    
        theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror, 0.001,
                                        foundAngles, (double *)&foundTransform);

	// redo with the suggested tolerance, now in variable gWorstTolReg
        if (theResult == true)
	  {
            useTol = gWorstTolReg;
            theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror, useTol,
                                            foundAngles, (double *)&foundTransform);
          }
	    
        for (i = 0; i < numberOfTargets; i++)
	  {
            if (savePoint[i] > 0)
	      {
                target_status[i] = 2;
              }
          }
      }

    /*
     *   finally prepare and send response
     */
    for (i = 0; i < numberOfTargets; i++)
      {
        if (savePoint[i] > 0 && pLgMaster->gColinear[i] > 0)
	  {
            intColinear = intColinear | (1 << i);
          }
	    
          if (pLgMaster->gCoplanarCount > 0 && pLgMaster->gCoplanar[i] > 0 )
	    {
              intPlanar = intPlanar | (1 << i);
            }
       }

    if (theResult != true)
      {
        pRespBuf->hdr.status = RESPFLEXFAIL;
        gWorstTolReg = 0.0;
      }
    else
      {
        pRespBuf->hdr.status = RESPGOOD;
      }

    if ((saveHeaderSpecialByte & 0x20) != 0x20)
      {
        if (theResult == true)
          {
            pRespBuf->hdr.status = RESPGOOD;
          }
        else
	  {
            pRespBuf->hdr.status = RESPFLEXFAIL;
            gWorstTolReg = 0.0;
	    pFlexFailResp->colineartargets = intColinear;
	    pFlexFailResp->coplanartargets = intPlanar;
	    memcpy(pFlexFailResp, &rc, sizeof(uint32_t));
	    HandleResponse (pLgMaster, flexFailRespLen, respondToWhom);
            return;
          }
      }
    
    TransformIntoArray(&foundTransform, (double *)&pRespBuf->transform);

    if (gSaved == 0)
      {
        gBestTolAll = 0.0;
        gWorstTolAll = 0.0;
        gWorstTolReg = 0.0;
      }

    pRespBuf->besttolerance = gBestTolAll;

    pRespBuf->worsttoleranceofanycalculatedtransform = gWorstTolAll;
   
    if ( gWorstTolReg > 0 )
      {
        pRespBuf->worsttoleranceofanyintolerancetransform = gWorstTolReg;
      }
    else
      {
        pRespBuf->worsttoleranceofanyintolerancetransform = 0.0;
      }

    pRespBuf->numberoftransforms = GnOfTrans;

    pRespBuf->numberoftargets = numberOfTargets;

    for (i = 0; i < MAX_TARGETSFLEX; i++)
      {
        ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+0] = Xarr[i];
        ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+1] = Yarr[i];

#ifdef AGS_DEBUG
        syslog(LOG_DEBUG, "RSP: target #%d: xangle: %d   yangle: %d",
	       i + 1,
	       ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+0],
  	       ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+1]);
#endif
	
      }

    for (i = 0; i < MAX_TARGETSFLEX; i++)
      {
        pRespBuf->target_geo_angle[i].Xangle = XExternalAngles[i];
        pRespBuf->target_geo_angle[i].Yangle = YExternalAngles[i];
      }

    for (i = 0; i < MAX_TARGETSFLEX; i++)
      {
        pRespBuf->target_status[i] = target_status[i];
      }

    pRespBuf->colineartargets = intColinear;

    // for kOK response, only report the coplanar targets
    // if the average error is greater than 0.025
    if (gFOMavg > 0.025)
      {
        pRespBuf->coplanartargets = intPlanar;
      }

#ifdef AGS_DEBUG
    syslog(LOG_DEBUG,"respLen2 %d  2Who %x  status %02x", respLen, respondToWhom, pRespBuf->hdr.status );

    LogFlexRightOnDoFullRegWithFeedbackResponse(pRespBuf, respLen);
#endif

    HandleResponse (pLgMaster, respLen, respondToWhom);

    return;
}

#ifdef AGS_DEBUG
void LogFlexRightOnDoFullRegWithFeedbackCommand(struct parse_flexrightondofullregwithfeedback_parms *param, struct lg_master *pLgMaster)
{
    int32_t          i;
    int32_t          numberOfTargets;
    int32_t          RawGeomFlag;
 
    syslog(LOG_DEBUG, "CMD: HeaderSpecialByte: %02x", pLgMaster->gHeaderSpecialByte);

    RawGeomFlag = param->angleflag;
    syslog(LOG_DEBUG, "CMD: angleflag: %d", param->angleflag);

    numberOfTargets = param->num_targets;
    syslog(LOG_DEBUG, "CMD: num_targets: %d", param->num_targets);

    for (i = 0; i < numberOfTargets; i++)
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
	for (i = 0; i < numberOfTargets; i++)
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
	for (i = 0; i < numberOfTargets; i++)
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


void LogFlexRightOnDoFullRegWithFeedbackResponse(struct parse_flexrightondofullregwithfeedback_resp *pRespBuf, uint32_t respLen)
{
    double           transform[MAX_NEW_TRANSFORM_ITEMS];
    int32_t          i;
    int32_t          numberOfTargets;
    
    syslog(LOG_DEBUG, "RSP: hdr: %08x", htonl(pRespBuf->hdr.hdr));
    
    if (respLen <= sizeof(pRespBuf->hdr.hdr))
      return;

    memcpy(transform, pRespBuf->transform, sizeof(transform));
    for (i = 0; i < MAX_NEW_TRANSFORM_ITEMS; i++)
      {
	syslog(LOG_DEBUG, "RSP: transform[%d]: %f", i, transform[i]);
      }

    syslog(LOG_DEBUG, "RSP: besttolerance: %le", pRespBuf->besttolerance);

    syslog(LOG_DEBUG, "RSP: worsttoleranceofanycalculatedtransform: %le", pRespBuf->worsttoleranceofanycalculatedtransform);

    syslog(LOG_DEBUG, "RSP: worsttoleranceofanyintolerancetransform: %le", pRespBuf->worsttoleranceofanyintolerancetransform);

    syslog(LOG_DEBUG, "RSP: numberoftransforms: %d", pRespBuf->numberoftransforms);

    numberOfTargets = pRespBuf->numberoftargets;
    syslog(LOG_DEBUG, "RSP: numberoftargets: %d", pRespBuf->numberoftargets);

    for (i = 0; i < numberOfTargets; i++)
      {
        syslog(LOG_DEBUG, "RSP: target #%d: anglepairsX[%d]: %d   anglepairsY[%d]: %d",
	       i + 1,
	       2*i+0,
	       ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+0],
	       2*i+1,
  	       ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+1]);
      }

    for (i = 0; i < numberOfTargets; i++)
      {
        syslog(LOG_DEBUG, "RSP: target #%d: target_geo_angle[%d].Xangle: %f   target_geo_angle[%d].yangle: %f",
	       i + 1,
	       i,
	       pRespBuf->target_geo_angle[i].Xangle,
	       i,
  	       pRespBuf->target_geo_angle[i].Yangle);
      }

    for (i = 0; i < numberOfTargets; i++)
      {
	syslog(LOG_DEBUG, "RSP: target #%d: target_status[%d]: %d", i + 1, i, pRespBuf->target_status[i]);
      }

    syslog(LOG_DEBUG, "RSP: target #%d: colineartargets: %08x", i + 1, pRespBuf->colineartargets);

    syslog(LOG_DEBUG, "RSP: target #%d: coplanartargets: %08x", i + 1, pRespBuf->coplanartargets);

    return;
}
#endif
