/*
static char rcsid[] = "$Id: FullRegWithFeedback.c,v 1.1 2006/06/05 18:40:30 ags-sw Exp ags-sw $";

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
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "3DTransform.h"
#include "L3DTransform.h"
#include "BoardComm.h"
#include "parse_data.h"
#include "FullRegWithFeedback.h"

#define	kNumberOfSensorSearchAttempts 3

void FullRegWithFeedback(struct lg_master *pLgMaster,
			 struct parse_rightondofullregwithfeedback_parms *param,
			 uint32_t respondToWhom)
{
    struct parse_rightondofullregwithfeedback_resp *pRespBuf;
    struct lg_xydata theAngleBuffer[MAX_TARGETSOLD];
    transform        foundTransform;
    double           foundAngles[MAX_TARGETSOLD * 2];
    double           theCoordinateBuffer[MAX_TARGETSOLD * 3];
    double           theTransformTolerance;
    double           XExternalAngles[MAX_TARGETSOLD];
    double           YExternalAngles[MAX_TARGETSOLD];
    double           XfoundAngles[MAX_TARGETSOLD];
    double           YfoundAngles[MAX_TARGETSOLD];
    int32_t          numberOfTargets;
    int32_t          RawGeomFlag;
    int32_t          target_status[MAX_TARGETSOLD];
    int              numberOfFoundTargets = 0;
    int              searchResult;
    int              useTarget[MAX_TARGETSOLD];
    uint32_t         respLen = (sizeof(struct parse_rightondofullregwithfeedback_resp)-kCRCSize);
    uint32_t         rc = 0;
    uint32_t         lostSensors;
    int16_t          fndX, fndY;
    int16_t          Xarr[MAX_TARGETSOLD];
    int16_t          Yarr[MAX_TARGETSOLD];
    int16_t          ptX, ptY;
    unsigned short   i, j;
    unsigned char    theResult;
   
    SlowDownAndStop(pLgMaster);
    pRespBuf = (struct parse_rightondofullregwithfeedback_resp *)pLgMaster->theResponseBuffer;
    
    theTransformTolerance  = pLgMaster->gArgTol;
    
    pLgMaster->gBestTargetNumber = 0;
    
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

    for (i = 0; i < MAX_TARGETSOLD; i++)
      {
        useTarget[i] = 1;
      }
				      
    gWorstTolReg = 1.0;
	
    RawGeomFlag = param->angleflag;

    numberOfTargets = param->num_targets;  // 1: raw binary angles, 2: Geometric angles

    if (RawGeomFlag == 1)
      {
        for (i = 0; i < numberOfTargets; i++)
	  {
	    theAngleBuffer[i].xdata = param->target_raw_angle[i].xangle & kMaxUnsigned;
	    theAngleBuffer[i].ydata = param->target_raw_angle[i].yangle & kMaxUnsigned;
	  }
      }
    else if (RawGeomFlag == 2)
	  {
	    for (i = 0; i < numberOfTargets; i++)
	      {
		rc = ConvertExternalAnglesToBinary(pLgMaster,
						   param->target_geo_angle[i].Xangle,
						   param->target_geo_angle[i].Yangle,
						   &theAngleBuffer[i].xdata,
						   &theAngleBuffer[i].ydata);
	      }
          }

    for (i = 0; i < numberOfTargets; i++)
      {
        theCoordinateBuffer[(3*i)+0] = param->target[i].Xtgt;
	theCoordinateBuffer[(3*i)+1] = param->target[i].Ytgt;
	theCoordinateBuffer[(3*i)+2] = param->target[i].Ztgt;
      }

    SaveFullRegCoordinates (numberOfTargets, theCoordinateBuffer);

    for (i = 0; i < numberOfTargets; i++)
      {
        for (j = i; j < numberOfTargets; j++)
	  {
            if ( i != j )
	      {
                if ((gX[i]==gX[j]) && (gY[i]==gY[j]) && (gZ[i]==gZ[j]))
		  useTarget[j] = 0;
	      }
	  }
      }
    
    lostSensors = 0;

    if (rc == 0)
      {
	for (i = 0; i < numberOfTargets; i++)
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
		if (useTarget[i] == 1)
		  {
		    ptX = theAngleBuffer[i].xdata;
		    ptY = theAngleBuffer[i].ydata;

		    searchResult = SearchForASensor(pLgMaster, ptX, ptY, &fndX, &fndY );
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
	        if (pLgMaster->gCoarse2SearchStep < 1)
	          {
	    	    pLgMaster->gCoarse2SearchStep = 1;
		    pLgMaster->gCoarse2Factor     = 1;
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

		pLgMaster->foundTarget[i] = 1;

		ConvertBinaryToGeometricAngles(pLgMaster, fndX, fndY, &(XfoundAngles[i]), &(YfoundAngles[i]) );

		ConvertBinaryToExternalAngles(pLgMaster, fndX, fndY, &(XExternalAngles[i]), &(YExternalAngles[i]) );
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
	      theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror,
					      theTransformTolerance, foundAngles,
					      (double *)&foundTransform);
	    }

          for (i = 0; i < numberOfTargets; i++)
	    {
              if (savePoint[i] > 0)
		target_status[i] = 2;
	    }
      }
    else
      theResult = false;

      if (gTargetDrift)
	InitDrift(Xarr, Yarr);

      /* 
       *  last desperate attempt to find transform
       *  provided that at least four targets have been found
       */
      if ((gSaved == 0) && (numberOfFoundTargets >= 4) && gForceTransform)
	{
	  theResult = FindTransformMatrix(pLgMaster, numberOfTargets, gDeltaMirror, 0.001,
			  	          foundAngles, (double *)&foundTransform);
	}

      /*
       *   finally prepare and send response
       */

        if (theResult == true)
	  {
	    pRespBuf->hdr.status3 = RESPGOOD;
	    pRespBuf->hdr.numTransforms = (unsigned char)(0xFF & GnOfTrans);
	  }
	else
	  {
            pRespBuf->hdr.status3 = RESPFAIL;
	    pRespBuf->hdr.hdr |= htonl(lostSensors);
	    HandleResponse (pLgMaster, sizeof(pRespBuf->hdr), respondToWhom);
            return;
	  }

        TransformIntoArray( &foundTransform, (double *)pRespBuf->transform);

        pRespBuf->besttolerance = gBestTolAll;

        pRespBuf->worsttoleranceofanycalculatedtransform = gWorstTolAll;

        if ( gWorstTolReg > 0 )
	  {
            pRespBuf->worsttoleranceofanyintolerancetransform = gWorstTolReg;
          }
	else
	  {
             pRespBuf->worsttoleranceofanyintolerancetransform = 1.0;
          }

        pRespBuf->numberoftransforms = GnOfTrans;

	pRespBuf->numberoftargets = numberOfTargets;

	for (i = 0; i < numberOfTargets; i++)
	  {
	    ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+0] = Xarr[i];
	    ((uint32_t *)(&(pRespBuf->anglepairs[0])))[2*i+1] = Yarr[i];
	  }

	for (i = 0; i < numberOfTargets; i++)
	  {
	    pRespBuf->target_geo_angle[i].Xangle = XExternalAngles[i];
	    pRespBuf->target_geo_angle[i].Yangle = YExternalAngles[i];
	  }

	for (i = 0; i < numberOfTargets; i++)
	  {
	    pRespBuf->target_status[i] = target_status[i];
	  }

	HandleResponse(pLgMaster, respLen, respondToWhom);
	return;
}
