/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include <syslog.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "FlexCalWithFeedback.h"
#include "3DTransform.h"
#include "Protocol.h"

void FlexCalWithFeedback(struct lg_master *pLgMaster, struct parse_flexcalxfdbk_parms *pInp, uint32_t respondToWhom)
{
    struct k_xyz_double    theCoordinateBuffer[MAX_TARGETSFLEX];
    struct k_xy_double     foundAngles[MAX_TARGETSFLEX];
    struct k_xy_anglepair  XYarr[MAX_TARGETSFLEX];
    int32_t                target_status[MAX_TARGETSFLEX];
    transform              foundTransform;
    struct parse_flexcalxfdbk_resp   *pResp;
    struct parse_flexcalxfdbkexp_resp   *pRespExp;
    double                 Xgeo, Ygeo;
    double                 theTransformTolerance;
    double                 angX, angY;
    double                 useTol;
    uint32_t               nTargets;
    int32_t                intColinear=0;
    int32_t                intPlanar=0;
    int16_t                Xbin, Ybin;
    uint16_t               i, j;
    uint8_t                theResult;

    // Zero for largest of 2 response buffers
    pResp = (struct parse_flexcalxfdbk_resp *)pLgMaster->theResponseBuffer;
    pRespExp = (struct parse_flexcalxfdbkexp_resp *)pLgMaster->theResponseBuffer;
    memset(pRespExp, 0, sizeof(struct parse_flexcalxfdbkexp_resp));

    // Get and validate number of targets, no sense continuing if not correct
    nTargets = pInp->num_targets;
    if ((nTargets < 4) || (nTargets > kNumberOfFlexPoints))
      {
	pResp->hdr.status = RESPFAIL;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
	return;
      }

    // Initialize variables & buffers to be used for command
    memset((char *)&theCoordinateBuffer, 0, sizeof(theCoordinateBuffer));
    memset((char *)&foundAngles, 0, sizeof(foundAngles));
    memset((char *)&XYarr, 0, sizeof(XYarr));
    memset((char *)&foundTransform, 0, sizeof(foundTransform));
    for (i = 0; i < MAX_TARGETSFLEX; i++)
       target_status[i] = 1;

    theTransformTolerance  = pLgMaster->gArgTol;
    for (i=0; i < nTargets; i++)
      {
	pLgMaster->gColinear[i] = 0;
	pLgMaster->foundTarget[i] = 1;
	angX = pInp->tgtAngle[i].Xangle;
	angY = pInp->tgtAngle[i].Yangle;
	ConvertExternalAnglesToMirror(angX, angY, &Xgeo, &Ygeo);
	ConvertMirrorToGeometricAngles(&Xgeo, &Ygeo);
	foundAngles[i].Xangle = Xgeo;
	foundAngles[i].Yangle = Ygeo;
	ConvertExternalAnglesToBinary(pLgMaster, angX, angY, &Xbin, &Ybin);
	XYarr[i].xangle = Xbin;
	XYarr[i].yangle = Ybin;
      } 
    for (i=0; i < nTargets; i++)
      {
	theCoordinateBuffer[i].Xtgt = pInp->target[i].Xtgt;
	theCoordinateBuffer[i].Ytgt = pInp->target[i].Ytgt;
	theCoordinateBuffer[i].Ztgt = pInp->target[i].Ztgt;
      }
    SaveFullRegCoordinates(nTargets, (double *)&theCoordinateBuffer);
    //
    // if gHeaderSpecialByte is NOT 0x40,
    // then reject duplicate targets
    //
    if (!(pLgMaster->gHeaderSpecialByte & 0x40))
      {
	for (i = 0; i < nTargets; i++)
	  {
	    for (j = i; j < nTargets; j++)
	      {
		if (i != j)
		  {
		    if ((gX[i]==gX[j]) && (gY[i]==gY[j]) && (gZ[i]==gZ[j]))
		      pLgMaster->foundTarget[j] = 0;
		  }
	      }
	  }
      }

#ifdef ZDEBUG
syslog( LOG_NOTICE, "about to FindTransformMatrix" );
#endif

    theResult = FindTransformMatrix(pLgMaster, nTargets, gDeltaMirror, theTransformTolerance,
				    (double *)&foundAngles, (double *)&foundTransform);

#ifdef ZDEBUG
syslog( LOG_NOTICE, "FindTransformMatrix result %d", theResult );
#endif

    /* desperate attempt to get something */
    if ((gSaved == 0) && gForceTransform)
      {
	pResp->hdr.status = RESPFLEXFAIL;
#ifdef ZDEBUG
syslog( LOG_NOTICE, "bad about to FindTransformMatrix" );
#endif
	theResult = FindTransformMatrix(pLgMaster, nTargets, gDeltaMirror, 0.001,
					(double *)&foundAngles, (double *)&foundTransform);
#ifdef ZDEBUG
syslog( LOG_NOTICE, "FindTransformMatrix result %d", theResult );
#endif
	// redo with the suggested tolerance
	// now in variable gWorstTolReg
	if (theResult == true)
	  {
	    useTol = gWorstTolReg;
#ifdef ZDEBUG
syslog( LOG_NOTICE, "worst about to FindTransformMatrix" );
#endif
	    theResult = FindTransformMatrix(pLgMaster, nTargets, gDeltaMirror, useTol,
					    (double *)&foundAngles, (double *)&foundTransform);
#ifdef ZDEBUG
syslog( LOG_NOTICE, "FindTransformMatrix result %d", theResult );
#endif
	  }
	GnOfTrans = 0;
	theResult = 0;
      }

    if (gSaved == 0)
      {
	pResp->hdr.status = RESPFLEXFAIL;
#ifdef ZDEBUG
syslog( LOG_NOTICE, "about to FAIL FindTransformMatrix" );
#endif
	theResult = FindTransformMatrix(pLgMaster, nTargets, gDeltaMirror, 0.001,
					(double *)&foundAngles, (double *)&foundTransform);
#ifdef ZDEBUG
syslog( LOG_NOTICE, "FindTransformMatrix result %d", theResult );
#endif
	// redo with the suggested tolerance
	// now in variable gWorstTolReg
	if (theResult == true)
	  {
	    useTol = gWorstTolReg;
#ifdef ZDEBUG
syslog( LOG_NOTICE, "about to worst fail FindTransformMatrix" );
#endif
	    theResult = FindTransformMatrix(pLgMaster, nTargets, gDeltaMirror, useTol,
					    (double *)&foundAngles, (double *)&foundTransform);
#ifdef ZDEBUG
syslog( LOG_NOTICE, "FindTransformMatrix result %d", theResult );
#endif
	  }
	theResult = 0;
      }

    for (i = 0; i < nTargets ; i++)
      {
	if (pLgMaster->gColinear[i] > 0)
	  intColinear = intColinear | (1 << i);
      }

	if (theResult)
	  pResp->hdr.status = RESPGOOD;
	else
	  pResp->hdr.status = RESPFLEXFAIL;

	TransformIntoArray(&foundTransform, (double *)&pResp->transform[0]);

	memset((char *)&pResp->anglepairs[0], 0xFF, ANGLEPAIRSLENFLEX);
	memcpy((char *)&pResp->anglepairs[0], (char *)&XYarr[0], sizeof(XYarr));

	pResp->num_xfrms = GnOfTrans;

	pResp->num_tgts = nTargets;

	pResp->colineartgt = intColinear;

	pResp->coplanartgts = intPlanar;

	for (i=0; i < nTargets; i++)
	  {
	    if (savePoint[i] > 0)
	      target_status[i] = 2;
	    pResp->tgt_status[i] = target_status[i];
	  }

        if (gSaved > 0)
	  {
	    pRespExp->bestTolerance = gBestTolAll;
            pRespExp->worstTolOfAnyCalcXfrm = gWorstTolAll;
            pRespExp->worstTolOfAnyInTolXfrm = gWorstTolReg;
	  }

        if (pLgMaster->gHeaderSpecialByte & 0x20)
	  {
	    HandleResponse(pLgMaster, (sizeof(struct parse_flexcalxfdbkexp_resp)-kCRCSize), respondToWhom);
	    return;
	  }
	else
	  {
	    HandleResponse(pLgMaster, (sizeof(struct parse_flexcalxfdbk_resp)-kCRCSize), respondToWhom);
	    return;
	  }

	return;
}
