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
#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "LaserInterface.h"
#include "SensorRegistration.h"
#include "SensorSearch.h"
#include "FlexCalculateTransform.h"
#include "3DTransform.h"
#include "Protocol.h"

void FlexCalculateTransform(struct lg_master *pLgMaster, struct parse_flexcalxfrm_parms *pInp, uint32_t respondToWhom)
{
    struct k_xyz_double    theCoordinateBuffer[MAX_TARGETSFLEX];
    struct k_xy_double     foundAngles[MAX_TARGETSFLEX];
    struct k_xy_anglepair  XYarr[MAX_TARGETSFLEX];
    transform              foundTransform;
    struct parse_flexcalxfrm_resp   *pResp;
    double                 Xgeo, Ygeo;
    double                 theTransformTolerance;
    double                 angX, angY;
    uint32_t               nTargets;
    int32_t                intColinear;
    int32_t                intPlanar;
    int16_t                Xbin, Ybin;
    uint16_t               i, j;
    uint8_t                theResult;

    pResp = (struct parse_flexcalxfrm_resp *)pLgMaster->theResponseBuffer;
    memset(pLgMaster->theResponseBuffer, 0, sizeof(struct parse_flexcalxfrm_resp));

    // Get and validate num_targets from sender, no sense in continuing if not correct!
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
    intColinear = 0;
    intPlanar = 0;
    theTransformTolerance  = pLgMaster->gArgTol;

    // Get the target-angle pairs
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
    theResult = FindTransformMatrix(pLgMaster, nTargets, gDeltaMirror, theTransformTolerance,
				    (double *)&foundAngles, (double *)&foundTransform);

    /* desperate attempt to get something */
    if ((gSaved == 0) && gForceTransform)
      {
	theResult = FindTransformMatrix(pLgMaster, nTargets, gDeltaMirror,
					0.00001, (double *)&foundAngles, (double *)&foundTransform);
	GnOfTrans = 0;
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

    HandleResponse(pLgMaster, (sizeof(struct parse_flexcalxfrm_resp)-kCRCSize), respondToWhom);

    return;
}
