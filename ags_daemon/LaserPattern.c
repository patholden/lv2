// static char rcsid[] = "$Id: LaserPattern.c,v 1.11 2001/12/27 22:47:03 ags-sw Exp ags-sw $";

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include <syslog.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "AppErrors.h"
#include "parse_data.h"
#include "LaserPattern.h"
#include "AppStrListIDs.h"
#include "LaserInterface.h"
#include "3DTransform.h"
#include "BoardCommDefines.h"
#include "segtest.h"
#include "segpoly.h"
#include "pnpoly.h"
#include "tan_init.h"

#define	kX 0
#define	kY 1
#define	kZ 2

   //  tan of slightly over 30 degrees (30.0021)
#define TanLimit  0.5774

enum
{
	kInitPatternBufferErr = 1,
	kInitializingPatternInterfaceMsg ,
	kInitVectorBufferErr
};
#if _NEW_PATH_
struct APTVector
{
    double        x, y, z;
    int32_t       ax, ay;
    unsigned char on;
};
#endif
double	gBeamLinearRangeX = 48.0;
double	gBeamLinearRangeY = 48.0;

double gMaxCos = 0.998;
double gLongToShort = 5.0;

double gCurveMax = 2.0;
double gCurveMin = 0.25;
double gCurveCos = 0.99996;
static	unsigned char gBeamOn = true;
static	unsigned char gItIsStart = true;
static	unsigned char gPenDownPending;
static	unsigned char gPendPenDown;
static	double        gDefaultZ = 0.0;
static struct lg_xydata *pCurXYData=0;
static struct lg_xydata *pXYPoints=0;
static	transform     gCurrentTransform;
static	double	      gStartPoint[3], gOldPoint[3];
static	double	      gPutPoint[3];
static  short         gOldFastStepsNumber;
#ifndef _NEW_PATH_
static	double	      gOldPutPoint[3];
static	short	      gOldSlowStepsNumber;
static	double        gOldPathFractionUnit;
#endif
static	double        gOldAcosX, gOldAcosY, gOldAcosZ;
static	unsigned char gNowInside = true;
static	unsigned char gOldBeamOn;
static	unsigned char gRawInput = false;

static uint32_t PutPointsEndingAt(struct lg_master *pLgMaster, double *newPoint);
static uint32_t PutSequenceOfPoints(struct lg_master *pLgMaster, double *theEndPoint);
static unsigned char WithinLaserLinearRange(double *point);
static unsigned char WithinLaserAngleRange(double *point);
static void GetLinearBorderPoint(double *pointInside, double *pointOutside, double *borderPoint);
static unsigned char AngleBorderPointsBetween(double *startPoint, double *endPoint,
					      double *entrancePoint, double *exitPoint);
static void GetAngleBorderPoint(double *pointInside, double *pointOutside, double *borderPoint);
static unsigned char LinearBorderPointsBetween(double *startPoint, double *endPoint,
					       double *entrancePoint, double *exitPoint);
static int findarc(struct lg_master *pLgMaster, double p1[3], double p2[3], double p3[3], double Dinterp );
static uint32_t PutAPoint(struct lg_master *pLgMaster, int32_t x, int32_t y );
#if _NEW_PATH_
static	uint32_t ProcessStoredPoints (struct lg_master *pLgMaster);
static	short				gNumberOfStoredPoints = 0;
static	struct APTVector	*gPointStorage = 0;
#endif


uint32_t PutGoTo2D (struct lg_master *pLgMaster, double x, double y )
{
  return PutGoTo3D (pLgMaster, x, y, gDefaultZ );
}

uint32_t PutGoTo3D (struct lg_master *pLgMaster, double x, double y, double z )
{
	uint32_t tempLong;
	double tempPoint[3], addedPoint[3];
	
	SetDefaultZ ( z );
	
	tempPoint[kX] = x;
	tempPoint[kY] = y;
	tempPoint[kZ] = z;

	TransformPoint ( &gCurrentTransform, tempPoint, addedPoint );
	

	if ( gItIsStart )
	{
		/*
		 * gItIsStart == true && gNowInside == false
                 * means that there is a gOldPoint
                 * outside the laser beam range
                 */
		if ( WithinLaserAngleRange ( addedPoint ) )
		{
		    if ( WithinLaserLinearRange ( addedPoint ) )
		    {
			gItIsStart = false;
			if ( gNowInside )
			{
				gStartPoint[kX] = addedPoint[kX];
				gStartPoint[kY] = addedPoint[kY];
				gStartPoint[kZ] = addedPoint[kZ];
				gPutPoint[kX] = gStartPoint[kX];
				gPutPoint[kY] = gStartPoint[kY];
				gPutPoint[kZ] = gStartPoint[kZ];
				gOldPoint[kX] = gStartPoint[kX];
				gOldPoint[kY] = gStartPoint[kY];
				gOldPoint[kZ] = gStartPoint[kZ];
				if (gPendPenDown)
				  UnpendPenDown();
				return(0);
			}
			else
			{
			  gNowInside = true;
			  GetLinearBorderPoint(addedPoint, gOldPoint, gStartPoint);
			  if (!(WithinLaserAngleRange(gStartPoint)))
			    GetAngleBorderPoint(addedPoint, gOldPoint, gStartPoint);
			  gPutPoint[kX] = gStartPoint[kX];
			  gPutPoint[kY] = gStartPoint[kY];
			  gPutPoint[kZ] = gStartPoint[kZ];
			  gOldPoint[kX] = gStartPoint[kX];
			  gOldPoint[kY] = gStartPoint[kY];
			  gOldPoint[kZ] = gStartPoint[kZ];
			}
		    } else {
			if (!gNowInside &&
			    (LinearBorderPointsBetween(gOldPoint, addedPoint, gStartPoint, tempPoint)))
			  {
			    if (!(WithinLaserAngleRange(gStartPoint)))
			      AngleBorderPointsBetween(gOldPoint, addedPoint, gStartPoint, tempPoint);
			    gNowInside = true;
			    gItIsStart = false;
			    gPutPoint[kX] = gStartPoint[kX];
			    gPutPoint[kY] = gStartPoint[kY];
			    gPutPoint[kZ] = gStartPoint[kZ];
			    gOldPoint[kX] = gStartPoint[kX];
			    gOldPoint[kY] = gStartPoint[kY];
			    gOldPoint[kZ] = gStartPoint[kZ];
			}
			else
			{
			  gNowInside = false;
			  gOldPoint[kX] = addedPoint[kX];
			  gOldPoint[kY] = addedPoint[kY];
			  gOldPoint[kZ] = addedPoint[kZ];
			  if (gPendPenDown)
			    UnpendPenDown();
			  return(0);
			}
                    }
		}
		else
		{
			if (!gNowInside &&
			    AngleBorderPointsBetween(gOldPoint, addedPoint, gStartPoint, tempPoint))
			{
			  if (!(WithinLaserLinearRange(gStartPoint)))
			    LinearBorderPointsBetween(gOldPoint, addedPoint, gStartPoint, tempPoint);
			  gNowInside = true;
			  gItIsStart = false;
			  gPutPoint[kX] = gStartPoint[kX];
			  gPutPoint[kY] = gStartPoint[kY];
			  gPutPoint[kZ] = gStartPoint[kZ];
			  gOldPoint[kX] = gStartPoint[kX];
			  gOldPoint[kY] = gStartPoint[kY];
			  gOldPoint[kZ] = gStartPoint[kZ];
			}
			else
			{
			  gNowInside = false;
			  gOldPoint[kX] = addedPoint[kX];
			  gOldPoint[kY] = addedPoint[kY];
			  gOldPoint[kZ] = addedPoint[kZ];
			  if (gPendPenDown)
			    UnpendPenDown();
			  return(0);
			}
		}
	}
	tempLong = PutPointsEndingAt (pLgMaster, addedPoint );
	
	gOldPoint[kX] = addedPoint[kX];
	gOldPoint[kY] = addedPoint[kY];
	gOldPoint[kZ] = addedPoint[kZ];
	
	if ( tempLong ) return tempLong;
	
	if (gPendPenDown)
	  UnpendPenDown();
	return(0);
}


uint32_t FinishPattern (struct lg_master *pLgMaster)
{
#if _NEW_PATH_
	uint32_t tempLong;
#endif
	if ( gItIsStart ) return 0U;
	if (gRawInput)
	  {
#ifdef AGS_DEBUG
	    syslog(LOG_DEBUG,"gRawInput set");
#endif
	    return(0);
	  }
	UnpendPenDown();
	SetPenUp();
	gNowInside = true;
#if _NEW_PATH_
	tempLong = PutPointsEndingAt (pLgMaster, gStartPoint );
	if ( tempLong ) return tempLong;
	return ProcessStoredPoints (pLgMaster);
#else
	return PutPointsEndingAt (pLgMaster, gStartPoint );
#endif
}

static uint32_t PutPointsEndingAt (struct lg_master *pLgMaster, double *newPoint )
{
	uint32_t tempLong;
	unsigned char beamWasOn;
	double *theEndPoint, dummyPoint[3], sillyPoint[3];

	theEndPoint = dummyPoint;
	if ( gNowInside )
	{
	  if ( ! WithinLaserAngleRange ( newPoint ) )
	    {
	      theEndPoint = dummyPoint;
	      GetAngleBorderPoint ( gOldPoint, newPoint, theEndPoint );
	      if ( ! WithinLaserLinearRange( theEndPoint ) ) {
		theEndPoint = dummyPoint;
		GetLinearBorderPoint(gOldPoint, newPoint, theEndPoint);
	      }
	      gNowInside = false;
	    } else if ( ! WithinLaserLinearRange ( newPoint ) ) {
	    theEndPoint = dummyPoint;
	    GetLinearBorderPoint(gOldPoint, newPoint, theEndPoint);
	    gNowInside = false;
	  } else
	    theEndPoint = newPoint;
	}
	else
	  {
	    if ( WithinLaserAngleRange ( newPoint ) )
	      {
		if ( WithinLaserLinearRange ( newPoint ) )
		  {
		    theEndPoint = newPoint;
		    GetAngleBorderPoint ( newPoint, gOldPoint, dummyPoint );
		    
		    if ( ! WithinLaserLinearRange( dummyPoint ) ) {
		      GetLinearBorderPoint(newPoint, gOldPoint, dummyPoint);
		    }
		    
		    beamWasOn = gBeamOn;
		    SetPenUp();
		    tempLong = PutSequenceOfPoints (pLgMaster, dummyPoint);
		    if ( tempLong ) return tempLong;
		    gBeamOn = beamWasOn;
		    
		    gNowInside = true;
		    gOldPoint[kX] = dummyPoint[kX];
		    gOldPoint[kY] = dummyPoint[kY];
		    gOldPoint[kZ] = dummyPoint[kZ];
		  } else {
		  if ( LinearBorderPointsBetween(gOldPoint, newPoint, sillyPoint, theEndPoint))
		    {
		      beamWasOn = gBeamOn;
		      SetPenUp();
		      tempLong = PutSequenceOfPoints (pLgMaster, sillyPoint );
		      if ( tempLong ) return tempLong;
		      gBeamOn = beamWasOn;
		      
		      gOldPoint[kX] = sillyPoint[kX];
		      gOldPoint[kY] = sillyPoint[kY];
		      gOldPoint[kZ] = sillyPoint[kZ];
		    }
		  else
		    return(0);
		}
	      } else {
	      theEndPoint = dummyPoint;
	      if (AngleBorderPointsBetween(gOldPoint, newPoint, sillyPoint, theEndPoint))
		{
		  beamWasOn = gBeamOn;
		  SetPenUp();
		  if ( ! WithinLaserLinearRange( sillyPoint ) ) {
		    LinearBorderPointsBetween ( gOldPoint
						, newPoint
						, sillyPoint
						, theEndPoint
						);
		  }
		  tempLong = PutSequenceOfPoints (pLgMaster, sillyPoint );
		  if ( tempLong ) return tempLong;
		  gBeamOn = beamWasOn;
		  
		  gOldPoint[kX] = sillyPoint[kX];
		  gOldPoint[kY] = sillyPoint[kY];
		  gOldPoint[kZ] = sillyPoint[kZ];
		}
	      else return 0U;
	    }
	  }
	return PutSequenceOfPoints (pLgMaster, theEndPoint );
}


#if _NEW_PATH_
uint32_t ProcessStoredPoints(struct lg_master *pLgMaster)
{
    double pt1[3];
    double pt2[3];
    double pt3[3];
    struct APTVector *currPoint, *prevPoint, *nextPoint;
    double cosXin, cosYin, cosXout, cosYout, cosIn, cosOut;
    double pathFraction, pathPoint[3], distLD, cosInOut;
    double shortSegment;
    double DXcp, DYcp, DZcp, Lcp;
    double DXcn, DYcn, DZcn, Lcn;
    double Ldot;
    uint32_t tempLong, theError;
    uint32_t slowDownStep = SlowDownStep();
    uint32_t fastStep = FastStep();
    int32_t dist, maxSlowDownDist, theSlowDownDist;
    int32_t distXin, distXout, distYin, distYout;
    int16_t   pathX, pathY;
    int32_t distSlow;
    int32_t axp, ayp, axc, ayc, axn, ayn;
    uint16_t currIndex, prevIndex, nextIndex, pilePoints;
    uint16_t i, numSlowSteps, numFastSteps, fastToSlowRatio;
    unsigned char fChangeBeam, fSlowDown, fMiddlePoints;

    if (gNumberOfStoredPoints == 0)
      return(0);

    fastToSlowRatio = fastStep / slowDownStep;
    maxSlowDownDist = ((fastToSlowRatio * (fastToSlowRatio + 1)) >> 1) * slowDownStep;
    shortSegment = (double)(slowDownStep << 2);

    /* 
     *  since the pattern can go OutOfRange,
     *  need to set theError intially to 0
     */
    theError = 0;
    currIndex = 0;
    do
      {
	if (currIndex)
	  prevIndex = currIndex - 1;
	else
	  prevIndex = gNumberOfStoredPoints - 1;
	nextIndex = (currIndex + 1) % gNumberOfStoredPoints;
	prevPoint = &gPointStorage[prevIndex];
	currPoint = &gPointStorage[currIndex];
	nextPoint = &gPointStorage[nextIndex];
	axp = prevPoint->ax - kMinSigned;
	ayp = prevPoint->ay - kMinSigned;
	axc = currPoint->ax - kMinSigned;
	ayc = currPoint->ay - kMinSigned;
	axn = nextPoint->ax - kMinSigned;
	ayn = nextPoint->ay - kMinSigned;

	DXcp = currPoint->x - prevPoint->x;
	DYcp = currPoint->y - prevPoint->y;
	DZcp = currPoint->z - prevPoint->z;
	Lcp  = sqrt( DXcp*DXcp + DYcp*DYcp + DZcp*DZcp );

	DXcn = currPoint->x - nextPoint->x;
	DYcn = currPoint->y - nextPoint->y;
	DZcn = currPoint->z - nextPoint->z;
	Lcn  = sqrt( DXcn*DXcn + DYcn*DYcn + DZcn*DZcn );

	if ((Lcp > 0.0) && (Lcn > 0.0))
	  Ldot = fabs(DXcp*DXcn + DYcp*DYcn + DZcp*DZcn) / (Lcp*Lcn);
	else
	  Ldot = 1.0;
	if (axc > axp)
	  {
	    distXin = axc - axp;
	    cosXin = (double)distXin;
	  }
	else
	  {
	    distXin = axp - axc;
	    cosXin = -(double)distXin;
	  }
	if (ayc > ayp)
	  {
	    distYin = ayc - ayp;
	    cosYin = (double)distYin;
	  }
	else
	  {
	    distYin = ayp - ayc;
	    cosYin = -(double)distYin;
	  }
	if (axn > axc)
	  {
	    distXout = axn - axc;
	    cosXout = (double)distXout;
	  }
	else
	  {
	    distXout = axc - axn;
	    cosXout = -(double)distXout;
	  }
	if (ayn > ayc)
	  {
	    distYout = ayn - ayc;
	    cosYout = (double)distYout;
	  }
	else
	  {
	    distYout = ayc - ayn;
	    cosYout = -(double)distYout;
	  }
	cosIn = sqrt ( cosXin * cosXin + cosYin * cosYin );
	cosOut = sqrt ( cosXout * cosXout + cosYout * cosYout );
	if (cosIn < DBL_MIN)
	  {
	    if (cosOut < DBL_MIN)
	      {
		fSlowDown = true;
		cosXin = -1.0;
		cosYin = 0.0;
		cosXout = 1.0;
		cosYout = 0.0;
	      }
	    else
	      {
		fSlowDown = true;
		cosXout /= cosOut;
		cosYout /= cosOut;
		cosXin = - cosXout;
		cosYin = - cosYout;
	      }
	  }
	else
	  {
	    if (cosOut < DBL_MIN)
	      {
		fSlowDown = true;
		cosXin /= cosIn;
		cosYin /= cosIn;
		cosXout = - cosXin;
		cosYout = - cosYin;
	      }
	    else
	      {
		fSlowDown = false;
		cosXin /= cosIn;
		cosYin /= cosIn;
		cosXout /= cosOut;
		cosYout /= cosOut;
	      }
	  }
	cosInOut = cosXin * cosXout + cosYin * cosYout;
	fChangeBeam = (currPoint->on && !nextPoint->on) || (!currPoint->on && nextPoint->on);

	//  first check for curve interpolation
	//     - make sure that the real-space line
	//       is curved as well (test Ldot)
	if (!fSlowDown && !fChangeBeam && (Ldot < gCurveCos)
	    && (Lcp > gCurveMin) && (Lcp < gCurveMax)
	    && (Lcn > gCurveMin) && (Lcn < gCurveMax)
	    && (cosInOut < gCurveCos) && (cosInOut >= gMaxCos))
	  {
	    pt1[0] = prevPoint->x;
	    pt1[1] = prevPoint->y;
	    pt1[2] = prevPoint->z;
	    pt2[0] = currPoint->x;
	    pt2[1] = currPoint->y;
	    pt2[2] = currPoint->z;
	    pt3[0] = nextPoint->x;
	    pt3[1] = nextPoint->y;
	    pt3[2] = nextPoint->z;
	    gBeamOn = currPoint->on;
	    theError = findarc(pLgMaster, pt1, pt2, pt3, gCurveMin); 
	    if (theError)
	      return(theError);
	    continue;
	  }
	if (!fSlowDown)
	  fSlowDown = fChangeBeam || (cosInOut < gMaxCos)
	    || ((cosIn / cosOut) > gLongToShort) || ((cosOut / cosIn) > gLongToShort);

	pilePoints = PilePoints ( fChangeBeam, fSlowDown, cosInOut );
/* */
	if ((pilePoints > 1) && (cosIn < shortSegment) && (cosOut < shortSegment))
	  fSlowDown = false;
	if (fSlowDown)
	  theSlowDownDist = maxSlowDownDist;
	else
	  theSlowDownDist = 0;
	gBeamOn = currPoint->on;
	if (distXin > distYin)
	  dist = distXin;
	else
	  dist = distYin;
	distLD = (double)dist;
	distSlow = dist >> 1;
	if (distSlow < theSlowDownDist)
	  numFastSteps = 0;
	else
	  numFastSteps = (distSlow - theSlowDownDist) / fastStep;
	fMiddlePoints = false;
	if (!fSlowDown)
	  numSlowSteps = 0;
	else
	  {
	    if (distSlow > theSlowDownDist)
	      numSlowSteps = fastToSlowRatio;
	    else
	      {
		numSlowSteps = 0;
		while (distSlow >= 0) 
		  {
		    numSlowSteps++;
		    distSlow -= numSlowSteps * slowDownStep;
		  }
		numSlowSteps--;
	      }
	    if (dist && (numFastSteps == 0))
	      {
		if (numSlowSteps)
		  numSlowSteps--;
		fMiddlePoints = true;
	      }
	  }
	if (numFastSteps || (!fSlowDown && (distLD > (double)fastStep)))
	  {
	    if (numFastSteps)
	      numFastSteps--;
	    pathFraction = .50 - (distLD - 2.0 * ((double)theSlowDownDist + ((double)numFastSteps * (double)fastStep)))
				/ (6.0 * distLD);
	    pathPoint[kX] = currPoint->x * (1.0 - pathFraction) + pathFraction * prevPoint->x;
	    pathPoint[kY] = currPoint->y * (1.0 - pathFraction) + pathFraction * prevPoint->y;
	    pathPoint[kZ] = currPoint->z * (1.0 - pathFraction) + pathFraction * prevPoint->z;
	    theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY );
	    if (theError)
	      {
		pLgMaster->gOutOfRange.errtype1 = RESPE1PATANGLEOUTOFRANGE;
		pLgMaster->gOutOfRange.errtype2 = (uint16_t)(theError & 0xFFFF);
	      }
	    else
	      {
		tempLong = PutAPoint (pLgMaster, pathX, pathY );
		if (tempLong)
		  return(tempLong);
	      }
	  }
	i = numFastSteps;
	while (i)
	  {
	    pathFraction =  ((double)theSlowDownDist + ((double)i * (double)fastStep)) / distLD;
	    pathPoint[kX] = currPoint->x * (1.0 - pathFraction) + pathFraction * prevPoint->x;
	    pathPoint[kY] = currPoint->y * (1.0 - pathFraction) + pathFraction * prevPoint->y;
	    pathPoint[kZ] = currPoint->z * (1.0 - pathFraction) + pathFraction * prevPoint->z;
	    theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY);
	    if (theError)
	      {
		pLgMaster->gOutOfRange.errtype1 = RESPE1PATANGLEOUTOFRANGE;
		pLgMaster->gOutOfRange.errtype2 = (uint16_t)(theError & 0xFFFF);
	      }
	    else
	      {
		tempLong = PutAPoint(pLgMaster, pathX, pathY);
		if (tempLong)
		  return(tempLong);
	      }
	    i--;
	  }
	if (fMiddlePoints)
	  {
	    pathFraction = .50 - (distLD - ((double)numSlowSteps * (double)((numSlowSteps + 1) * slowDownStep)))
	      / (6.0 * distLD);
	    pathPoint[kX] = currPoint->x * (1.0 - pathFraction) + pathFraction * prevPoint->x;
	    pathPoint[kY] = currPoint->y * (1.0 - pathFraction) + pathFraction * prevPoint->y;
	    pathPoint[kZ] = currPoint->z * (1.0 - pathFraction) + pathFraction * prevPoint->z;
	    theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY );
	    if (theError)
	      {
		pLgMaster->gOutOfRange.errtype1 = RESPE1PATANGLEOUTOFRANGE;
		pLgMaster->gOutOfRange.errtype2 = (uint16_t)(theError & 0xFFFF);
	      }
	    else
	      {
		tempLong = PutAPoint(pLgMaster, pathX, pathY);
		if (tempLong)
		  return(tempLong);
	      }
	  }
	i = numSlowSteps;
	while (i)
	  {
	    pathFraction = (double)(((i * (i + 1)) >> 1) * slowDownStep) / distLD;
	    pathPoint[kX] = currPoint->x * (1.0 - pathFraction) + pathFraction * prevPoint->x;
	    pathPoint[kY] = currPoint->y * (1.0 - pathFraction) + pathFraction * prevPoint->y;
	    pathPoint[kZ] = currPoint->z * (1.0 - pathFraction) + pathFraction * prevPoint->z;
	    theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY);
	    if (theError)
	      {
		pLgMaster->gOutOfRange.errtype1 = RESPE1PATANGLEOUTOFRANGE;
		pLgMaster->gOutOfRange.errtype2 = (uint16_t)(theError & 0xFFFF);
	      }
	    else
	      {
		tempLong = PutAPoint(pLgMaster, pathX, pathY );
		if (tempLong)
		  return(tempLong);
	      }
	    i--;
	  }
	if (theError == 0)
	  {
	    i = pilePoints;
	    while (i--)
	      {
		tempLong = PutAPoint(pLgMaster, currPoint->ax, currPoint->ay);
		if (tempLong)
		  return(tempLong);
	      }
	  }
	gBeamOn = nextPoint->on;
	if (distXout > distYout)
	  dist = distXout;
	else
	  dist = distYout;
	distLD = (double)dist;
	distSlow = (int32_t)(dist >> 1);

	if (distSlow < (int32_t)theSlowDownDist) numFastSteps = 0;
	else numFastSteps = (distSlow - theSlowDownDist) / fastStep;

	fMiddlePoints = false;
	if (!fSlowDown)
	  numSlowSteps = 0;
	else
	  {
	    if (distSlow > (int32_t)theSlowDownDist)
	      numSlowSteps = fastToSlowRatio;
	    else
	      {
		numSlowSteps = 0;
		while (distSlow >= 0) 
		  {
		    numSlowSteps++;
		    distSlow -= numSlowSteps * slowDownStep;
		  }
		numSlowSteps--;
	      }
	    if (dist && (numFastSteps == 0))
	      {
		if (numSlowSteps)
		  numSlowSteps--;
		fMiddlePoints = true;
	      }
	  }
	i = 1;
	while (i <= numSlowSteps)
	  {
	    pathFraction = (double)(((i * (i + 1)) >> 1) * slowDownStep) / distLD;
	    pathPoint[kX] = currPoint->x * (1.0 - pathFraction) + pathFraction * nextPoint->x;
	    pathPoint[kY] = currPoint->y * (1.0 - pathFraction) + pathFraction * nextPoint->y;
	    pathPoint[kZ] = currPoint->z * (1.0 - pathFraction) + pathFraction * nextPoint->z;
	    theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY);
	    if (theError)
	      {
		pLgMaster->gOutOfRange.errtype1 = RESPE1PATANGLEOUTOFRANGE;
		pLgMaster->gOutOfRange.errtype2 = (uint16_t)(theError & 0xFFFF);
	      }
	    else
	      {
		tempLong = PutAPoint(pLgMaster, pathX, pathY );
		if (tempLong)
		  return(tempLong);
	      }
	    i++;
	  }
	if (fMiddlePoints)
	  {
	    pathFraction = .50 - (distLD - ((double)numSlowSteps * (double)((numSlowSteps + 1) * slowDownStep)))
				/ (6.0 * distLD);
	    pathPoint[kX] = currPoint->x * (1.0 - pathFraction) + pathFraction * nextPoint->x;
	    pathPoint[kY] = currPoint->y * (1.0 - pathFraction) + pathFraction * nextPoint->y;
	    pathPoint[kZ] = currPoint->z * (1.0 - pathFraction) + pathFraction * nextPoint->z;
	    theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY);
	    if (theError)
	      {
		pLgMaster->gOutOfRange.errtype1 = RESPE1PATANGLEOUTOFRANGE;
		pLgMaster->gOutOfRange.errtype2 = (uint16_t)(theError & 0xFFFF);
	      }
	    else
	      {
		tempLong = PutAPoint(pLgMaster, pathX, pathY );
		if (tempLong)
		  return(tempLong);
	      }
	  }
	i = 1;
	while ( i < numFastSteps )
	  {
	    pathFraction = ((double)theSlowDownDist + ((double)i * (double)fastStep)) / distLD;
	    pathPoint[kX] = currPoint->x * (1.0 - pathFraction) + pathFraction * nextPoint->x;
	    pathPoint[kY] = currPoint->y * (1.0 - pathFraction) + pathFraction * nextPoint->y;
	    pathPoint[kZ] = currPoint->z * (1.0 - pathFraction) + pathFraction * nextPoint->z;
	    theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY);
	    if (theError)
	      {
		pLgMaster->gOutOfRange.errtype1 = RESPE1PATANGLEOUTOFRANGE;
		pLgMaster->gOutOfRange.errtype2 = (uint16_t)(theError & 0xFFFF);
	      }
	    else
	      {
		tempLong = PutAPoint(pLgMaster, pathX, pathY);
		if (tempLong)
		  return(tempLong);
	      }
	    i++;
	  }
	if (numFastSteps || (!fSlowDown && (distLD > (double)fastStep)))
	  {
	    if (numFastSteps)
	      numFastSteps--;
	    pathFraction = .50 - (distLD - 2.0 * ((double)theSlowDownDist + ((double)numFastSteps * (double)fastStep)))
	      / (6.0 * distLD);
	    pathPoint[kX] = currPoint->x * (1.0 - pathFraction) + pathFraction * nextPoint->x;
	    pathPoint[kY] = currPoint->y * (1.0 - pathFraction) + pathFraction * nextPoint->y;
	    pathPoint[kZ] = currPoint->z * (1.0 - pathFraction) + pathFraction * nextPoint->z;
	    theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY);
	    if (theError)
	      {
		pLgMaster->gOutOfRange.errtype1 = RESPE1PATANGLEOUTOFRANGE;
		pLgMaster->gOutOfRange.errtype2 = (uint16_t)(theError & 0xFFFF);
	      }
	    else
	      {
		tempLong = PutAPoint(pLgMaster, pathX, pathY );
		if (tempLong)
		  return(tempLong);
	      }
	  }
      } while (++currIndex < gNumberOfStoredPoints);
    return(0);
}
#endif

uint32_t PutSequenceOfPoints (struct lg_master *pLgMaster, double *theEndPoint )
{	
#if _NEW_PATH_
  int16_t  endPointX, endPointY;
  uint32_t theError;

	theError = PointToBinary(pLgMaster, theEndPoint, &endPointX, &endPointY );
	if ( theError ) {
/*
 *		return kPatternAngleOutOfRange + theError;
 */
	 pLgMaster->gOutOfRange.errtype1 = RESPE1PATANGLEOUTOFRANGE;
	 pLgMaster->gOutOfRange.errtype2 = (uint16_t)(theError & 0xFFFF);
        } else {
	  gPointStorage[gNumberOfStoredPoints].x = theEndPoint[kX];
	  gPointStorage[gNumberOfStoredPoints].y = theEndPoint[kY];
	  gPointStorage[gNumberOfStoredPoints].z = theEndPoint[kZ];
	  gPointStorage[gNumberOfStoredPoints].ax = endPointX;
	  gPointStorage[gNumberOfStoredPoints].ay = endPointY;
	  gPointStorage[gNumberOfStoredPoints].on = gBeamOn;

	  gNumberOfStoredPoints++;
        } 
	if ( gNumberOfStoredPoints >= kMaxNumberOfAPTVectors )
			return kTooManyAPTVectors;
	return(0);
	
#else

#define	kFastDistance		0x400
#define	kSlowDistance		0x70
#define	kSlowStepsNumber	8
#define kPilePoints			6
#define kCornerAngleRad		0.0050
	int32_t endPointX, endPointY, startPointX, startPointY;
	int32_t pathX, pathY, tempLong, theError;
	int32_t absDistX, absDistY, theAbsDist;
	short i;
	double pathFraction, pathFractionUnit, pathPoint[3];
	unsigned char currentBeamOn;
	double acosX, acosY, acosZ, norm;

	theError = PointToBinary(pLgMaster, gPutPoint, &startPointX, &startPointY );
	if ( theError ) { 
          return kPatternAngleOutOfRange + theError;
        }

	theError = PointToBinary(pLgMaster, theEndPoint, &endPointX, &endPointY );
	if ( theError ) {
          return kPatternAngleOutOfRange + theError;
        }

	acosX = theEndPoint[kX] - gPutPoint[kX];
	acosY = theEndPoint[kY] - gPutPoint[kY];
	acosZ = theEndPoint[kZ] - gPutPoint[kZ];
	norm = sqrt ( acosX * acosX + acosY * acosY + acosZ * acosZ );
	if ( norm > DBL_MIN )
	{
		acosX = acos ( acosX / norm );
		acosY = acos ( acosY / norm );
		acosZ = acos ( acosZ / norm );
	}
	else
	{
		acosX = - M_PI;
		acosY = - M_PI;
		acosZ = - M_PI;
	}

	currentBeamOn = gBeamOn;
	gBeamOn = gOldBeamOn;
	if ( 
		( fabs ( acosX - gOldAcosX ) >= kCornerAngleRad ) ||
		( fabs ( acosY - gOldAcosY ) >= kCornerAngleRad ) ||
		( fabs ( acosZ - gOldAcosZ ) >= kCornerAngleRad ) )
	{
		if ( gOldFastStepsNumber > 0 )
		{
			i = gOldSlowStepsNumber;
			while ( i )
			{
				pathFraction = gOldPathFractionUnit * i--;
				pathPoint[kX] = gPutPoint[kX] * ( 1.0 - pathFraction ) +
					pathFraction * gOldPutPoint[kX];
				pathPoint[kY] = gPutPoint[kY] * ( 1.0 - pathFraction ) +
					pathFraction * gOldPutPoint[kY];
				pathPoint[kZ] = gPutPoint[kZ] * ( 1.0 - pathFraction ) +
					pathFraction * gOldPutPoint[kZ];
				theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY );
				if ( theError ) {
				  return kPatternAngleOutOfRange + theError;
                                }
				tempLong = PutAPoint(pLgMaster, pathX, pathY );
				if ( tempLong ) return tempLong;
			}
		}
		else
		{
			if ( gOldFastStepsNumber < 0 )
			{
				i = kSlowStepsNumber;
				while ( i-- )
				{
					tempLong = PutAPoint(pLgMaster, startPointX, startPointY );
					if ( tempLong ) return tempLong;
				}
			}
		}
		
		i = kPilePoints;
		while ( i-- )
		{
			tempLong = PutAPoint(pLgMaster, startPointX, startPointY );
			if ( tempLong ) return tempLong;
		}
	}
	tempLong = PutAPoint (pLgMaster, startPointX, startPointY );
	if ( tempLong ) return tempLong;
	gBeamOn = currentBeamOn;
	gOldBeamOn = currentBeamOn;

	absDistX = endPointX - startPointX;
	if ( absDistX < 0L ) absDistX = -absDistX;

	absDistY = endPointY - startPointY;
	if ( absDistY < 0L ) absDistY = -absDistY;
	
	if ( absDistX > absDistY ) theAbsDist = absDistX;
	else theAbsDist = absDistY;
	
	gOldFastStepsNumber = ( theAbsDist -
		kSlowStepsNumber * kSlowDistance ) / kFastDistance;
	if ( gOldFastStepsNumber < 0 ) gOldFastStepsNumber = 0;

	if ( theAbsDist ) /* Was gBeamOn && */
	{
		pathFractionUnit = (double)kFastDistance/
			(double)theAbsDist;
		i = 0;
		while ( i++ < gOldFastStepsNumber )
		{
			pathFraction = (double)i * pathFractionUnit;
			pathPoint[kX] = theEndPoint[kX] * pathFraction +
				( 1.0 - pathFraction ) * gPutPoint[kX];
			pathPoint[kY] = theEndPoint[kY] * pathFraction +
				( 1.0 - pathFraction ) * gPutPoint[kY];
			pathPoint[kZ] = theEndPoint[kZ] * pathFraction +
				( 1.0 - pathFraction ) * gPutPoint[kZ];
			theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY );
			if ( theError ) {
				return kPatternAngleOutOfRange + theError;
                        }
			tempLong = PutAPoint(pLgMaster, pathX, pathY );
			if ( tempLong ) return tempLong;
		}
	}
	
	gOldSlowStepsNumber = theAbsDist / kSlowDistance;
	if ( gOldSlowStepsNumber > kSlowStepsNumber )
		gOldSlowStepsNumber = kSlowStepsNumber;
	if ( theAbsDist ) gOldPathFractionUnit =
		(double)kSlowDistance / (double)theAbsDist;
	else gOldPathFractionUnit = 0.0;
	
	gOldPutPoint[kX] = gPutPoint[kX];
	gOldPutPoint[kY] = gPutPoint[kY];
	gOldPutPoint[kZ] = gPutPoint[kZ];
	gPutPoint[kX] = theEndPoint[kX];
	gPutPoint[kY] = theEndPoint[kY];
	gPutPoint[kZ] = theEndPoint[kZ];
	gOldAcosX = acosX;
	gOldAcosY = acosY;
	gOldAcosZ = acosZ;
	return(0);
#endif
}


static uint32_t PutAPoint(struct lg_master *pLgMaster, int32_t newXBin, int32_t newYBin )
{
    pLgMaster->gBuiltPattern += sizeof(struct lg_xydata);
    if (pLgMaster->gBuiltPattern > (kMaxNumberOfPatternPoints *
				    sizeof(struct lg_xydata)))
      return kTooManyPatternPoints;

    if ((kX > pLgMaster->gBuiltPattern) || (kY > pLgMaster->gBuiltPattern))
      return(kTooManyPatternPoints);
  
    pCurXYData->xdata = newXBin; 
    pCurXYData->ydata = newYBin;
    if (!gRawInput)
      {
	if (gBeamOn)
	  SetLowBeam (pCurXYData);  // display beam is now "Low"
	else
	  SetDarkBeam(pCurXYData);
      }
    pCurXYData++;
  return 0;
}

void PendPenDown(void)
{
    gPendPenDown = true;
    gPenDownPending = false;
    return;
}

void UnpendPenDown(void)
{
    gPendPenDown = false;
    if (gPenDownPending)
      {
	gPenDownPending = false;
	return SetPenDown();
      }
    return;
}

void SetPenDown(void)
{
    if (gPendPenDown)
      gPenDownPending = true;
    else
      gBeamOn = true;
    return;
}


void SetPenUp(void)
{
    gBeamOn = false;
    return;
}

void SetDefaultZ(double z)
{
    gDefaultZ = z;
    return;
}

void ChangeTransform (double *transform)
{
  if (!transform)
    gRawInput = true;
  else
    {
      gRawInput = false;
      ArrayIntoTransform ( transform, &gCurrentTransform );
    }
  return;
}

struct lg_xydata *SetUpLaserPattern(struct lg_master *pLgMaster, double *transform)
{
	SetDefaultZ ( 0.0 );
	pLgMaster->gBuiltPattern = 0;
	pCurXYData = pXYPoints;  // Set current to start of gXYPoints, it will increment
	                            // for each XY pair.
	SetPenUp (  );
	PendPenDown (  );
	gOldBeamOn = false;
	gItIsStart = true;
	gNowInside = true;
	gOldFastStepsNumber = -1;
	gOldAcosX = - M_PI;
	gOldAcosY = - M_PI;
	gOldAcosZ = - M_PI;
#if _NEW_PATH_
	gNumberOfStoredPoints = 0;
#endif
	if (!transform)
	  gRawInput = true;
	else
	{
		gRawInput = false;
		ArrayIntoTransform ( transform, &gCurrentTransform );
	}
	return(pXYPoints);
}

void CloseLaserPattern ( void )
{
	if (pXYPoints) free((void *)pXYPoints );
#if _NEW_PATH_
	if ( gPointStorage ) free((void *)gPointStorage );
#endif
}

unsigned char	WithinLaserLinearRange ( double *point )
{
	return ( ( fabs ( point[kX] ) <= gBeamLinearRangeX ) &&
		( fabs ( point[kY] ) <= gBeamLinearRangeY ) );
}

unsigned char	WithinLaserAngleRange ( double *point )
{
        double xtan, ytan;
        int itest;

        if ( fabs(point[kZ]) < DBL_MIN ) return(0);

        xtan = point[kX] / point[kZ];
        ytan = point[kY] / point[kZ];

        itest = pnpoly( gNtanpoly, Xtans, Ytans, xtan, ytan );

	return (itest);
}

/* We do not check if the points are really inside and outside */
void GetLinearBorderPoint ( double *pointInside,
		double *pointOutside, double *borderPoint )
{
	double thePointInside[3], thePointOutside[3], cX, cY, c;
	
	if ( pointOutside[kX] > 0.0 )
	{
		thePointOutside[kX] = pointOutside[kX];
		thePointInside[kX] = pointInside[kX];
	}
	else
	{
		thePointOutside[kX] = -pointOutside[kX];
		thePointInside[kX] = -pointInside[kX];
	}
	
	if ( pointOutside[kY] > 0.0 )
	{
		thePointOutside[kY] = pointOutside[kY];
		thePointInside[kY] = pointInside[kY];
	}
	else
	{
		thePointOutside[kY] = -pointOutside[kY];
		thePointInside[kY] = -pointInside[kY];
	}
	
	if ( thePointOutside[kX] > gBeamLinearRangeX )
	{
		if ( thePointOutside[kY] > gBeamLinearRangeY )
		{
			cX = thePointOutside[kX] - thePointInside[kX];
			cY = thePointOutside[kY] - thePointInside[kY];
			if ( cX <= DBL_MIN )
			{
				if ( cY <= DBL_MIN ) c = 0.0;
				else
					c = ( thePointOutside[kY] - gBeamLinearRangeY ) / cY;
			}
			else
			{
				if ( cY <= DBL_MIN )
					c = ( thePointOutside[kX] - gBeamLinearRangeX ) / cX;
				else
				{
					cX = ( thePointOutside[kX] - gBeamLinearRangeX )/cX;
					cY = ( thePointOutside[kY] - gBeamLinearRangeY )/cY;
					if ( cX > cY ) c = cX;
					else c = cY;
				}
			}
		}
		else
		{
			c = thePointOutside[kX] - thePointInside[kX];
			if ( c <= DBL_MIN ) c = 0.0;
			else c = ( thePointOutside[kX] - gBeamLinearRangeX ) / c;
		}
	}
	else
	{
		if ( thePointOutside[kY] > gBeamLinearRangeY )
		{
			c = thePointOutside[kY] - thePointInside[kY];
			if ( c <= DBL_MIN ) c = 0.0;
			else c = ( thePointOutside[kY] - gBeamLinearRangeY ) / c;
		}
		else c = 0.0;
	}

	borderPoint[kX] = pointInside[kX] * c +
		pointOutside[kX] * ( 1.0 - c );
	borderPoint[kY] = pointInside[kY] * c +
		pointOutside[kY] * ( 1.0 - c );
	borderPoint[kZ] = pointInside[kZ] * c +
		pointOutside[kZ] * ( 1.0 - c );
}


/* We do not check if the points are really inside and outside */
void GetAngleBorderPoint ( double *pointInside,
		double *pointOutside, double *borderPoint )
{
	double c;
        double XtanInside,  YtanInside;
        double XtanOutside, YtanOutside;
        double Xtnew, Ytnew;
	
        XtanInside = pointInside[kX] / pointInside[kZ];
        YtanInside = pointInside[kY] / pointInside[kZ];

        XtanOutside = pointOutside[kX] / pointOutside[kZ];
        YtanOutside = pointOutside[kY] / pointOutside[kZ];

#if 1
	segpoly( gNtanpoly
		 , Xtans
		 , Ytans
		 , XtanInside
		 , YtanInside
		 , XtanOutside
		 , YtanOutside
		 , &Xtnew
		 , &Ytnew
		 , &c
		 );
#else
        int itest;
	itest = segpoly( gNtanpoly
		 , Xtans
		 , Ytans
		 , XtanInside
		 , YtanInside
		 , XtanOutside
		 , YtanOutside
		 , &Xtnew
		 , &Ytnew
		 , &c
		 );
#endif	
	borderPoint[kX] = pointInside[kX] * c +
		pointOutside[kX] * ( 1.0 - c );
	borderPoint[kY] = pointInside[kY] * c +
		pointOutside[kY] * ( 1.0 - c );
	borderPoint[kZ] = pointInside[kZ] * c +
		pointOutside[kZ] * ( 1.0 - c );
}


unsigned char	LinearBorderPointsBetween (
			double *startPoint, double *endPoint,
			double *entrancePoint, double *exitPoint )
{
	double cInX, cOutX, cInY, cOutY, cIn, cOut, dX, dY;
	
	dX = endPoint[kX] - startPoint[kX];
	dY = endPoint[kY] - startPoint[kY];
	if ( fabs ( dX ) <= DBL_MIN )
	{
		if ( fabs ( dY ) <= DBL_MIN ) return false;
		if ( ( fabs ( startPoint[kX] ) > gBeamLinearRangeX ) ||
			( fabs ( endPoint[kX] ) > gBeamLinearRangeX ) ) return false;
		if ( dY < 0.0 )
		{
			cIn = ( gBeamLinearRangeY - startPoint[kY] ) / dY;
			cOut = ( -gBeamLinearRangeY - startPoint[kY] ) / dY;
		}
		else
		{
			cIn = ( -gBeamLinearRangeY - startPoint[kY] ) / dY;
			cOut = ( gBeamLinearRangeY - startPoint[kY] ) / dY;
		}
		if ( ( cIn < 0.0 ) || ( cIn > 1.0 ) ||
			( cOut < 0.0 ) || ( cOut > 1.0 ) ) return false;
	}
	else
	{
		if ( fabs ( dY ) <= DBL_MIN )
		{
			if ( ( fabs ( startPoint[kY] ) > gBeamLinearRangeY ) ||
				( fabs ( endPoint[kY] ) > gBeamLinearRangeY ) )
				return false;
			if ( dX < 0.0 )
			{
				cIn = ( gBeamLinearRangeX - startPoint[kX] ) / dX;
				cOut = ( -gBeamLinearRangeX - startPoint[kX] ) / dX;
			}
			else
			{
				cIn = ( -gBeamLinearRangeX - startPoint[kX] ) / dX;
				cOut = ( gBeamLinearRangeX - startPoint[kX] ) / dX;
			}
			if ( ( cIn < 0.0 ) || ( cIn > 1.0 ) ||
				( cOut < 0.0 ) || ( cOut > 1.0 ) ) return false;
		}
		else
		{
			if ( dX < 0.0 )
			{
				cInX = ( gBeamLinearRangeX - startPoint[kX] ) / dX;
				cOutX = ( -gBeamLinearRangeX - startPoint[kX] ) / dX;
			}
			else
			{
				cInX = ( -gBeamLinearRangeX - startPoint[kX] ) / dX;
				cOutX = ( gBeamLinearRangeX - startPoint[kX] ) / dX;
			}

			if ( dY < 0.0 )
			{
				cInY = ( gBeamLinearRangeY - startPoint[kY] ) / dY;
				cOutY = ( -gBeamLinearRangeY - startPoint[kY] ) / dY;
			}
			else
			{
				cInY = ( -gBeamLinearRangeY - startPoint[kY] ) / dY;
				cOutY = ( gBeamLinearRangeY - startPoint[kY] ) / dY;
			}

			if ( ( cInX > 1.0 ) || ( cOutX < 0.0 ) ||
				( cInY > 1.0 ) || ( cOutY < 0.0 ) ) return false;
			if ( cInX > cInY ) cIn = cInX;
			else cIn = cInY;
			if ( cOutX < cOutY ) cOut = cOutX;
			else cOut = cOutY;
			if ( cIn < 0.0 ) return false;
			if ( cOut > 1.0 ) return false;
		}
	}

	entrancePoint[kX] = endPoint[kX] * cIn +
		startPoint[kX] * ( 1.0 - cIn );
	entrancePoint[kY] = endPoint[kY] * cIn +
		startPoint[kY] * ( 1.0 - cIn );
	entrancePoint[kZ] = endPoint[kZ] * cIn +
		startPoint[kZ] * ( 1.0 - cIn );

	exitPoint[kX] = endPoint[kX] * cOut +
		startPoint[kX] * ( 1.0 - cOut );
	exitPoint[kY] = endPoint[kY] * cOut +
		startPoint[kY] * ( 1.0 - cOut );
	exitPoint[kZ] = endPoint[kZ] * cOut +
		startPoint[kZ] * ( 1.0 - cOut );
	
	return true;
}


unsigned char	AngleBorderPointsBetween (
			double *startPoint, double *endPoint,
			double *entrancePoint, double *exitPoint )
{
	double cInX, cOutX, cInY, cOutY, cIn, cOut, dX, dY;
        double XtanStart, YtanStart;
        double XtanEnd,   YtanEnd;
	
	if ( fabs ( startPoint[kZ] ) <= DBL_MIN ) return false;
	if ( fabs ( endPoint[kZ] )   <= DBL_MIN ) return false;

        XtanStart = startPoint[kX] / startPoint[kZ];
        YtanStart = startPoint[kY] / startPoint[kZ];
        XtanEnd   = endPoint[kX] / endPoint[kZ];
        YtanEnd   = endPoint[kY] / endPoint[kZ];

	dX = XtanEnd - XtanStart;
	dY = YtanEnd - YtanStart;
	if ( fabs ( dX ) <= DBL_MIN )
	{
		if ( fabs ( dY ) <= DBL_MIN ) return false;
		if ( ( fabs ( XtanStart ) > TanLimit ) ||
			( fabs ( XtanEnd ) > TanLimit ) ) return false;
		if ( dY < 0.0 )
		{
			cIn = ( TanLimit - YtanStart ) / dY;
			cOut = ( -TanLimit - YtanStart ) / dY;
		}
		else
		{
			cIn = ( -TanLimit - YtanStart ) / dY;
			cOut = ( TanLimit - YtanStart ) / dY;
		}
		if ( ( cIn < 0.0 ) || ( cIn > 1.0 ) ||
			( cOut < 0.0 ) || ( cOut > 1.0 ) ) return false;
	}
	else
	{
		if ( fabs ( dY ) <= DBL_MIN )
		{
			if ( ( fabs ( YtanStart ) > TanLimit ) ||
				( fabs ( YtanEnd ) > TanLimit ) )
				return false;
			if ( dX < 0.0 )
			{
				cIn  = (  TanLimit - XtanStart ) / dX;
				cOut = ( -TanLimit - XtanStart ) / dX;
			}
			else
			{
				cIn  = ( -TanLimit - XtanStart ) / dX;
				cOut = (  TanLimit - XtanStart ) / dX;
			}
			if ( ( cIn < 0.0 ) || ( cIn > 1.0 ) ||
				( cOut < 0.0 ) || ( cOut > 1.0 ) ) return false;
		}
		else
		{
			if ( dX < 0.0 )
			{
				cInX  = (  TanLimit - XtanStart ) / dX;
				cOutX = ( -TanLimit - XtanStart ) / dX;
			}
			else
			{
				cInX  = ( -TanLimit - XtanStart ) / dX;
				cOutX = (  TanLimit - XtanStart ) / dX;
			}

			if ( dY < 0.0 )
			{
				cInY  = (  TanLimit - YtanStart ) / dY;
				cOutY = ( -TanLimit - YtanStart ) / dY;
			}
			else
			{
				cInY  = ( -TanLimit - YtanStart ) / dY;
				cOutY = (  TanLimit - YtanStart ) / dY;
			}

			if ( ( cInX > 1.0 ) || ( cOutX < 0.0 ) ||
				( cInY > 1.0 ) || ( cOutY < 0.0 ) ) return false;
			if ( cInX > cInY ) cIn = cInX;
			else cIn = cInY;
			if ( cOutX < cOutY ) cOut = cOutX;
			else cOut = cOutY;
			if ( cIn < 0.0 ) return false;
			if ( cOut > 1.0 ) return false;
		}
	}

	entrancePoint[kX] = endPoint[kX] * cIn +
		startPoint[kX] * ( 1.0 - cIn );
	entrancePoint[kY] = endPoint[kY] * cIn +
		startPoint[kY] * ( 1.0 - cIn );
	entrancePoint[kZ] = endPoint[kZ] * cIn +
		startPoint[kZ] * ( 1.0 - cIn );
	exitPoint[kX] = endPoint[kX] * cOut +
		startPoint[kX] * ( 1.0 - cOut );
	exitPoint[kY] = endPoint[kY] * cOut +
		startPoint[kY] * ( 1.0 - cOut );
	exitPoint[kZ] = endPoint[kZ] * cOut +
		startPoint[kZ] * ( 1.0 - cOut );	
	return true;
}




uint32_t Transform3DPointToBinary(struct lg_master *pLgMaster, double x, double y,
	double z, int16_t *xAngle, int16_t *yAngle )
{
	double inputPoint[3], outputPoint[3];
	inputPoint[kX] = x;
	inputPoint[kY] = y;
	inputPoint[kZ] = z;
	TransformPoint ( &gCurrentTransform, inputPoint, outputPoint );
	return PointToBinary(pLgMaster, outputPoint, xAngle, yAngle );
}

uint32_t PointToBinary(struct lg_master *pLgMaster,  double *point, int16_t *xAngle, int16_t *yAngle)
{
  double angleX, angleY;
  GeometricAnglesFrom3D(pLgMaster,point[kX], point[kY], point[kZ], &angleX, &angleY );
  return(ConvertGeometricAnglesToBinary(pLgMaster, angleX, angleY, xAngle, yAngle));
}

int InitLaserPattern ( void )
{

        tan_init();
	
	pXYPoints = (struct lg_xydata *)malloc( (size_t) ( ( kMaxNumberOfPatternPoints + 1 ) *
		2 * sizeof ( unsigned long ) ) );
	memset(pXYPoints, 0, ((kMaxNumberOfPatternPoints + 1) * 2 * sizeof(uint32_t)));
#if _NEW_PATH_
        gPointStorage = (struct APTVector *)malloc( (size_t)
		( kMaxNumberOfAPTVectors * sizeof ( struct APTVector ) ) );
#endif
	return(0);
}



static int  findarc(struct lg_master *pLgMaster, double p1[3], double p2[3], double p3[3], double Dinterp )
{
    double Xd21;
    double Yd21;
    double Zd21;
    double Xd32;
    double Yd32;
    double Zd32;
    double NXarc;
    double NYarc;
    double NZarc;
    double tmag;
    double Darc;
    double D1;
    double NX1;
    double NY1;
    double NZ1;
    double Xmid12;
    double Ymid12;
    double Zmid12;
    double NXmid12;
    double NYmid12;
    double NZmid12;
    double Dmid12;
    double D2, NX2, NY2, NZ2;
    double Xmid32;
    double Ymid32;
    double Zmid32;
    double NXmid32;
    double NYmid32;
    double NZmid32;
    double Dmid32;
    double D3, NX3, NY3, NZ3;
    double Xcross23;
    double Ycross23;
    double Zcross23;
    double Xcross31;
    double Ycross31;
    double Zcross31;
    double Xcross12;
    double Ycross12;
    double Zcross12;
    double denom;
    double CX;
    double CY;
    double CZ;
    double RX;
    double RY;
    double RZ;
    double radius;
    double uRX;
    double uRY;
    double uRZ;
    double vRX;
    double vRY;
    double vRZ;
    double p2dX;
    double p2dY;
    double p2dZ;

    double ux2, vy2;
    double angrad;
    double anginterp;
    double Dnum;
    double angstep;
    double ang;
    double sinA, cosA;
    double Xtmp, Ytmp, Ztmp;

    uint32_t tempLong;
    int16_t  pathX, pathY;
    uint32_t theError;
    double pathPoint[3];    

    //  find plane of three points

    Xd21  =  p2[0] - p1[0];
    Yd21  =  p2[1] - p1[1];
    Zd21  =  p2[2] - p1[2];

    Xd32  =  p3[0] - p2[0];
    Yd32  =  p3[1] - p2[1];
    Zd32  =  p3[2] - p2[2];

    NXarc =  (Yd21*Zd32 - Zd21*Yd32);
    NYarc =  (Zd21*Xd32 - Xd21*Zd32);
    NZarc =  (Xd21*Yd32 - Yd21*Xd32);

         // normalize
    tmag = sqrt(NXarc*NXarc + NYarc*NYarc + NZarc*NZarc);
    NXarc = NXarc / tmag;
    NYarc = NYarc / tmag;
    NZarc = NZarc / tmag;

    Darc  =  NXarc * p1[0] + NYarc * p1[1] + NZarc * p1[2];

    D1 = Darc; NX1 = NXarc; NY1 = NYarc;  NZ1 = NZarc;

    //  find midpoints of arcs

    Xmid12 = 0.5 * (p1[0] + p2[0]);
    Ymid12 = 0.5 * (p1[1] + p2[1]);
    Zmid12 = 0.5 * (p1[2] + p2[2]);

    Xmid32 = 0.5 * (p3[0] + p2[0]);
    Ymid32 = 0.5 * (p3[1] + p2[1]);
    Zmid32 = 0.5 * (p3[2] + p2[2]);

    //  find normals and planes of bisecting planes

    NXmid12 =  (p1[0] - p2[0]);
    NYmid12 =  (p1[1] - p2[1]);
    NZmid12 =  (p1[2] - p2[2]);

    tmag = sqrt(NXmid12*NXmid12 + NYmid12*NYmid12 + NZmid12*NZmid12);
    NXmid12 = NXmid12 / tmag;
    NYmid12 = NYmid12 / tmag;
    NZmid12 = NZmid12 / tmag;

    Dmid12 = NXmid12 * Xmid12 + NYmid12 * Ymid12 + NZmid12 * Zmid12;

    D2 = Dmid12; NX2 = NXmid12; NY2 = NYmid12; NZ2 = NZmid12;
    

    NXmid32 =  (p3[0] - p2[0]);
    NYmid32 =  (p3[1] - p2[1]);
    NZmid32 =  (p3[2] - p2[2]);

    tmag = sqrt(NXmid32*NXmid32 + NYmid32*NYmid32 + NZmid32*NZmid32);
    NXmid32 = NXmid32 / tmag;
    NYmid32 = NYmid32 / tmag;
    NZmid32 = NZmid32 / tmag;

    Dmid32 = NXmid32 * Xmid32 + NYmid32 * Ymid32 + NZmid32 * Zmid32;

    D3 = Dmid32; NX3 = NXmid32; NY3 = NYmid32; NZ3 = NZmid32;

    // find intersection of three places

    //  d1 (N2 x N3) + d2 (N3 x N1) + d3 (N1 x N2) / N1 . (N2 x N3)

    Xcross23 = (NY2*NZ3 - NZ2*NY3);
    Ycross23 = (NZ2*NX3 - NX2*NZ3);
    Zcross23 = (NX2*NY3 - NY2*NX3);

    Xcross31 = (NY3*NZ1 - NZ3*NY1);
    Ycross31 = (NZ3*NX1 - NX3*NZ1);
    Zcross31 = (NX3*NY1 - NY3*NX1);

    Xcross12 = (NY1*NZ2 - NZ1*NY2);
    Ycross12 = (NZ1*NX2 - NX1*NZ2);
    Zcross12 = (NX1*NY2 - NY1*NX2);

    denom = NX1 * Xcross23 + NY1 * Ycross23 + NZ1 * Zcross23;

    if ( fabs( denom ) < 0.000000001 ) return(0);
    CX = (D1 * Xcross23 + D2 * Xcross31 + D3 * Xcross12) / denom;
    CY = (D1 * Ycross23 + D2 * Ycross31 + D3 * Ycross12) / denom;
    CZ = (D1 * Zcross23 + D2 * Zcross31 + D3 * Zcross12) / denom;

    // find radius vector to first point
    RX  = p1[0] - CX;
    RY  = p1[1] - CY;
    RZ  = p1[2] - CZ;
    radius = sqrt( RX*RX + RY*RY + RZ*RZ ); 

    uRX = RX / radius;
    uRY = RY / radius;
    uRZ = RZ / radius;

    // synthesize a perpendicular radius vector
    vRX =  NYarc * uRZ - NZarc * uRY;
    vRY =  NZarc * uRX - NXarc * uRZ;
    vRZ =  NXarc * uRY - NYarc * uRX;
    tmag = sqrt( vRX*vRX + vRY*vRY + vRZ*vRZ );
    vRX = vRX / tmag;
    vRY = vRY / tmag;
    vRZ = vRZ / tmag;

    // using the new bases, find angle of second point
    p2dX = p2[0] - CX;
    p2dY = p2[1] - CY;
    p2dZ = p2[2] - CZ;

    ux2  = uRX * p2dX + uRY * p2dY + uRZ * p2dZ;
    vy2  = vRX * p2dX + vRY * p2dY + vRZ * p2dZ;

    angrad = atan2( vy2, ux2 );

    anginterp = Dinterp / radius;

    if ( angrad > 0 ) {
       Dnum = 1 + floor(   angrad / anginterp );
    } else {
       Dnum = 1 + floor( - angrad / anginterp );
    }
    if ( Dnum < 2.0 ) { Dnum = 2.0; }

    angstep = angrad / Dnum;
    if ( angstep > 0 ) { 
       for ( ang = angstep; ang < (angrad-0.1*angstep); ang += angstep ) {
              sinA = sin(ang);
              cosA = cos(ang);
              Xtmp = radius * (cosA * uRX + sinA * vRX) + CX;
              Ytmp = radius * (cosA * uRY + sinA * vRY) + CY;
              Ztmp = radius * (cosA * uRZ + sinA * vRZ) + CZ;
              pathPoint[kX] = Xtmp;
              pathPoint[kY] = Ytmp;
              pathPoint[kZ] = Ztmp;
              theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY );
              if ( theError ) {
                       return kPatternAngleOutOfRange;
              } else {
                       tempLong = PutAPoint(pLgMaster, pathX, pathY );
                       if ( tempLong ) return tempLong;
              }

       }
    } else {
       for ( ang = angstep; ang > (angrad-0.1*angstep); ang += angstep ) {
              sinA = sin(ang);
              cosA = cos(ang);
              Xtmp = radius * (cosA * uRX + sinA * vRX) + CX;
              Ytmp = radius * (cosA * uRY + sinA * vRY) + CY;
              Ztmp = radius * (cosA * uRZ + sinA * vRZ) + CZ;
              pathPoint[kX] = Xtmp;
              pathPoint[kY] = Ytmp;
              pathPoint[kZ] = Ztmp;
              theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY );
              if ( theError ) {
                       return kPatternAngleOutOfRange;
              } else {
                       tempLong = PutAPoint(pLgMaster, pathX, pathY );
                       if ( tempLong ) return tempLong;
              }
       }
    }

    //  print out second point
    pathPoint[kX] = p2[0];
    pathPoint[kY] = p2[1];
    pathPoint[kZ] = p2[2];
    theError = PointToBinary(pLgMaster, pathPoint, &pathX, &pathY );
    if ( theError ) {
           return kPatternAngleOutOfRange;
    } else {
           tempLong = PutAPoint(pLgMaster, pathX, pathY );
           if ( tempLong ) return tempLong;
    }

    return( 0 );
}
