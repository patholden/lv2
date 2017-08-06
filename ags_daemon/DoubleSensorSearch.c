/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
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
#include "SensorSearch.h"
#include "DoubleSensorSearch.h"
#include "QuickCheckManager.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "DoCoarseScan.h"
#include "DoCoarseScan2.h"
#include "LaserInterface.h"
#include "LaserCmds.h"

#define NZERO 5
#define kFineSearchStep                     0x2
#define kFineSearchSpanSteps                70
#define kFineExtend                         5
#define kSuperExtend                        25
#define kSuperFineSearchStep                0x1
#define kSuperFineSearchSpanSteps           256
#define kNumberOfCrossesToAverage           7
#define kNumberOfCrosses                    7
#define kNumberOfSuperCrossesToAverage      2
#define kDELLEV                             60
#define kCoarseStepPeriod                   20
#define kFineStepPeriod                     50
#define kQuickStepPeriod                    30



int DoubleSearchForASensor(struct lg_master *pLgMaster
                          , int16_t startX
                          , int16_t startY
                          , double *foundX
                          , double *foundY
                          )
{
    struct lg_xydata   xydata;
    struct lg_xydelta  xydelta;
    int16_t            MinX=0,MinY=0;
    int16_t            MaxX=0, MaxY=0;
    int32_t  posStepSize, negStepSize;
    int32_t  xSteps = 0;
    int32_t  ySteps = 0;
    uint32_t nSteps = 0;
    uint32_t longFactor = 0;
    int32_t  i = 0;
    int16_t tempX = 0;
    int16_t tempY = 0;
    int16_t eolXPos = 0;
    int16_t eolYPos = 0;
    int16_t eolXNeg = 0;
    int16_t eolYNeg = 0;
    int16_t f1x=0,f1y=0;
    int16_t f2x=0,f2y=0;
    double avgX = 0;
    double avgY = 0;
    int16_t centX = 0;
    int16_t centY = 0;
    int16_t sense_step = 0;
    int16_t new_step = 0;
    int16_t HatchCount = 0;
    int16_t hf2 = 0;
    int theResult = 0;
    int16_t DwellHalfSpan = 0;
    int16_t DwellStep = 0;

        
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));

    // initialize if first sensor
    if (gSearchCurrentSensor == 0 || gLoutPtr == 0)
    {
      memset((char *)gLoutBase, 0, sizeof(struct targetreturns_file_header));
      gLoutPtr   = gLoutBase + sizeof(struct targetreturns_file_header);
      gLoutSize  = sizeof(struct targetreturns_file_header);
      gLoutCount = 0;

      syslog(LOG_DEBUG, "SenseThreshold: %d,  SenseThresholdCoarse: %d,  SenseThresholdFine: %d,  SenseThresholdSuperFine: %d",
	     pLgMaster->gSenseThreshold, pLgMaster->gSenseThresholdCoarse, pLgMaster->gSenseThresholdFine, pLgMaster->gSenseThresholdSuperFine);
    }    
	
    xydata.xdata =  startX;
    xydata.ydata =  startY;
    // Move mirrors to XY in the dark to avoid ghost/tail
    move_dark(pLgMaster, (struct lg_xydata *)&xydata);
    usleep(250);
    eolXPos = startX;
    eolYPos = startY;
    eolXNeg = startX;
    eolYNeg = startY;
    pLgMaster->gHeaderSpecialByte = 0x40;
    if (pLgMaster->LongOrShortThrowSearch == 1)
      {
	for (longFactor = 1; longFactor <= 4 ; longFactor *= 2)
	  {
            pLgMaster->gSearchType = COARSESEARCH;
	    theResult = DoCoarseScan(pLgMaster, xydata.xdata, xydata.ydata, 0x4,
				     32 * longFactor, &f1x, &f1y);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kCoarseNotFound)
	      continue;
            pLgMaster->gSearchType = COARSESEARCH;
	    theResult = DoCoarseScan2(pLgMaster, xydata.xdata, xydata.ydata,
				      0x4, 32 * longFactor, &f2x, &f2y);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kCoarseNotFound)
	      continue;
	    avgX = f2x;
	    avgY = f1y;
	    theResult = DoubleFineLevel(pLgMaster
                                       , avgX
                                       , avgY
                                       , foundX
                                       , foundY
                                       , &MinX
                                       , &MinY
                                       , &MaxX
                                       , &MaxY
                                       );
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kFineNotFound || theResult == kCoarseNotFound)
	      continue;
	    theResult = DoubleSuperSearch(pLgMaster, foundX, foundY, &MinX, &MinY, &MaxX, &MaxY);
	    if (theResult)
	      return(theResult);
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == kSuperFineNotFound)
	      continue;
	    if (theResult == 0)
	      {
		*foundX = avgX;
		*foundY = avgY;
		return(0);
	      }
	  }
	if (theResult)
	  return(theResult);
	*foundX = avgX;
	*foundY = avgY;
	return(0);
      }
    HatchCount       =  pLgMaster->gHatchFactor / pLgMaster->gCoarse2Factor;
    pLgMaster->gNumberOfSpirals =  pLgMaster->gSpiralFactor / pLgMaster->gCoarse2Factor;

    theResult = kCoarseNotFound;
    sense_step = HatchCount * pLgMaster->gCoarse2SearchStep;
    limitCalc(startX, startY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, sense_step);

    for (i = 0; i < (2*(HatchCount-2)); i++)
      {
	// new_step = ((2*i) + 1) * pLgMaster->gCoarse2SearchStep;
	hf2 = (((i&1)<<1)-1)*(i>>1);
	new_step = hf2 * pLgMaster->gCoarse2SearchStep;
	tempX = startX + new_step;
	tempY = startY + new_step;
	// limitCalc(tempX, tempY,&eolXNeg, &eolXPos, &eolYNeg, &eolYPos, sense_step);
	posStepSize = pLgMaster->gCoarse2SearchStep;
	negStepSize = -pLgMaster->gCoarse2SearchStep;
	xSteps = (eolXPos - eolXNeg) / posStepSize;
	ySteps = (eolYPos - eolYNeg) / posStepSize;
#ifdef ZDEBUG
syslog(LOG_DEBUG,"xySteps %x %x %x %x", posStepSize, negStepSize, xSteps, ySteps );
#endif
	// Search around XNeg & Y
	theResult = DoubleCoarseLeg(pLgMaster
                                   , eolXNeg
                                   , tempY
                                   , posStepSize
                                   , 0
                                   , xSteps
                                   , foundX
                                   , foundY
                                   );
#ifdef ZDEBUG
syslog(LOG_DEBUG,"S4ASxny0 Coarse xy %x %x fnd %lf %lf result %x", eolXNeg, tempY, *foundX, *foundY, theResult );
#endif  /* ZDEBUG */
	if (theResult == kStopWasDone)
	  return theResult;
	if (theResult == 0)
	  break;
	// Search around X & NegY
	theResult = DoubleCoarseLeg(pLgMaster
                                   , tempX
                                   , eolYNeg
                                   , 0
                                   , posStepSize
                                   , ySteps
                                   , foundX
                                   , foundY
                                   );
#ifdef ZDEBUG
syslog(LOG_DEBUG,"S4ASxnyn Coarse xy %x %x fnd %lf %lf result %x", tempX, eolYNeg, *foundX, *foundY, theResult );
#endif  /* ZDEBUG */
	if (theResult == kStopWasDone)
	  return theResult;
	if (theResult == 0)
	  break;
      }
    if (theResult != 0)
      {
         // start the out-going spirals
         // slightly inside the outermost "hatch"
        eolXPos -= 2 * pLgMaster->gCoarse2SearchStep;
        eolYPos -= 2 * pLgMaster->gCoarse2SearchStep;
        eolXNeg += 2 * pLgMaster->gCoarse2SearchStep;
        eolYNeg += 2 * pLgMaster->gCoarse2SearchStep;
	for (i= 0; i < 2 + (pLgMaster->gNumberOfSpirals - (HatchCount/2)); i++)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);

	    AdjustXYLimit(&eolXPos, &eolXNeg, &eolYPos, &eolYNeg, pLgMaster->gCoarse2SearchStep);
	    posStepSize = pLgMaster->gCoarse2SearchStep;
	    negStepSize = -pLgMaster->gCoarse2SearchStep;
	    xSteps = (eolXPos - eolXNeg) / posStepSize;
	    ySteps = (eolYPos - eolYNeg) / posStepSize;
 
	    // Search around NegX/NegY
	    theResult = DoubleCoarseLeg(pLgMaster
                                 , eolXNeg
                                 , eolYNeg
                                 , posStepSize
                                 , 0
                                 , xSteps
                                 , foundX
                                 , foundY
                                 );
#ifdef ZDEBUG
syslog(LOG_DEBUG,"S5ASxnyn Coarse xy %x %x fnd %lf %lf result %x", eolXNeg, eolYNeg, *foundX, *foundY, theResult );
#endif  /* ZDEBUG */
	    if (theResult == kStopWasDone)
	      return(theResult);
	    if (theResult == 0)
	      break;
	    // Search around PosX/NegY
	    theResult = DoubleCoarseLeg(pLgMaster
                                       , eolXPos
                                       , eolYNeg
                                       , 0
                                       , posStepSize
                                       , ySteps
                                       , foundX
                                       , foundY
                                       );
#ifdef ZDEBUG
syslog(LOG_DEBUG,"S5ASxpyn Coarse xy %x %x fnd %lf %lf result %x", eolXPos, eolYNeg, *foundX, *foundY, theResult );
#endif  /* ZDEBUG */
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == 0)
	      break;
	    // Search around PosX/PosY
	    theResult = DoubleCoarseLeg(pLgMaster
                                       , eolXPos
                                       , eolYPos
                                       , negStepSize
                                       , 0
                                       , xSteps
                                       ,
				  foundX , foundY
                                       );
#ifdef ZDEBUG
syslog(LOG_DEBUG,"S5ASxpyp Coarse xy %x %x fnd %lf %lf result %x", eolXPos, eolYPos, *foundX, *foundY, theResult );
#endif  /* ZDEBUG */
	    if (theResult == kStopWasDone)
	      return(theResult);
	    if (theResult == 0)
	      break;
	    // Search around NegX/PosY
	    theResult = DoubleCoarseLeg(pLgMaster
                                       , eolXNeg
                                       , eolYPos
                                       , 0
                                       , negStepSize
                                       , ySteps
                                       , foundX
                                       , foundY
                                       );
#ifdef ZDEBUG
syslog(LOG_DEBUG,"S5ASxnyp Coarse xy %x %x fnd %lf %lf result %x", eolXNeg, eolYPos, *foundX, *foundY, theResult );
#endif  /* ZDEBUG */
	    if (theResult == kStopWasDone)
	      return theResult;
	    if (theResult == 0)
	      break;
	  }
      }
    if ((theResult == 0) && ((*foundX == 0) || (*foundY == 0)))
      {
	syslog(LOG_ERR, "SearchForASensor:  Indicates found but XY pair invalid");
	return(kCoarseNotFound);
      }
    if ((theResult == 0) && (pLgMaster->gDwell > 0))
      {
        DwellStep = 2;
        DwellHalfSpan = 64;
        nSteps = (2 * DwellHalfSpan) / DwellStep;

	for (i=0; i<pLgMaster->gDwell; i++)
	  {
	    centX = *foundX;
	    centY = *foundY - DwellHalfSpan;
	    xydata.xdata =  centX;
	    xydata.ydata =  centY;
	    xydelta.xdata = 0;
	    xydelta.ydata = DwellStep;
	    if (DoLineSearch(pLgMaster, (struct lg_xydata *)&xydata,
			     (struct lg_xydelta *)&xydelta, nSteps))
	      {
		return kStopWasDone;
	      }
	    centX = *foundX - DwellHalfSpan;
	    centY = *foundY;
	    xydata.xdata =  centX;
	    xydata.ydata =  centY;
	    xydelta.xdata = DwellStep;
	    xydelta.ydata = 0;
	    if (DoLineSearch(pLgMaster, (struct lg_xydata *)&xydata,
			     (struct lg_xydelta *)&xydelta, nSteps))
	      {
		return kStopWasDone;
	      }
	  }
      }
#ifdef ZDEBUG
syslog(LOG_DEBUG,"S6ASxy fnd %lf %lf result %x", *foundX, *foundY, theResult );
#endif  /* ZDEBUG */
    if (theResult == 0)
      {
	xydata.xdata =  *foundX;
	xydata.ydata =  *foundY;
#ifdef ZDEBUG
	syslog(LOG_NOTICE
              , "SearchForASensor: Found target startX=%x,startY=%x,foundX=%lf,foundY=%lf dbgcount %d"
              , startX
              ,startY
              ,*foundX
              ,*foundY
              , pLgMaster->debug_count
              );
#endif
      }
    else
      {
	syslog(LOG_NOTICE, "SearchForASensor: Not Found for startX=%x,startY=%x, rc=%x", startX,startY,theResult);
	xydata.xdata =  startX;
	xydata.ydata =  startY;
      }
    syslog(LOG_NOTICE, "SearchForASensor: move-dark x=%x,y=%x", xydata.xdata,xydata.ydata);
    move_one_dark(pLgMaster, (struct lg_xydata *)&xydata);
    return(theResult);
}

int DoubleCoarseLeg(struct lg_master *pLgMaster
                          , int16_t Xin
                          , int16_t Yin
                          , int16_t delX
                          , int16_t delY
                          , uint32_t nStepsIn
                          , double  *foundX
                          , double  *foundY
                          )
{
    struct lg_xydata  xydata;
    struct lg_xydelta xydelta;
    int16_t           tmpX, tmpY;
    int16_t           eolX, eolY;
    double            avgX, avgY;
    int16_t           MinX, MinY;
    int16_t           MaxX, MaxY;
    int16_t           exMinX, exMinY;
    int16_t           exMaxX, exMaxY;
    int32_t           sumX=0,sumY=0;
    int32_t           i, count=0;
    int32_t           count1, sumX1, sumY1;
    int32_t           count2, sumX2, sumY2;
    int               theResult;
    uint32_t          numStepsFixed = 15;  // for 2nd coarse set, fix number of steps



    // Initialize variables and buffers to be used
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    memset((char *)&xydelta, 0, sizeof(struct lg_xydata));

    // Start searching
    xydata.xdata =  Xin;
    xydata.ydata =  Yin;
    xydelta.xdata = delX;
    xydelta.ydata = delY;
    pLgMaster->gSearchType = COARSESEARCH;
    theResult = DoLevelSearch(pLgMaster
                             , (struct lg_xydata*)&xydata
                             , (struct lg_xydelta*)&xydelta
                             , nStepsIn
                             , gLout
                             , 1
                             );
#ifdef ZDEBUG
    count = 0;
    for (i = 0; i < nStepsIn; i++)
      {
	tmpX = Xin + (i * delX);
	tmpY = Yin + (i * delY);
syslog(LOG_DEBUG
      , "coarser %d %d %d %d %d %d del %d %d i %d  dbgcnt %06d"
      , tmpX
      , tmpY
      , gLout[0+3*i]
      , gLout[1+3*i]
      , gLout[2+3*i]
      , nStepsIn
      , delX
      , delY
      , i
      , pLgMaster->debug_count
      );
      }
#endif  /* ZDEBUG */
    if (theResult)
      return(theResult);

    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return(kStopWasDone);

    count = 0;
    for (i = 0; i < nStepsIn; i++)
      {
	tmpX = Xin + (i * delX);
	tmpY = Yin + (i * delY);
	tmpX = gLout[1+3*i];
	tmpY = gLout[2+3*i];
	if (gLout[3*i] < pLgMaster->gSenseThresholdCoarse && gLout[3*i] > 1)
	  {
	    sumX += tmpX;
	    sumY += tmpY;
	    count++;
	  }
      }
    if (count >= 1)
      {
	avgX = (sumX/count);
	avgY = (sumY/count);
	tmpX = avgX;
	tmpY = avgY;
	*foundX = avgX;
	*foundY = avgY;
	//
	// center back-search on found point
        // because of scanner lag,
        // searches need to be done in both directions
        // and will probably not overlap
        //
        // finally, fix the number of steps for this last set
        //
        AdjustOneXYSet(tmpX, tmpY, &eolX, &eolY, delX, delY, numStepsFixed);

           //  search in +delX/+delY direction
	xydata.xdata =  eolX;
	xydata.ydata =  eolY;
	xydelta.xdata = delX;
	xydelta.ydata = delY;
        pLgMaster->gSearchType = COARSESEARCH;
	theResult = DoLevelSearch(pLgMaster
                                 , (struct lg_xydata*)&xydata
				 , (struct lg_xydelta*)&xydelta
                                 , 2*numStepsFixed
                                 , gLout
                                 , 1
                                 );
	if (theResult == kStopWasDone)
	  return(theResult);

	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
                 
	tmpX = eolX;
	tmpY = eolY;
	sumX1 = 0;
	sumY1 = 0;
	count1 = 0;
	for (i=0; i < 2*numStepsFixed; i++)
	  {
	    tmpX += delX;
	    tmpY += delY; 

#ifdef ZDEBUG
syslog( LOG_DEBUG
      , "coarsel %d %d %d %d %d %d del %d %d i %d  a dbgcnt %06d"
      , tmpX
      , tmpY
      , gLout[0+3*i]
      , gLout[1+3*i]
      , gLout[2+3*i]
      , numStepsFixed
      , delX
      , delY
      , i 
      , pLgMaster->debug_count
      ); /* ZDEBUG */
#endif  /* ZDEBUG */

	    tmpX = gLout[1+3*i];
	    tmpY = gLout[2+3*i];
	    if (gLout[3*i] < pLgMaster->gSenseThresholdCoarse && gLout[3*i] > 1)
	      {
		sumX1 += tmpX;
		sumY1 += tmpY;
		count1++;
	      }
	  }

           //  now search in -delX/-delY direction
	xydata.xdata =  eolX + 2*numStepsFixed * delX;
	xydata.ydata =  eolY + 2*numStepsFixed * delY;
	xydelta.xdata = -delX;
	xydelta.ydata = -delY;
        pLgMaster->gSearchType = COARSESEARCH;
	theResult = DoLevelSearch(pLgMaster
                                 , (struct lg_xydata*)&xydata
                                 , (struct lg_xydelta*)&xydelta
                                 , 2*numStepsFixed
                                 , gLout
                                 , 1
                                 );
	if (theResult == kStopWasDone)
	  return(theResult);

	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
                 
	tmpX = eolX + 2*numStepsFixed * delX;
	tmpY = eolY + 2*numStepsFixed * delY;
	sumX2 = 0;
	sumY2 = 0;
	count2 = 0;
	for (i=0; i < 2*numStepsFixed; i++)
	  {
	    tmpX -= delX;
	    tmpY -= delY; 

#ifdef ZDEBUG
syslog( LOG_DEBUG
      , "coarsel %d %d %d %d %d %d del %d %d i %d  b dbgcnt %06d"
      , tmpX
      , tmpY
      , gLout[0+3*i]
      , gLout[1+3*i]
      , gLout[2+3*i]
      , numStepsFixed
      , delX
      , delY
      , i 
      , pLgMaster->debug_count
      ); /* ZDEBUG */
#endif  /* ZDEBUG */

	    tmpX = gLout[1+3*i];
	    tmpY = gLout[2+3*i];
	    if (gLout[3*i] < pLgMaster->gSenseThresholdCoarse && gLout[3*i] > 1)
	      {
		sumX2 += tmpX;
		sumY2 += tmpY;
		count2++;
	      }
	  }

#ifdef ZDEBUG
syslog( LOG_DEBUG, "cnt 1,2 %d %d", count1, count2 );
#endif
	if (count1 >= 1 && count2 >= 1)
	  {
	    // Coarse search found target approximate location.
            //  Finer search will hone on more-specific location
            //  for debugging, the two sets of sums were separated
            //
	    avgX = (int16_t)((sumX1+sumX2)/(count1+count2));
	    avgY = (int16_t)((sumY1+sumY2)/(count1+count2));
#ifdef ZDEBUG
	    syslog(LOG_NOTICE,"CoarseLeg Found %d points, avgX=%lf,avgY=%lf.  Move on to DoFineLevel.",count,avgX,avgY);
#endif
	    theResult = DoubleFineLevel(pLgMaster
                                   , avgX
                                   , avgY
                                   , foundX
                                   , foundY
                                   , &MinX
                                   , &MinY
                                   , &MaxX
                                   , &MaxY
                                   );
#ifdef ZDEBUG
syslog( LOG_DEBUG, "Fine min/max  %x %x %x %x", MinX, MinY, MaxX, MaxY );
#endif
            if (theResult == kStopWasDone || theResult == kCoarseNotFound)
              return(theResult);

	    // Coarse & Fine searches found target where-abouts.  Now time to get more exact location
	    *foundX = avgX;
	    *foundY = avgY;

            
#ifdef ZDEBUG
syslog(LOG_NOTICE
      ,"CoarseLeg->SuperSearch: DoFineLevel Found target at x=%lf,y=%lf,rc=%x"
      ,avgX
      ,avgY
      ,theResult
      );
#endif
            // extend search borders
            exMinX = MinX;
            exMaxX = MaxX;
            exMinY = MinY;
            exMaxY = MaxY;
            SetFineLevelResults( MinX
                               , MinY
                               , MaxX
                               , MaxY
                               , &exMinX
                               , &exMinY
                               , &exMaxX
                               , &exMaxY
                               );
#ifdef ZDEBUG
syslog( LOG_DEBUG, "SetFine min/max  %x %x %x %x", exMinX, exMinY, exMaxX, exMaxY );
#endif
	    theResult = DoubleSuperSearch(pLgMaster, foundX, foundY, &exMinX, &exMinY, &exMaxX, &exMaxY);
	    if (theResult == 0)
	      {
#ifdef ZDEBUG
syslog( LOG_NOTICE
      , "SuperSearch:  Found target at x=%lf,y=%lf"
      , *foundX
      , *foundY
      );
#endif
		return(0);
	      }
	    if (theResult == kStopWasDone)
	      return(theResult);
	  }
      }
    return(theResult);
}        


int DoubleFineLevel(struct lg_master *pLgMaster
                          , int16_t inpX
                          , int16_t inpY
                          , double  *foundX
                          , double  *foundY
                          , int16_t *outMinX
                          , int16_t *outMinY
                          , int16_t *outMaxX
                          , int16_t *outMaxY
                          )
{
    int16_t MinX, MinY, MaxX, MaxY; 
    int    result;
    int32_t stepSize,  nSteps;
    int16_t eolXPos=0, eolYPos=0, eolXNeg=0, eolYNeg=0;
    int16_t currentX, currentY, centerX, centerY;
    int16_t firstX, firstY, lastX, lastY;
    int16_t Xm, Ym;
        
    nSteps = 150;
    // Seed with starting input XY pair
    centerX = inpX;
    centerY = inpY;
    MinX   = inpX;
    MaxX   = inpX;
    MinY   = inpY;
    MaxY   = inpY;

    stepSize = kFineSearchStep;
    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;
	
    // Start with NegX/NegY pair
    // starting Y, NegX
    // also try to find dot size

    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * (nSteps/2));
    currentX = eolXNeg;
    currentY = centerY;
    result =  findFirstLast(pLgMaster
                           , &firstX
                           , &firstY
                           , &lastX
                           , &lastY
                           , &Xm
                           , &Ym
                           , currentX
                           , currentY
                           , stepSize
                           , 0
                           , nSteps
                           );
#ifdef ZDEBUG
syslog( LOG_NOTICE, "after nXcY 1stL result %d ", result );
#endif
    if (result == kStopWasDone || result == kCoarseNotFound)
      return(result);


    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &MinX, &MinY, &MaxX, &MaxY);
    *outMinX = MinX;
    *outMinY = MinY;
    *outMaxX = MaxX;
    *outMaxY = MaxY;

#ifdef ZDEBUG
syslog( LOG_NOTICE
      , "DoFineLevel: Found1242 minX=%x,maxX=%x,minY=%x,maxY=%x"
      , *outMinX
      , *outMaxX
      , *outMinY
      , *outMaxY
      );
#endif

    // Starting X, NegY
    currentX = centerX;
    currentY = eolYNeg;
    result =  findFirstLast(pLgMaster
                           , &firstX
                           , &firstY
                           , &lastX
                           , &lastY
                           , &Xm
                           , &Ym
                           , currentX
                           , currentY
                           , 0
                           , stepSize
                           , nSteps
                           );
#ifdef ZDEBUG
syslog( LOG_NOTICE, "after cXnY 1stL result %d ", result );
#endif
    if (result == kStopWasDone || result == kCoarseNotFound)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &MinX, &MinY, &MaxX, &MaxY);
    if ( MinX < *outMinX) *outMinX = MinX;
    if ( MinY < *outMinY) *outMinY = MinY;
    if ( MaxX > *outMaxX) *outMaxX = MaxX;
    if ( MaxY > *outMaxY) *outMaxY = MaxY;

#ifdef ZDEBUG
syslog( LOG_NOTICE
      , "DoFineLevel: Found1270 minX=%x,maxX=%x,minY=%x,maxY=%x"
      , *outMinX
      , *outMaxX
      , *outMinY
      , *outMaxY
      );
#endif

    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;

    //  End of original XY pair, on to next set.
    centerX = Xm;
    centerY = Ym;
    limitCalc(centerX, centerY, &eolXNeg, &eolXPos, &eolYNeg, &eolYPos, stepSize * (nSteps/2));


    // NegX, CtrY
    currentX = eolXNeg;
    currentY = centerY;
    result =  findFirstLast(pLgMaster, &firstX, &firstY, &lastX, &lastY, &Xm,
			    &Ym, currentX, currentY, stepSize, 0, nSteps);
#ifdef ZDEBUG
syslog( LOG_NOTICE, "after nXcY 1stL result %d ", result );
#endif
    if (result == kStopWasDone || result == kCoarseNotFound)
      return(result);
    // Set up for next round, look for min/max of current XY pair
    SetFineLevelResults(firstX, firstY, lastX, lastY, &MinX, &MinY, &MaxX, &MaxY);
    if ( MinX < *outMinX) *outMinX = MinX;
    if ( MinY < *outMinY) *outMinY = MinY;
    if ( MaxX > *outMaxX) *outMaxX = MaxX;
    if ( MaxY > *outMaxY) *outMaxY = MaxY;

#ifdef ZDEBUG
syslog( LOG_NOTICE
      , "DoFineLevel: Found1295 minX=%x,maxX=%x,minY=%x,maxY=%x"
      , *outMinX
      , *outMaxX
      , *outMinY
      , *outMaxY
      );
#endif

    if (IfStopThenStopAndNeg1Else0(pLgMaster))
      return kStopWasDone;

    *outMinX -= 4 * stepSize;
    *outMinY -= 4 * stepSize;
    *outMaxX += 2 * stepSize;
    *outMaxY += 2 * stepSize;
    centerX = Xm;
    centerY = Ym;
    *foundX = centerX;
    *foundY = centerY;

#ifdef ZDEBUG
syslog( LOG_NOTICE
      , "DoFineLevel: Found x=%lf,y=%lf,minX=%x,maxX=%x,minY=%x,maxY=%x"
      , *foundX
      , *foundY
      , *outMinX
      , *outMaxX
      , *outMinY
      , *outMaxY
      );
#endif

    return(0);
}


int doubleFirstLast(struct lg_master *pLgMaster
                          , int16_t *firstX
                          , int16_t *firstY
                          , int16_t *lastX
                          , int16_t *lastY
                          , double  *foundX
                          , double  *foundY
                          , int16_t currentX
                          , int16_t currentY
                          , int16_t xStep
                          , int16_t yStep
                          , uint16_t nSteps
                          )
{
	struct lg_xydata  xydata;
	struct lg_xydelta xydelta;
	int32_t           sumX=0, sumY=0;
	int32_t           count;
	int               i, rc;
        int16_t           tmpX, tmpY;
        int               firstFlag;

        firstFlag = 1;

#ifdef ZDEBUG
syslog(LOG_DEBUG, "1stL xy %x %x step %x %x n %d", currentX, currentY, xStep, yStep, nSteps );
#endif
	memset((char *)&xydata, 0, sizeof(struct lg_xydata));
	memset((char *)&xydelta, 0, sizeof(struct lg_xydata));

	// Coming into this function already have an approximate XY pair
	// so seed found XY with that set.
	// If new XY pair (ie. closer than original to pinpointed target
	// location, then will send that value.
	// Seed first/last.  First should always be first X/Y, since that's the starting point
	// They will be sorted later.
	*foundX = currentX;
	*foundY = currentY;
	*firstX = currentX;
	*firstY = currentY;
	*lastX = currentX;
	*lastY = currentY;
	
	xydata.xdata = currentX;
	xydata.ydata = currentY;
	xydelta.xdata = xStep;
	xydelta.ydata = yStep;
        pLgMaster->gSearchType = FINESEARCH;
#ifdef ZDEBUG
syslog( LOG_DEBUG, "1stL about to level  steps %d", nSteps );
#endif
	rc = DoLevelSearch( pLgMaster
                          , (struct lg_xydata *)&xydata
                          , (struct lg_xydelta *)&xydelta
                          , nSteps
                          , gLout
                          , 1
                          );
	if (rc)
	  return(rc);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

        // test for background level near start and finish

	sumX = 0;
	sumY = 0;
	count = 0;
        for (i=0; i < nSteps; i++)
	  {
	    tmpX = currentX + (i * xStep);
	    tmpY = currentY + (i * yStep);

#ifdef ZDEBUG
syslog( LOG_DEBUG
      , "1stLxyi %d %d %d %d %d  c %d  target %d dbgcnt %06d"
      , tmpX
      , tmpY
      , gLout[0+3*i]
      , gLout[1+3*i]
      , gLout[2+3*i]
      , count
      , pLgMaster->current_target
      , pLgMaster->debug_count
      ); /* ZDEBUG */
#endif  /* ZDEBUG */

	    tmpX = gLout[1+3*i];
	    tmpY = gLout[2+3*i];
	    if (gLout[3*i] < pLgMaster->gSenseThresholdFine && gLout[3*i] > 1)
	      {
                if ( firstFlag == 1 )
                  {
                    firstFlag = 0;
		    *firstX = tmpX;
		    *firstY = tmpY;
                  }
		*lastX = tmpX;
		*lastY = tmpY;
		sumX += tmpX;
		sumY += tmpY;
		count++;
	      }
	  }
        if (count == 0)
	  return(kFineNotFound);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

          // reject searches with signal at end or beginning
        if ( gLout[3] < pLgMaster->gSenseThreshold ||
             gLout[3*(nSteps-2)] < pLgMaster->gSenseThresholdFine ) {
          return(kFineNotFound);
        }

	if (count >= 1)
	  {
	    *foundX = (double)sumX/(double)count;
	    *foundY = (double)sumY/(double)count;
	    syslog(LOG_NOTICE,"FindFirstLast: Found target at x=%lf,y=%lf",*foundX,*foundY);
	    return(0);
	  }
	else
	  syslog(LOG_NOTICE,"FindFirstLast: unable to find target for x=%x,y=%x",currentX,currentY);
        return kFineNotFound;
}




int DoubleSuperSearch(struct lg_master *pLgMaster
               , double  *foundX
               , double  *foundY
               , int16_t *MinX
               , int16_t *MinY
               , int16_t *MaxX
               , int16_t *MaxY
               )
{
    int               result, sweep;
    int32_t           sumX, sumY;
    int32_t           countX, countY;
    int32_t           dummyCount, dummySum;
    int32_t           Dstep, XSpan, YSpan;
    int32_t           nSteps12, XSpan12, YSpan12;
    int16_t           firstX, firstY, lastX, lastY;
    double            tempX, tempY;
    double            avgX, avgY;
    int16_t           currentX, currentY, newX, newY;
    int16_t           Xmid, Ymid, Xlow, Ylow, Xhigh, Yhigh;
    int16_t           delNeg, delPos;
    uint16_t          nXsteps, nYsteps;

    // Initialize buffers for new sense
    gLoutIndex  = 0;
    gSuperIndex = 0;
    memset((char *)gXsuperSave, 0, SENSE_BUF_SIZE);
    memset((char *)gYsuperSave, 0, SENSE_BUF_SIZE);
    memset((char *)gSaveAvgX, 0, SENSE_BUF_SIZE);
    memset((char *)gSaveAvgY, 0, SENSE_BUF_SIZE);

    // extend the edges of search  
#ifdef ZDEBUG
syslog( LOG_DEBUG, "SuperSe2190 X min/max %x %x Y min/max %x %x", *MinX, *MaxX, *MinY, *MaxY );
#endif
    Xmid  = (*MaxX + *MinX)/2;
    Ymid  = (*MaxY + *MinY)/2;

    XSpan = *MaxX - *MinX;
    YSpan = *MaxY - *MinY;

    delNeg = -gSuperFineSearchStep;
    delPos = gSuperFineSearchStep;

     // for one or more sweeps
    if ( 2*YSpan < 240 ) {
       YSpan12 = 240;
    } else {
       YSpan12 = 2 * YSpan;
    }


    currentY = Ymid - (YSpan12 / 2 );
    nSteps12 = (uint16_t)((double)YSpan12 / (double)gSuperFineSearchStep);
    
     // move in positive Y
    result = doubleSuperFirstLast( pLgMaster
                           , &firstX
                           , &firstY
                           , &lastX
                           , &lastY
                           , &tempX 
                           , &tempY 
                           , Xmid
                           , currentY
                           , 0
                           , delPos
                           , nSteps12
                           );
    if ( firstX < *MinX ) *MinX = firstX;
    if ( firstY < *MinY ) *MinY = firstY;
    if ( firstX > *MaxX ) *MaxX = firstX;
    if ( firstY > *MaxY ) *MaxY = firstY;
    if ( lastX < *MinX ) *MinX = lastX;
    if ( lastY < *MinY ) *MinY = lastY;
    if ( lastX > *MaxX ) *MaxX = lastX;
    if ( lastY > *MaxY ) *MaxY = lastY;
#ifdef ZDEBUG
syslog( LOG_DEBUG, "SuperSe2235 X min/max %x %x Y min/max %x %x", *MinX, *MaxX, *MinY, *MaxY );
#endif

    currentY = Ymid + (YSpan12 / 2 );
    nSteps12 = (uint16_t)((double)YSpan12 / (double)gSuperFineSearchStep);
    
     // move in negative Y
    result = doubleSuperFirstLast( pLgMaster
                           , &firstX
                           , &firstY
                           , &lastX
                           , &lastY
                           , &tempX 
                           , &tempY 
                           , Xmid
                           , currentY
                           , 0
                           , delNeg
                           , nSteps12
                           );
    if ( firstX < *MinX ) *MinX = firstX;
    if ( firstY < *MinY ) *MinY = firstY;
    if ( firstX > *MaxX ) *MaxX = firstX;
    if ( firstY > *MaxY ) *MaxY = firstY;
    if ( lastX < *MinX ) *MinX = lastX;
    if ( lastY < *MinY ) *MinY = lastY;
    if ( lastX > *MaxX ) *MaxX = lastX;
    if ( lastY > *MaxY ) *MaxY = lastY;
#ifdef ZDEBUG
syslog( LOG_DEBUG, "SuperSe2264 X min/max %x %x Y min/max %x %x", *MinX, *MaxX, *MinY, *MaxY );
#endif

    

    if (pLgMaster->gMultipleSweeps >= 2) {
         // for two or more sweeps
        if ( 2*XSpan < 240 ) {
           XSpan12 = 240;
        } else {
           XSpan12 = 2 * XSpan;
        }



        currentX = Xmid - (XSpan12 / 2 );
        nSteps12 = (uint16_t)((double)XSpan12 / (double)gSuperFineSearchStep);
    
         // move in positive X
        result = doubleSuperFirstLast( pLgMaster
                               , &firstX
                               , &firstY
                               , &lastX
                               , &lastY
                               , &tempX 
                               , &tempY 
                               , currentX
                               , Ymid
                               , delPos
                               , 0
                               , nSteps12
                               );
        if ( firstX < *MinX ) *MinX = firstX;
        if ( firstY < *MinY ) *MinY = firstY;
        if ( firstX > *MaxX ) *MaxX = firstX;
        if ( firstY > *MaxY ) *MaxY = firstY;
        if ( lastX < *MinX ) *MinX = lastX;
        if ( lastY < *MinY ) *MinY = lastY;
        if ( lastX > *MaxX ) *MaxX = lastX;
        if ( lastY > *MaxY ) *MaxY = lastY;

#ifdef ZDEBUG
syslog( LOG_DEBUG, "SuperSe2306 X min/max %x %x Y min/max %x %x", *MinX, *MaxX, *MinY, *MaxY );
#endif
    
        currentX = Xmid + (XSpan12 / 2 );
        nSteps12 = (uint16_t)((double)XSpan12 / (double)gSuperFineSearchStep);
        
         // move in negative X
        result = doubleSuperFirstLast( pLgMaster
                               , &firstX
                               , &firstY
                               , &lastX
                               , &lastY
                               , &tempX 
                               , &tempY 
                               , currentX
                               , Ymid
                               , delNeg
                               , 0
                               , nSteps12
                               );
        if ( firstX < *MinX ) *MinX = firstX;
        if ( firstY < *MinY ) *MinY = firstY;
        if ( firstX > *MaxX ) *MaxX = firstX;
        if ( firstY > *MaxY ) *MaxY = firstY;
        if ( lastX < *MinX ) *MinX = lastX;
        if ( lastY < *MinY ) *MinY = lastY;
        if ( lastX > *MaxX ) *MaxX = lastX;
        if ( lastY > *MaxY ) *MaxY = lastY;

#ifdef ZDEBUG
syslog( LOG_DEBUG, "SuperSe2336 X min/max %x %x Y min/max %x %x", *MinX, *MaxX, *MinY, *MaxY );
#endif
    }
    *MaxX += kSuperExtend;
    *MaxY += kSuperExtend;
    *MinX -= kSuperExtend;
    *MinY -= kSuperExtend;

    XSpan = *MaxX - *MinX;
    YSpan = *MaxY - *MinY;

    nXsteps = (uint16_t)((double)XSpan / (double)gSuperFineSearchStep);
    nYsteps = (uint16_t)((double)YSpan / (double)gSuperFineSearchStep);
    delNeg = -gSuperFineSearchStep;
    delPos = gSuperFineSearchStep;

    Xmid  = (*MaxX + *MinX)/2;
    Ymid  = (*MaxY + *MinY)/2;
    Xlow  = *MinX - ((*MaxX - *MinX)/2);
    Xhigh = *MaxX + ((*MaxX - *MinX)/2);
    Ylow  = *MinY - ((*MaxY - *MinY)/2);
    Yhigh = *MaxY + ((*MaxY - *MinY)/2);
    // Xlow  = *MinX;
    // Xhigh = *MaxX;
    // Ylow  = *MinY;
    // Yhigh = *MaxY;

    sweep = 1;
    sumX = 0;
    sumY = 0;
    countX = 0;
    countY = 0;
    Dstep = 2 * delPos;
    dummySum = 0; dummyCount = 0;
    for (currentX = *MinX; currentX < *MaxX; currentX +=  Dstep ) {
	if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return(kStopWasDone);
	// Sweep 1---Scanset is (currentX, currentY, delNeg) and (currentX, MinY, delPos)
	//   since Y is moving fast, use this to determine X position
        result = SearchWithTwoSetsOutYplus(pLgMaster
                              , sweep
                              , currentX
                              , *MaxY
                              , 0
                              , delNeg
                              , (currentX+delPos)
                              , *MinY
                              , 0
                              , delPos
                              , gLout2
                              , gLout1
                              , nYsteps
                              , &countX
                              , &dummyCount
                              , &sumX
                              , &dummySum
                              );
	if (result)
	  return(result);

#ifdef ZDEBUG
	if (countX >= 1)
	    syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit countX %d, #samples %d,delPos=%x",sweep,countX,nYsteps,delPos);
#endif

    }
      // always do sweeps 1 and 2
      //   since X is moving fast, use this to determine Y position
    sweep = 2;
    Dstep = 2 * delPos;
    dummySum = 0; dummyCount = 0;
    for (currentY = *MinY; currentY < *MaxY; currentY +=  Dstep) {
            if (IfStopThenStopAndNeg1Else0(pLgMaster))
              return kStopWasDone;
              
            // Sweep 2---Scanset is (MaxX, currentY, delNeg) and (MinX, currentY, delPos)
            //   since X is moving fast, use this to determine Y position
            result = SearchWithTwoSetsOutXplus( pLgMaster
                                      , sweep
                                      , *MaxX
                                      , currentY
                                      , delNeg
                                      , 0
                                      , *MinX
                                      , (currentY + delPos)
                                      , delPos
                                      , 0
                                      , gLout2
                                      , gLout1
                                      , nXsteps
                                      , &dummyCount
                                      , &countY
                                      , &dummySum
                                      , &sumY
                                      );
            if (result)
              return(result);
#ifdef ZDEBUG
            if (countX >= 1 && countY >= 1)
              syslog(LOG_NOTICE,"SuperSearch:  Sweep %d hit count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
#endif
    }
    if ((countX < 3) && (gSuperIndex == 0)) {
        syslog(LOG_NOTICE,"SuperSearch: END: Not enough hits, Sweep %d, hit countX %d, #samples %d,delNeg=%x,delPos=%x",
            sweep, countX ,nYsteps,delNeg,delPos);
        return(kSuperFineTooFew);
    }
    if ((countY < 3) && (gSuperIndex == 0)) {
        syslog(LOG_NOTICE,"SuperSearch: END: Not enough hits, Sweep %d, hit countY %d, #samples %d,delNeg=%x,delPos=%x",
           sweep, countY ,nYsteps,delNeg,delPos);
        return(kSuperFineTooFew);
    }
    if (pLgMaster->gMultipleSweeps >= 3)
      {
	sweep = 3;
	Dstep = delPos;
	for (currentX = *MaxX; currentX > *MinX; currentX -=  Dstep)
	  {
    	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    // Sweep 3---Scanset is (currentX, MaxY, delNeg) and (currentX, MinY, delPos)
            result = SearchWithTwoSetsOutY(pLgMaster
                                  , sweep
                                  , currentX
                                  , *MaxY
                                  , 0
                                  , delNeg
                                  , currentX
                                  , *MinY
                                  , 0
                                  , delPos
                                  , gLout2
                                  , gLout1
                                  , nYsteps
                                  , &countX
                                  , &countY
                                  , &sumX
                                  , &sumY
                                  );
            if (result)
              return(result);
            if (countX >= 1 && countY >= 1)
              syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
	  }
      }
    if (pLgMaster->gMultipleSweeps >= 4)
      {
	sweep = 4;
	Dstep = delPos;
	for (currentY = *MaxY; currentY > *MinY; currentY -=  Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    // Sweep 4---Scanset is (MaxX, currentY, delNeg) and (MinX, currentY, delPos)
	    result = SearchWithTwoSetsOutX(pLgMaster
                                  , sweep
                                  , *MaxX
                                  , currentY
                                  , delNeg
                                  , 0
                                  , *MinX
                                  , currentY
                                  , delPos
                                  , 0
                                  , gLout2
                                  , gLout1
                                  , nXsteps
                                  , &countX
                                  , &countY
                                  , &sumX
                                  , &sumY
                                  );
	    if (result)
	      return(result);
	    if (countX >= 1 && countY >= 1)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
	  }
      }
    if (pLgMaster->gMultipleSweeps >= 5)
      {
	sweep = 5;
	//  start of diagonals
	Dstep = delPos;
	for (currentX=Xlow,currentY=Ymid; ((currentX<Xmid) && (currentY<Yhigh)); (currentX+=Dstep), (currentY+=Dstep))
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return kStopWasDone;
	    // Sweep 5---Scanset is (dbl, dblY, delPos, delNeg)
	    result = SearchSingleSetOutXY(pLgMaster
				 , sweep
                                 , currentX
                                 , currentY
                                 , delPos
                                 , delNeg
                                 , gLout1
                                 , nYsteps
                                 , &countX
                                 , &countY
                                 , &sumX
                                 , &sumY
                                 );
	    if (result)
	      return(result);

	    // Sweep 5---Scanset is (dblX+(delPos * steps), (dblY+(delPos*steps)), delNeg, delPos)
	    newX = currentX + (Dstep * nYsteps);
	    newY = currentY - (Dstep * nYsteps);
	    result = SearchSingleSetOutXY( pLgMaster
			         , sweep
                                 , newX
                                 , newY
                                 , delNeg
                                 , delPos
                                 , gLout1
                                 , nYsteps
                                 , &countX
                                 , &countY
                                 , &sumX
                                 , &sumY
                                 );
	    if (result)
	      return(result);
	    if (countX >= 1 && countY >= 1)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
	  }
      }
    if (pLgMaster->gMultipleSweeps >= 6)
      {
        sweep = 6;
        for (currentX=Xhigh,currentY=Ymid; ((currentX>Xmid) && (currentY<Yhigh)); currentX-=Dstep,currentY-=Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);

	    // Sweep 6---Scanset is (currentX, currentY, delNeg, delPos)
	    result = SearchSingleSetOutXY (pLgMaster
    			       , sweep
                               , currentX
                               , currentY
                               , delNeg
                               , delPos
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    // Sweep 6---Scanset is (currentX-(delPos * steps), (currentY+(delPos*steps)), delPos, delNeg)
	    newX = currentX - (Dstep * nYsteps);
	    newY = currentY + (Dstep * nYsteps);
	    result = SearchSingleSetOutXY( pLgMaster
    			       , sweep
                               , newX
                               , newY
                               , delPos
                               , delNeg
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    if (countX >= 1 && countY >= 1)
	        syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
          }
      }
    if (pLgMaster->gMultipleSweeps >= 7)
      {
        sweep = 7;
        for (currentX = Xmid, currentY = Yhigh; ((currentX < Xhigh) && (currentY > Ymid)); currentX += Dstep, currentY -= Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);
	    // Sweep 7---Scanset is (dbl, dblY, delNeg, delNeg)
	    result = SearchSingleSetOutXY( pLgMaster
    			       , sweep
                               , currentX
                               , currentY
                               , delNeg
                               , delNeg
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    // Sweep 7---Scanset is (currentX-(delPos * steps), (currentY-(delPos*steps)), delPos, delPos)
	    newX = currentX - (Dstep * nYsteps);
	    newY = currentY - (Dstep * nYsteps);
	    result = SearchSingleSetOutXY( pLgMaster
    			       , sweep
                               , newX
                               , newY
                               , delPos
                               , delPos
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    if (countX >= 1 && countY >= 1)
	        syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit count %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
          }
      }
    if (pLgMaster->gMultipleSweeps >= 8)
      {
        sweep = 8;
        for (currentX = Xmid, currentY = Ylow; ((currentX >Xlow) && (currentY<Ymid)); currentX -= Dstep, currentY += Dstep)
	  {
	    if (IfStopThenStopAndNeg1Else0(pLgMaster))
	      return(kStopWasDone);

	    // Sweep 8---Scanset is (currentX, currentY, delPos, delPos)
	    result = SearchSingleSetOutXY( pLgMaster
    			       , sweep
                               , currentX
                               , currentY
                               , delPos
                               , delPos
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    // Sweep 8---Scanset is (currentX+(delPos * steps), (currentY+(delPos*steps)), delNeg, delNeg)
	    newX =  currentX + (Dstep * nYsteps);
	    newY =  currentY + (Dstep * nYsteps);
	    result = SearchSingleSetOutXY( pLgMaster
    			       , sweep
                               , newX
                               , newY
                               , delNeg
                               , delNeg
                               , gLout1
                               , nYsteps
                               , &countX
                               , &countY
                               , &sumX
                               , &sumY
                               );
	    if (result)
	      return(result);
	    if (countX >= 1)
	      syslog(LOG_NOTICE,"SuperSearch:  Sweep %d, hit countX %d %d, #samples %d,delPos=%x",sweep,countX,countY,nYsteps,delPos);
          }
      }

    if ((countX < 3) && (gSuperIndex == 0))
      {
        syslog(LOG_NOTICE,"SuperSearch: END: Not enough hits, Sweep %d, hit countX %d, #samples %d,delNeg=%x,delPos=%x",
           sweep, countX ,nYsteps,delNeg,delPos);
        return(kSuperFineTooFew);
      }
    if ((countY < 3) && (gSuperIndex == 0))
      {
        syslog(LOG_NOTICE,"SuperSearch: END: Not enough hits, Sweep %d, hit countY %d, #samples %d,delNeg=%x,delPos=%x",
           sweep, countY ,nYsteps,delNeg,delPos);
        return(kSuperFineTooFew);
      }

    if (countX >= 3 && countY >= 3)
      {
        avgX = (double)sumX / (double)countX;
        avgY = (double)sumY / (double)countY;
        *foundX = avgX;
        *foundY = avgY;
        return(0);
      }
#if 0
    else if (gSuperIndex > 0)
      {
        sumX = 0;
        sumY = 0;
        for (i = 0; i < gSuperIndex; i++)
	  {
	    sumX += gXsuperSave[i];
	    sumY += gYsuperSave[i];
	  }
        avgX = (double)sumX / (double)gSuperIndex;
        avgY = (double)sumY / (double)gSuperIndex;
        *foundX = avgX;
        *foundY = avgY;
      }
#endif
  return(0);
}





int doubleSuperFirstLast(struct lg_master *pLgMaster
                         , int16_t *firstX
                         , int16_t *firstY
                         , int16_t *lastX
                         , int16_t *lastY
                         , double  *foundX
                         , double  *foundY
                         , int16_t currentX
                         , int16_t currentY
                         , int16_t xStep
                         , int16_t yStep
                         , uint16_t nSteps
                         )
{
	struct lg_xydata  xydata;
	struct lg_xydelta xydelta;
	int32_t           sumX=0, sumY=0;
	int32_t           count;
	int               i, rc;
        int16_t           tmpX, tmpY;
        int               firstFlag;

        firstFlag = 1;

#ifdef ZDEBUG
syslog(LOG_DEBUG, "superL xy %x %x step %x %x n %d", currentX, currentY, xStep, yStep, nSteps );
#endif
	memset((char *)&xydata, 0, sizeof(struct lg_xydata));
	memset((char *)&xydelta, 0, sizeof(struct lg_xydata));

	// Coming into this function already have an approximate XY pair
	// so seed found XY with that set.
	// If new XY pair (ie. closer than original to pinpointed target
	// location, then will send that value.
	// Seed first/last.  First should always be first X/Y, since that's the starting point
	// They will be sorted later.
	*foundX = currentX;
	*foundY = currentY;
	*firstX = currentX;
	*firstY = currentY;
	*lastX = currentX;
	*lastY = currentY;
	
	xydata.xdata = currentX;
	xydata.ydata = currentY;
	xydelta.xdata = xStep;
	xydelta.ydata = yStep;
        pLgMaster->gSearchType = SUPERSEARCH;
#ifdef ZDEBUG
syslog( LOG_DEBUG, "superL about to level  steps %d", nSteps );
#endif
	rc = DoLevelSearch( pLgMaster
                          , (struct lg_xydata *)&xydata
                          , (struct lg_xydelta *)&xydelta
                          , nSteps
                          , gLout
                          , 1
                          );
	if (rc)
	  return(rc);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

        // test for background level near start and finish

	sumX = 0;
	sumY = 0;
	count = 0;
        for (i=0; i < nSteps; i++)
	  {
	    tmpX = currentX + (i * xStep);
	    tmpY = currentY + (i * yStep);

#ifdef ZDEBUG
syslog( LOG_DEBUG
      , "superLxyi %d %d %d %d %d  c %d  target %d dbgcnt %06d"
      , tmpX
      , tmpY
      , gLout[0+3*i]
      , gLout[1+3*i]
      , gLout[2+3*i]
      , count
      , pLgMaster->current_target
      , pLgMaster->debug_count
      ); /* ZDEBUG */
#endif  /* ZDEBUG */

	    tmpX = gLout[1+3*i];
	    tmpY = gLout[2+3*i];
	    if (gLout[3*i] < pLgMaster->gSenseThresholdSuperFine && gLout[3*i] > 1)
	      {
                if ( firstFlag == 1 )
                  {
                    firstFlag = 0;
		    *firstX = tmpX;
		    *firstY = tmpY;
                  }
		*lastX = tmpX;
		*lastY = tmpY;
		sumX += tmpX;
		sumY += tmpY;
		count++;
	      }
	  }
        if (count == 0)
	  return(kFineNotFound);
        if (IfStopThenStopAndNeg1Else0(pLgMaster))
	  return kStopWasDone;

          // reject searches with signal at end or beginning
        if ( gLout[3] < pLgMaster->gSenseThresholdSuperFine ||
             gLout[3*(nSteps-2)] < pLgMaster->gSenseThresholdSuperFine ) {
          return(kFineNotFound);
        }

	if (count >= 1)
	  {
	    *foundX = (double)sumX/(double)count;
	    *foundY = (double)sumY/(double)count;
	    syslog(LOG_NOTICE,"doubleSuperFirstLast: Found target at x=%lf,y=%lf",*foundX,*foundY);
	    return(0);
	  }
	else
	  syslog(LOG_NOTICE,"doubleSuperFirstLast: unable to find target for x=%x,y=%x",currentX,currentY);
        return kFineNotFound;
}
