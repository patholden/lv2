#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "DoCoarseScan.h"
#include "DoCoarseScan2.h"
#include "SensorSearch.h"
#include "Protocol.h"
#include "LaserInterface.h"

uint32_t DCS2minlevel = 70;

#define DELLEV 30

static int c2intsort( const void *elem1, const void *elem2 );

int DoCoarseScan2(struct lg_master *pLgMaster,
		  int16_t dX, int16_t dY,
		  uint16_t lsstep, uint16_t lscount,
		  int16_t *xfound, int16_t *yfound)
{
      struct lg_xydata  xydata;
      struct lg_xydelta  xydelta;
      int16_t Xarray[65536];
      int16_t Yarray[65536];
      double Dxsum, Dysum;
      double Dcount;
      int16_t xoff, yoff;
      int16_t x, y;
      uint32_t nSteps;
      int firstCoarse = 1;
      int index;
      int target_cnt;
      int halfStep;
      int notarget;
      int min_target = 6;
      int theResult;
      uint32_t testlevel;
      int16_t xmid;
      int16_t ymid;
      int Icount;
      int Ihalf;
      int16_t Ixavg;
      int16_t Iyavg;


      xmid = dX;
      ymid = dY;
      target_cnt = 0;

      memset((char *)&xydata, 0, sizeof(struct lg_xydata));
      memset((char *)&xydelta, 0, sizeof(struct lg_xydata));
      memset((char *)&Xarray[0], 0, sizeof(Xarray));
      memset((char *)&Yarray[0], 0, sizeof(Yarray));
      memset((char *)pLgMaster->coarsedata, 0, 512*512*sizeof(int16_t));

      xoff = xmid - ((lscount * lsstep) / 2 );
      yoff = ymid - ((lscount * lsstep) / 2 );
   
      testlevel = DELLEV;
      for ( x = 1; x <= (lscount+1); x += 2) {
          notarget = 0;
	  xydata.xdata = (x * lsstep) + xoff;
	  xydata.ydata = lsstep + yoff;
          xydelta.xdata   = 0;
          xydelta.ydata = lsstep;
          nSteps = lscount;
          theResult = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				    (struct lg_xydelta *)&xydelta, nSteps, pLgMaster->gScan,1);
          if ( theResult == kStopWasDone ) {
	    SearchBeamOff(pLgMaster);
                   return theResult;
          }
          for (y = 1; y <= lscount; y ++)
	    {
	      index = y * lscount + x;
	      if (x > 1)
		pLgMaster->coarsedata[index] = pLgMaster->gScan[y-1];
	      pLgMaster->gLsort[y-1] = pLgMaster->gScan[y-1];
	    }
          qsort(pLgMaster->gLsort, lscount, sizeof(int16_t), c2intsort);
          halfStep = lsstep / 2;
          testlevel = pLgMaster->gLsort[halfStep];
          if ( firstCoarse == 1 ) {
               firstCoarse = 0;
               DCS2minlevel = testlevel + DELLEV;
          } 
          if (x > 1)
	    {
              for (y = 1; y <= lscount; y ++)
		{
		  if (pLgMaster->gScan[y] >= DCS2minlevel)
		    {
		      target_cnt++;
		      notarget = 0;
		      break;
		    }
		  else
		    notarget = 1;
		}
	    }
          if ((notarget == 1) && (target_cnt >= 1))
	    break;
          if (target_cnt >= min_target)
	    break;

	  xydata.xdata = ((x + 1) * lsstep) + xoff;
	  xydata.ydata = (lscount * lsstep) + yoff;
          xydelta.xdata = 0;
          xydelta.ydata = -lsstep;
          nSteps = lscount;
          theResult = DoLevelSearch(pLgMaster, (struct lg_xydata *)&xydata,
				    (struct lg_xydelta *)&xydelta,
				    nSteps, pLgMaster->gScan,0);
          if (theResult == kStopWasDone)
	    {
	      SearchBeamOff(pLgMaster);
	      return theResult;
	    }
          for (y = lscount; y > 0 ; y -= 1)
	    {
	      index = ((lscount - y) * lscount) + (x + 1);
	      pLgMaster->coarsedata[index] = pLgMaster->gScan[y];
	    }
          for (y = lscount; y > 0 ; y -= 1)
	    {
	      if (pLgMaster->gScan[y] >= DCS2minlevel)
		{
		  target_cnt++;
		  notarget = 0;
		  break;
		}
	      else
		notarget = 1;
	    }
          if ((notarget == 1) && (target_cnt >= 1))
	    break;
          if (target_cnt >= min_target)
	    break;
      }

      SlowDownAndStop(pLgMaster);
      if (target_cnt == 0)
	return(kCoarseNotFound);

      Dxsum = 0.0;
      Dysum = 0.0;
      Dcount = 0.0;
      Icount = 0;
      for (y = 1; y <= lscount; y ++)
	{
          for (x = 1; x <= lscount; x ++)
	    {
	      index = y * lscount + x;
	      if (pLgMaster->coarsedata[index] > DCS2minlevel)
		{
		  Dxsum += (double)x;
		  Dysum += (double)y;
		  Dcount += 1.0;
		  Xarray[Icount] = x;
		  Yarray[Icount] = y;
		  Icount++;
		}
	    }
	}
      if (Dcount < 1.0)
	return(kCoarseNotFound);
      qsort(Xarray, Icount, sizeof(int16_t), c2intsort);
      qsort(Yarray, Icount, sizeof(int16_t), c2intsort);
      Ihalf = Icount / 2;
      Ixavg = Xarray[Ihalf];
      Iyavg = Yarray[Ihalf];
      *xfound = ((int16_t)Ixavg * lsstep) + xoff;
      *yfound = ((int16_t)Iyavg * lsstep) + yoff;
      return(0);
}


static int c2intsort(const void *elem1, const void *elem2)
{
    int16_t numone, numtwo;

    numone = *(const int16_t *)elem1;
    numtwo = *(const int16_t *)elem2;

    if ( numone < numtwo ) return -1;
    if ( numone > numtwo ) return  1;

    return 0;
}

