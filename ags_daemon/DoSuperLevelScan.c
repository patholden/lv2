#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "DoSuperLevelScan.h"
#include "SensorSearch.h"
#include "Protocol.h"
#include "AppResponses.h"


// FIXME---PAH---NEEDS CMD/RESP FIXES
void DoSuperLevelScan(double dX, double dY )
{
      uint32_t xoff, yoff;
      uint32_t xout, yout;
      uint32_t delX, delY;
      uint32_t x, y;
      uint32_t nSteps;
      int index;

      uint32_t xmid;
      uint32_t ymid;
      char theResponseBuffer[sizeof ( uint32_t ) + kCRCSize];     

      *(uint32_t *)theResponseBuffer =
           ConvertExternalAnglesToBinary( dX, dY, &xmid, &ymid );

      memset( (void *)&(scandata[0]), 0, 66*66*sizeof(uint32_t) );
      
      if ( *(uint32_t *)theResponseBuffer )
      {
                *(uint32_t *)theResponseBuffer +=
                        kFail + kInputAngleOutOfRange;
                HandleResponse ( (char *)theResponseBuffer,
                        sizeof ( uint32_t ), kRespondExtern );
                return;
      }


      xoff = xmid - ((LSCNTSTEP * LSSTEPSIZE) / 2 );
      yoff = ymid - ((LSCNTSTEP * LSSTEPSIZE) / 2 );
   
      for ( y = 0x1U; y <= LSCNTSTEP; y += 0x2U ) {
          xout   =     LSSTEPSIZE + xoff;
          yout   = y * LSSTEPSIZE + yoff;
          delX   = LSSTEPSIZE;
          delY   = 0;
          nSteps = LSCNTSTEP;
          if ( DoLevelSearch( xout, yout, delX, delY, nSteps, gScan ) ) {
                   return;    // kStopWasDone should be the reason
          }
          for ( x = 0x1U; x <= LSCNTSTEP; x += 0x1U ) {
                 index = y * LSCNTSTEP + x;
                 scandata[index] = gScan[x-1];
          }
          xout   = LSCNTSTEP *  LSSTEPSIZE + xoff;
          yout   =   (y + 1) * LSSTEPSIZE + yoff;
          delX   = -LSSTEPSIZE;
          delY   = 0;
          nSteps = LSCNTSTEP;
          if ( DoLevelSearch( xout, yout, delX, delY, nSteps, gScan ) ) {
                   return;    // kStopWasDone should be the reason
          }
          for ( x = LSCNTSTEP; x > 0 ; x -= 0x1U ) {
                 index = (y + 1) * LSCNTSTEP + (LSCNTSTEP - x);
                 scandata[index] = gScan[x];
          }


      }

      SlowDownAndStop( );
      *(uint32_t *)theResponseBuffer = kOK;
      HandleResponse ( (char *)theResponseBuffer,
                        sizeof ( uint32_t ), kRespondExtern );
      return;
}

void
InitLevelScan( void )
{
      scandata = (uint32_t *)calloc(66*66, sizeof(uint32_t));
      gScan = (uint32_t *)calloc(1024, sizeof(uint32_t));
      
}
