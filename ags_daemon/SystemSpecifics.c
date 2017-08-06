/*
static char rcsid[] = "$Id: SystemSpecifics.c,v 1.10 2007/03/30 18:57:42 pickle Exp pickle $";
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "parse_data.h"
#include "BoardCommDefines.h"
#include "AppErrors.h"
#include "AppStrListIDs.h"
#include "QuickCheckManager.h"
#include "ROI.h"
#include "Init.h"
#include "Video.h"
#include "LaserInterface.h"
#include "LaserCmds.h"
#include "SensorSearch.h"
#include "SystemSpecifics.h"

static  int gFirstTimer = 1;

void DoSystemPeriodic (struct lg_master *pLgMaster)
{
    int doQuickCheck;
    int result;
    int32_t QCresult;
    int32_t diff;

    result = 0;
    QCresult = 0;
    if (pLgMaster->gDisplayFlag <= 0 )
      gFirstTimer = 1;
    if (gQCtimer < 0)
      QCresult = GetQCflag(pLgMaster);

#if defined(KITDEBUG)
    LTemp = GetQCcounter(pLgMaster);
    syslog(LOG_ERR, "SS42 %d %d %d %d", gQCtimer,QCresult,gFirstTimer,LTemp );
#endif
    if (gQCtimer > 0)
      {
#ifdef KITDEBUG
	syslog(LOG_ERR, "SS43 gDisplayFlag %d", pLgMaster->gDisplayFlag );
	syslog(LOG_ERR, "SS44 gFirstTimer %d", gFirstTimer );
	syslog(LOG_ERR, "SS45 gQCtimer %d", gQCtimer );
#endif
	if (pLgMaster->gDisplayFlag > 0)
	  {
	    if (gFirstTimer == 1)
	      {
		InitQCtimer();
		gFirstTimer = 0;
	      }
	    else
	      {
		diff = GetQCtimer();
#ifdef KITDEBUG
		syslog(LOG_ERR, "SS54 diff %d", diff );
#endif
		if (diff >= gQCtimer)
		  QCresult = 1;
		else
		  QCresult = 0;
	      }
	  }
      }

    if (QCresult > 0)
      {
#ifdef KITDEBUG
	syslog(LOG_ERR, "line 124 SystemSpecifics about to Stop QC %d", QCresult );
#endif
	gFirstTimer = 1;
	if (gQuickCheck == 1)
	  {
	    doQuickCheck = ShouldDoQuickCheck(); 
#ifdef KITDEBUG
	    syslog(LOG_ERR, "SS67 doQC %d", doQuickCheck );
#endif
	    if (doQuickCheck)
	      {
		SlowDownAndStop(pLgMaster);
		result = PerformPeriodicQuickCheck(pLgMaster);

		resetQCcounter(pLgMaster);
		if (result != kStopWasDone)
		  ResumeDisplay(pLgMaster);
	      }
	    else
	      {
		DummyQuickCheck();
		if (gQCtimer < 0)
		  resetQCcounter(pLgMaster);
	      }
	  }
	else
	  {
#ifdef KITDEBUG
	    syslog(LOG_NOTICE, "line 124 SystemSpecifics about to Stop QC %d", QCresult);
	    syslog(LOG_NOTICE, "line 125 SystemSpecifics gVideoCheck %d", gVideoCheck);
	    syslog(LOG_NOTICE, "line 126 SystemSpecifics gVideoCount %d", gVideoCount);
#endif
	    if (gVideoCheck == 1)
	      {
		SlowDownAndStop(pLgMaster);
		PerformVideoCheck (pLgMaster, 1);
		SetQCcounter(pLgMaster, gVideoCount );
		ResumeDisplay(pLgMaster);
	      } 
	  }
      }
    return;
}

int32_t SystemGoAhead ( void )
{
/*
**    static count = 5;
**    return count--;
*/
  return(1);
}
