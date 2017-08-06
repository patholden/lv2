/*
static char rcsid[] = "$Id$";

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
#include "parse_data.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "AppCommon.h"
#include "ChangeDisplayPeriod.h"

void DoChangeDisplayPeriod (struct lg_master *pLgMaster, struct parse_chngdisp_parms *pInp)
{
  struct parse_chngdisp_resp *pResp;
  uint32_t newPeriod;

  pResp = (struct parse_chngdisp_resp *)pLgMaster->theResponseBuffer;
  memset(pResp, 0, sizeof(struct parse_chngdisp_resp));

  newPeriod = pInp->displayPeriod;

  pResp->hdr.status = RESPGOOD;

#ifdef ZDEBUG
  syslog(LOG_NOTICE,"change display  %d\n", newPeriod );
#endif

  if ((newPeriod >= 50) && (newPeriod <= 150)) {
    pLgMaster->gPeriod = newPeriod;
    pResp->hdr.status = RESPGOOD;
  } else if (newPeriod == -1) {    // do not change period
    pResp->hdr.status = RESPGOOD;
  } else if (newPeriod == 0) {
    pLgMaster->gPeriod = pLgMaster->gSavePeriod;
    pResp->hdr.status = RESPGOOD;
  } else {
    pResp->hdr.status =RESPFAIL;
  }

  pResp->displayPeriod = pLgMaster->gPeriod;

#ifdef ZDEBUG
  syslog(LOG_NOTICE,"actual display  %d  status %x\n", pLgMaster->gPeriod, pResp->hdr.status );
#endif

  HandleResponse(pLgMaster, (sizeof(struct parse_chngdisp_resp)-kCRCSize),
		 kRespondExtern);
  return;
}
