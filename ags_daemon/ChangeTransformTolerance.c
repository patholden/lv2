/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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
#include "comm_loop.h"
#include "parse_data.h"
#include "Protocol.h"
#include "ChangeTransformTolerance.h"

void DoChangeTransformTolerance(struct lg_master *pLgMaster, struct parse_chngxfrmtol_parms *pInp)
{
    struct parse_chngxfrmtol_resp *pResp=(struct parse_chngxfrmtol_resp *)pLgMaster->theResponseBuffer;
    double      newTol;
  
    memset(pResp, 0, sizeof(struct parse_chngxfrmtol_resp));
    newTol = pInp->new_tolerance;

    if ((newTol >= 1.0e-10) && (newTol < 1.0))
      {
	pResp->hdr.status = RESPGOOD;
	pLgMaster->gArgTol = newTol;
      }
    else if (fabs(newTol+1.0) < 0.00001)
      pResp->hdr.status = RESPGOOD;
    else if (fabs(newTol) < 1.0e-30)
      {
	pResp->hdr.status = RESPGOOD;
	pLgMaster->gArgTol = GARGTOL_DEFAULT;
      }
    else
      pResp->hdr.status = RESPFAIL;

    // Send new value back to caller
    pResp->new_tolerance = pLgMaster->gArgTol;

    HandleResponse(pLgMaster, (sizeof(struct parse_chngxfrmtol_resp)-kCRCSize), kRespondExtern);

    return;
}
