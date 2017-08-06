/*
static char rcsid[] = "$Id: FOM.c,v 1.2 1999/05/04 15:32:35 ags-sw Exp $";

*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "parse_data.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "RefreshRate.h"
#include "SensorRegistration.h"

void DoRefreshRate (struct lg_master *pLgMaster, uint32_t respondToWhom )
{
  struct parse_rfrshrt_resp *pResp;

  pResp = (struct parse_rfrshrt_resp *)pLgMaster->theResponseBuffer;
  memset(pResp, 0, sizeof(struct parse_rfrshrt_resp));
  pResp->hdr.status = RESPGOOD;
  pResp->num_points = get_num_disp_points();
  pResp->laser_period = pLgMaster->gPeriod;
  HandleResponse(pLgMaster, (sizeof(struct parse_rfrshrt_resp)-kCRCSize), respondToWhom);
   return;
}
 
void DoGetDacs(struct lg_master *pLgMaster, uint32_t respondToWhom )
{
   struct parse_getdacs_resp *pResp;
   struct lg_xydata    xydata;
 
   ioctl( pLgMaster->fd_laser, LGGETANGLE, &xydata);
        
   pResp = (struct parse_getdacs_resp *)pLgMaster->theResponseBuffer;
   memset(pResp, 0, sizeof(struct parse_getdacs_resp));
   pResp->hdr.status = RESPGOOD;
   pResp->x_dac = xydata.xdata;
   pResp->y_dac = xydata.ydata;  
   HandleResponse(pLgMaster, (sizeof(struct parse_getdacs_resp)-kCRCSize), respondToWhom);
  return;
}
