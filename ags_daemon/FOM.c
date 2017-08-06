/*
 *  static char rcsid[] = "$Id: FOM.c,v 1.2 1999/05/04 15:32:35 ags-sw Exp $";
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
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
#include "FOM.h"
#include "Protocol.h"
#include "comm_loop.h"
#include "SensorRegistration.h"

void DosuperFOM (struct lg_master *pLgMaster, uint32_t respondToWhom )
{
    struct parse_superfom_resp   *pResp;
    struct k_targetinfo          tgt_info;
    int32_t                      respLen;
    int i;

    pResp = (struct parse_superfom_resp *)pLgMaster->theResponseBuffer;
    memset((char *)pResp, 0, sizeof(struct parse_superfom_resp));
    pResp->hdr.status = RESPGOOD;
    pResp->num_targets = get_number_of_points();
    pResp->chi_square = get_chisquare_val();
    for (i = 0; i < pResp->num_targets; i++)
      {
	if (pLgMaster->foundTarget[i] == 1)
	  {
	    get_target_info(&tgt_info, i);
	    pResp->tgt_info[i].tgt_number = tgt_info.tgt_number;
	    pResp->tgt_info[i].xdev    = tgt_info.xdev;
	    pResp->tgt_info[i].ydev    = tgt_info.ydev;
	  }
	else
	  {
	    pResp->tgt_info[i].tgt_number = -1;
	    pResp->tgt_info[i].xdev    = 0.0;
	    pResp->tgt_info[i].ydev    = 0.0;
	  }
      }
    // requires a variable length response, don't include unused targets
    respLen = offsetof(struct parse_superfom_resp, tgt_info) + (pResp->num_targets * sizeof(struct k_targetinfo));

    HandleResponse(pLgMaster, respLen, respondToWhom);

    return;
}
