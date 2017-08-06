#include <stdint.h>
#include <time.h>
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/ioctl.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "Protocol.h"
#include "SetBit.h"
#include "CRCHandler.h"

void DoSetBit(struct lg_master *pLgMaster, struct parse_setbit_parms *pInp, uint32_t respondToWhom )
{
      struct lg_xydata  xydata;
      struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
      unsigned int bitID;
      unsigned int bitValue;

      memset((char *)&xydata, 0, sizeof(struct lg_xydata));
      
      bitID      = pInp->bit_id;
      bitValue   = pInp->bit_value;

      pResp->hdr.status = RESPGOOD;
      switch ( bitID )
	{
	case 1:
	  // unblank intensity  (DIM?)
	  doLGSTOP(pLgMaster);
	  ioctl(pLgMaster->fd_laser, LGGETANGLE, &xydata);
	  doClearSearchBeam(pLgMaster);
	  xydata.ctrl_flags = 0;
	  if (bitValue == 0)
	    {
	      doWriteDevPoints(pLgMaster, (struct lg_xydata *)&xydata);
	    }
	  else if (bitValue == 1)
	    {
	      xydata.ctrl_flags = BEAMONISSET | LASERENBISSET;
	      doWriteDevPoints(pLgMaster, (struct lg_xydata *)&xydata);
	    }
	  else
	    pResp->hdr.status = RESPFAIL;
	  break;
	case 2:
	  // search intensity (HIGH?)
          doLGSTOP(pLgMaster);
	  ioctl(pLgMaster->fd_laser, LGGETANGLE, &xydata);
	  if (bitValue == 0)  
	    {
	      doClearSearchBeam(pLgMaster);
	      xydata.ctrl_flags = 0;
	      doWriteDevPoints(pLgMaster, (struct lg_xydata *)&xydata);
	    }
	  else if (bitValue == 1)
	    {
	      doSetSearchBeam(pLgMaster);
	      xydata.ctrl_flags = BRIGHTBEAMISSET | BEAMONISSET | LASERENBISSET;
	      doWriteDevPoints(pLgMaster, (struct lg_xydata *)&xydata);
	    }
	  else {
	    pResp->hdr.status = RESPFAIL;
	  }
	  break;
	default:
	  pResp->hdr.status = RESPFAIL;
	  break;
	}
      HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp) -kCRCSize), respondToWhom);
      return;
}
