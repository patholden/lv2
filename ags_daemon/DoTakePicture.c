#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/time.h>
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
#include "LaserInterface.h"
#include "LaserPattern.h"
#include "DoTakePicture.h"
#include "3DTransform.h"

// FIXME---PAH---data[] needs to be lg_xydata struct
void DoTakePicture ( struct lg_master *pLgMaster,
		     struct parse_takepic_parms * pInp,
		     uint32_t respondToWhom)
{
  struct parse_basic_resp *pResp;
  struct lg_xydata dark_angles;
  double tmpDoubleArr[MAX_NEW_TRANSFORM_ITEMS];
  double partPoint[3];
  double outputPoint[3];
  int16_t  data[2];
  transform theTransform;
  uint32_t theError;
#ifdef SPECIAL
  struct timeval  tv;
  struct timezone tz;
#endif
  
  // Prep response packet
  pResp = (struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  memset(pResp, 0, sizeof(struct parse_basic_resp));


  // Get points
  memcpy((char *)&tmpDoubleArr, (char *)&pInp->transform[0], sizeof(tmpDoubleArr));
  ArrayIntoTransform((double *)&tmpDoubleArr, &theTransform );
		
  partPoint[0] = pInp->visionX;
  partPoint[1] = pInp->visionY;
  partPoint[2] = pInp->visionZ;
  
  TransformPoint ( &theTransform, partPoint, outputPoint );
  theError = PointToBinary(pLgMaster, outputPoint, &data[0], &data[1]);
  if (theError)
    {
      syslog(LOG_ERR, "DoTakePicture failed out of range" );
      pResp->hdr.status = RESPFAIL;
      pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE; 
      HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
      return;
    }

#ifdef SPECIAL
  gettimeofday( &tv, &tz );
  syslog(LOG_NOTICE, "Picture78 tv %d %d", tv.tv_sec, tv.tv_usec );
#endif

//  note kDarkAngle  no longer works in PostCommand
//  PostCommand(pLgMaster, kDarkAngle, (char *)&data[0], kRespondExtern );
  dark_angles.xdata = data[0];
  dark_angles.ydata = data[1];

  PostCmdDarkAngle(pLgMaster, &dark_angles );

#ifdef SPECIAL
  gettimeofday( &tv, &tz );
  syslog(LOG_NOTICE, "Picture85 tv %d %d", tv.tv_sec, tv.tv_usec );
#endif
  return;
}
