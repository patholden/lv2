/*
static char rcsid[] = "$Id: RightOnCert.c,v 1.2 2000/05/05 23:54:44 ags-sw Exp pickle $";
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <stddef.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/ioctl.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "AppCommon.h"
#include "parse_data.h"
#include "3DTransform.h"
#include "AngleCorrections.h"
#include "Protocol.h"
#include "LaserInterface.h"
#include "RightOnCert.h"
#include "SensorSearch.h"
#ifdef NEW_TFIND
#include "TargetFind.h"
#endif
static double uptime(void);

void RightOnCert(struct lg_master *pLgMaster,
		 struct parse_rtoncert_parms *param,
		 uint32_t respondToWhom )
{
   struct parse_rtoncert_resp *pRespBuf;
   transform theTransform;
   double    aXd;
   double    aXe;
   double    aXf;
   double    aXg;
   double    aYd;
   double    aYe;
   double    aYf;
   double    aYg;
   double    inPt[3];
   double    outPt[3];
   double    time_diff;
   double    time_now;
   double    time_start;
   double    Xe;
   double    Xexpect;
   double    Xexternal;
   double    Xf;
   double    Xfound;
   double    XgeomAngle;
   double    Xin;
   double    Xtr;
   double    Ye;
   double    Yexpect;
   double    Yexternal; 
   double    Yf;
   double    Yfound;
   double    YgeomAngle;
   double    Yin;
   double    Ytr;
   double    Zin;
   double    Ztr;
   int32_t   RawGeomFlag;
   int32_t   XrawAngle;
   int32_t   YrawAngle;
   int       status;
   uint32_t  respLen = (sizeof(struct parse_rtoncert_resp) - kCRCSize);
   int16_t   bXe;
   int16_t   bXf;
   int16_t   bYe;
   int16_t   bYf;

#ifdef AGS_DEBUG
   syslog(LOG_DEBUG, "Entered Routine: RightOnCert");

   LogRightOnCertCommand(param, pLgMaster);
#endif

   pRespBuf = (struct parse_rtoncert_resp *)pLgMaster->theResponseBuffer;
   memset(pRespBuf, 0, sizeof(respLen));

   time_start = uptime();

   RawGeomFlag = param->inp_angle_flag;

   ArrayIntoTransform((double *)param->inp_transform, &theTransform);

   Xin = param->inp_Xin;
   Yin = param->inp_Yin;
   Zin = param->inp_Zin;

   inPt[0] = Xin;
   inPt[1] = Yin;
   inPt[2] = Zin; 

   TransformPoint(&theTransform, inPt, outPt);

   Xtr = (double)outPt[0];
   Ytr = (double)outPt[1];
   Ztr = (double)outPt[2];

   XrawAngle = param->inp_XrawAngle;
   YrawAngle = param->inp_YrawAngle;

   XgeomAngle = param->inp_XgeomAngle;
   YgeomAngle = param->inp_YgeomAngle;

   GeometricAnglesFrom3D(pLgMaster, Xtr, Ytr, Ztr, &aXd, &aYd);

   ConvertGeometricAnglesToMirror(&aXd, &aYd);

   aXe = aXd;
   aYe = aYd;

   ApplyPoly(pLgMaster, &aXe, &aYe);

   aXg = aXe;
   aYg = aYe;

   ConvertMirrorToGeometricAngles(&aXg, &aYg);

   XYFromGeometricAnglesAndZ(pLgMaster, aXg, aYg, Ztr, &Xe, &Ye );

   ConvertMirrorAnglesToBinary(aXe, aYe, &bXe, &bYe);

   if (RawGeomFlag == 1)
     {
       bXe = XrawAngle;
       bYe = YrawAngle;
     }
   else if (RawGeomFlag == 2)
     ConvertExternalAnglesToBinary(pLgMaster, XgeomAngle, YgeomAngle, &bXe, &bYe);

   switch(pLgMaster->gHeaderSpecialByte)
     {
       case 0:
         pLgMaster->gCoarse2Factor     = kCoarseFactorDef;
         pLgMaster->gCoarse2SearchStep = kCoarseSrchStpDef;
         break;
       case 1:
         pLgMaster->gCoarse2Factor     = kCoarseFactorMin;
         pLgMaster->gCoarse2SearchStep = kCoarseFactorDef;
         break;
       case 2:
         pLgMaster->gCoarse2Factor     = 2;
         pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;
         break;
       case 3:
         pLgMaster->gCoarse2Factor     = 4;
         pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;
         break;
       case 4:
         pLgMaster->gCoarse2Factor     = 8;
         pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;
         break;
       default:
         pLgMaster->gCoarse2Factor     = kCoarseFactorMin;
         pLgMaster->gCoarse2SearchStep = pLgMaster->gCoarse2Factor;
         break;
     }

#ifdef ZDEBUG
   pLgMaster->debug_count++;
#endif
   pLgMaster->current_target = 0;
#ifdef NEW_TFIND
   status = FindTarget(pLgMaster, bXe, bYe, &bXf, &bYf);
#else
   status = SearchForASensor(pLgMaster, bXe, bYe, &bXf, &bYf);
#endif
   if (status == kStopWasDone)
     {
       SearchBeamOff(pLgMaster);
       pRespBuf->hdr.status = RESPGOOD;
#ifdef AGS_DEBUG
       LogRightOnCertResponse(pRespBuf, sizeof(pRespBuf->hdr.hdr));
#endif
       HandleResponse(pLgMaster, sizeof(pRespBuf->hdr.hdr), respondToWhom);
       return;
     }

   ConvertBinaryToMirrorAngles(bXf, bYf, &aXf, &aYf);

   ConvertMirrorToGeometricAngles(&aXf, &aYf);

   XYFromGeometricAnglesAndZ(pLgMaster, aXf, aYf, Ztr, &Xf, &Yf);

   Xexpect = Xe;
   Yexpect = Ye;

   Xfound  = Xf;
   Yfound  = Yf;

   time_now = uptime( );
   time_diff = time_now - time_start;

   syslog(LOG_NOTICE, "RightOnCert time diff %lf  end/start %lf %lf", time_diff, time_now, time_start);

   if (status)
     {
       pRespBuf->hdr.status = RESPFAIL;
#ifdef AGS_DEBUG
       LogRightOnCertResponse(pRespBuf, sizeof(pRespBuf->hdr.hdr));
#endif
       HandleResponse(pLgMaster, sizeof(pRespBuf->hdr.hdr), respondToWhom);
       return;
     }
   else
     {
       pRespBuf->hdr.status = RESPGOOD;
       pRespBuf->Xtr = Xtr; 
       pRespBuf->Ytr = Ytr; 
       pRespBuf->Ztr = Ztr; 
       pRespBuf->Xfound = Xfound; 
       pRespBuf->Yfound = Yfound; 
       pRespBuf->Xexpect = Xexpect; 
       pRespBuf->Yexpect = Yexpect; 
       pRespBuf->bXf = bXf;
       pRespBuf->bYf = bYf;
       ConvertBinaryToExternalAngles(pLgMaster, bXf, bYf, &Xexternal, &Yexternal);
       pRespBuf->Xexternal = Xexternal;
       pRespBuf->Yexternal = Yexternal;
#ifdef AGS_DEBUG
       LogRightOnCertResponse(pRespBuf, respLen);
#endif
       HandleResponse(pLgMaster, respLen, respondToWhom);
       return;
     }
   
   return;
}

static double uptime(void)
{
    double dtime;
    double dummy;
    int    up_fd;
    int    up_size;
    char   up_buff[1024];

    up_fd = open("/proc/uptime", O_RDONLY);

    up_size = read(up_fd, up_buff, 1024);

    if (up_size == 1024) up_size = 1023;

    up_buff[up_size] = 0;

    close(up_fd);

    sscanf(up_buff, "%lf %lf", &dtime, &dummy);

    if (dtime < 0.0 && dtime > 1000000.0)
      {
         dtime = -1.0;
      }
    
    return dtime;
}


#ifdef AGS_DEBUG
void LogRightOnCertCommand(struct parse_rtoncert_parms *param, struct lg_master *pLgMaster)
{
    double           transform[MAX_NEW_TRANSFORM_ITEMS];
    int32_t          i;

    syslog(LOG_DEBUG, "CMD: HeaderSpecialByte: %02x", pLgMaster->gHeaderSpecialByte);

    syslog(LOG_DEBUG, "CMD: angleflag: %d", param->inp_angle_flag);

    memcpy(transform, param->inp_transform, sizeof(transform));
    for (i = 0; i < MAX_NEW_TRANSFORM_ITEMS; i++)
      {
	syslog(LOG_DEBUG, "CMD: transform[%d]: %f", i, transform[i]);
      }

    syslog(LOG_DEBUG, "CMD: inp_Xin: %f", param->inp_Xin);

    syslog(LOG_DEBUG, "CMD: inp_Yin: %f", param->inp_Yin);

    syslog(LOG_DEBUG, "CMD: inp_Zin: %f", param->inp_Zin);

    syslog(LOG_DEBUG, "CMD: inp_XrawAngle: %d", param->inp_XrawAngle);

    syslog(LOG_DEBUG, "CMD: inp_YrawAngle: %d", param->inp_YrawAngle);

    syslog(LOG_DEBUG, "CMD: inp_XgeomAngle: %f", param->inp_XgeomAngle);

    syslog(LOG_DEBUG, "CMD: inp_YgeomAngle: %f", param->inp_YgeomAngle);

    return;    
}


void LogRightOnCertResponse(struct parse_rtoncert_resp *pRespBuf, uint32_t respLen)
{    
  syslog(LOG_DEBUG, "RSP: hdr: %08x",htonl(pRespBuf->hdr.hdr));

    if (respLen <= sizeof(pRespBuf->hdr.hdr))
	return;

    syslog(LOG_DEBUG, "RSP: Xtr: %f", pRespBuf->Xtr);

    syslog(LOG_DEBUG, "RSP: Ytr: %f", pRespBuf->Ytr);

    syslog(LOG_DEBUG, "RSP: Ztr: %f", pRespBuf->Ztr);

    syslog(LOG_DEBUG, "RSP: Xfound: %f", pRespBuf->Xfound);

    syslog(LOG_DEBUG, "RSP: Yfound: %f", pRespBuf->Yfound);

    syslog(LOG_DEBUG, "RSP: Xexpect: %f", pRespBuf->Xexpect);

    syslog(LOG_DEBUG, "RSP: Yexpect: %f", pRespBuf->Yexpect);
 
    syslog(LOG_DEBUG, "RSP: bXf: %d", pRespBuf->bXf);

    syslog(LOG_DEBUG, "RSP: bYf: %d", pRespBuf->bYf);
 
    syslog(LOG_DEBUG, "RSP: Xexternal: %f", pRespBuf->Xexternal);

    syslog(LOG_DEBUG, "RSP: Yexternal: %f", pRespBuf->Yexternal);

    return;
}
#endif
