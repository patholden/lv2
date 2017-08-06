/*
static char rcsid[]="$Id: Video.c,v 1.15 2000/09/28 19:46:11 ags-sw Exp ags-sw $";
*/
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <values.h>
#include <syslog.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "AppCommon.h"
#include "Video.h"
#include "LaserInterface.h"
#include "3DTransform.h"
#include "AngleCorrections.h"
#include "Net.h"
#include "Web.h"

#define MAXLINE 1024
#define BUFFSIZE  (2048+3*512*512)

#define kMaxFOD   50


double g_xFODmin =  10.0;
double g_xFODmax = 500.0;
double g_yFODmin =  10.0;
double g_yFODmax = 470.0;
/* 
** double g_xFODmin =   5.0;
** double g_xFODmax = 250.0;
** double g_yFODmin =   5.0;
** double g_yFODmax = 235.0;
*/

int g_FODflag = 0;

static char FromVideo[BUFFSIZE];
static char ToVideo[MAXLINE];

double gXbeam = 125.5;
double gYbeam = 119.3;
/* double gRadToVid = 1.0; */
double VidMatrix[2][2] = {
  {1.0, 0.0},
  {0.0, 1.0}
};
double InvMatrix[2][2] = {
  {1.0, 0.0},
  {0.0, 1.0}
};


double gMinIntenStep = 0.0;
double gLastInten = 0.0;




void   PerformVideoCheck( struct lg_master *pLgMaster, int respondFlag )
{
    int count;
    int slen;
    char * token;
    double  Red;
    double  Green;
    double  Blue;
    uint32_t iRed;
    uint32_t iBlue;
    uint32_t iGreen;
    int gonetest;
    int      return_code;

    // Response buffer needs to be initialized
    memset(pLgMaster->theResponseBuffer, 0, sizeof(COMM_RESP_ERROR_LEN1));

    GoToRaw(pLgMaster, (struct lg_xydata *)&pLgMaster->gCheckXY);
#if defined(SDEBUG) || defined(KITDEBUG)
    syslog(LOG_ERR, "check XY raw %x %X", pLgMaster->gXcheck, pLgMaster->gYcheck);
#endif

    count = sprintf( ToVideo,
      "GET  /cgi-bin/checkColor"
         "   HTTP/1.0\n\n"
             );

    usleep( gVideoPreDwell );
    slen = GetWebVideo(pLgMaster, ToVideo, count, FromVideo );
    if( slen <= 0 ) {
      return_code = kFail | kNoVideo;
      memcpy(pLgMaster->theResponseBuffer, &return_code, sizeof(return_code));
      HandleResponse (pLgMaster, sizeof(uint32_t), gRespondToWhom );
      return;
    }
#if defined(SDEBUG) || defined(KITDEBUG)
    write( STDOUT_FILENO, FromVideo, slen );
#endif

    token = strtok(FromVideo, "\n");   /* throw away first token */
    Red    = 0.0;
    Green  = 0.0;
    Blue   = 0.0;
    gonetest = 0;
    while( (token = strtok(NULL, "\n")) ) {
      if( strncmp( token, "red =", 5 ) == 0 ) {
           sscanf( token, "red = %lf", &Red );
      }
      if( strncmp( token, "grn =", 5 ) == 0 ) {
           sscanf( token, "grn = %lf", &Green );
      }
      if( strncmp( token, "blu =", 5 ) == 0 ) {
           sscanf( token, "blu = %lf", &Blue );
      }
      if( strncmp( token, "gonetest PASS", 13 ) == 0 ) {
           gonetest = 1;
      }
    }
    
    iRed     = (uint32_t)Red;
    iGreen   = (uint32_t)Green;
    iBlue    = (uint32_t)Blue;
    iRed     &= 0xFF;
    iGreen   &= 0xFF;
    iBlue    &= 0xFF;
    if ( gonetest ) {
      return_code = kItsGone + iRed + (iGreen << 8) + (iBlue << 16);
      memcpy(pLgMaster->theResponseBuffer, &return_code, sizeof(uint32_t));
      gVideoCheck = 0;
      gVideoCount = 0;
      gVideoPreDwell = 0;

      if ( respondFlag ) {
        HandleResponse (pLgMaster, (sizeof(uint32_t)), gRespondToWhom);
      }
      return;
    } else {
      return_code = kItsHere + iBlue + (iGreen << 8) + (iRed << 16);
      memcpy(pLgMaster->theResponseBuffer, &return_code, sizeof(uint32_t));
      if ( respondFlag ) {
        HandleResponse (pLgMaster, (sizeof(uint32_t)), gRespondToWhom);
      }
    }
}

void TwoDInvert( double VM[2][2], double IM[2][2] )
{
   double deter;

   deter = VM[0][0] * VM[1][1] - VM[0][1] * VM[1][0];
   if( fabs(deter) < DBL_MIN ) {
     IM[0][0] = 0.0;
     IM[0][1] = 0.0;
     IM[1][0] = 0.0;
     IM[1][1] = 0.0;
   } else {
     IM[0][0] = VM[1][1] / deter;
     IM[0][1] = - VM[0][1] / deter;
     IM[1][0] = - VM[1][0] / deter;
     IM[1][1] = VM[0][0] / deter;
   }
   

}
