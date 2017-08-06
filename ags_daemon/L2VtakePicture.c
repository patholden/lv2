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
#include <unistd.h>
#include <netdb.h>

#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "LaserInterface.h"
#include "LaserPattern.h"
#include "L2VtakePicture.h"
#include "3DTransform.h"
#include "CRCHandler.h"


struct sockaddr_in     tcp_srv_addr;
struct servent         tcp_serv_info;
struct hostent         tcp_host_info;

#define MAXLINE 1024
#define BUFFSIZE  (2048+3*512*512)

static char inbuff[BUFFSIZE];
static char hexbuff[BUFFSIZE];
static char outbuff[BUFFSIZE];


void L2VtakePicture ( struct lg_master *pLgMaster,
                     struct parse_takepic_parms * pInp,
                     uint32_t respondToWhom)
{
        struct parse_basic_resp  *pResp;
        double tmpDoubleArr[MAX_NEW_TRANSFORM_ITEMS];
        double partPoint[3];
        double outputPoint[3];
        // int16_t  data[2];

        char * tmpPtr;

        int index, i;
        transform projT;
        transform p2v_T;
        transform theT;
        uint32_t   inaddr;
        int             port = 1234;
        int             n, count;
        char            linebuff[MAXLINE];
        int length;
        int incount;
        int total;
        int fd;
        fd_set socks;
        int readsocks;
        int highsock = 0;
        struct timeval timeout;
        int hexcount;


        // Prep response packet
        pResp = (struct parse_basic_resp *)pLgMaster->theResponseBuffer;
        memset(pResp, 0, sizeof(struct parse_basic_resp));


        // Get points
        memcpy((char *)&tmpDoubleArr, (char *)&pInp->transform[0], sizeof(tmpDoubleArr));
        ArrayIntoTransform((double *)&tmpDoubleArr, &theT );




        ArrayIntoTransform( tmpDoubleArr, &projT );

        for( i=0; i<12; i++ ) {
                tmpDoubleArr[i] = pLgMaster->l2vtransform[i];
        }

        ArrayIntoTransform( tmpDoubleArr, &p2v_T );

        Multiply3DMatrices( p2v_T.rotMatrix, projT.rotMatrix, theT.rotMatrix ); 

        TransformPoint( &p2v_T, projT.transVector, theT.transVector );

syslog(LOG_NOTICE, "proj form mat %lf\n", projT.rotMatrix[0][0] );
syslog(LOG_NOTICE, "transform mat %lf\n", projT.rotMatrix[0][1] );
syslog(LOG_NOTICE, "transform mat %lf\n", projT.rotMatrix[0][2] );
syslog(LOG_NOTICE, "transform mat %lf\n", projT.rotMatrix[1][0] );
syslog(LOG_NOTICE, "transform mat %lf\n", projT.rotMatrix[1][1] );
syslog(LOG_NOTICE, "transform mat %lf\n", projT.rotMatrix[1][2] );
syslog(LOG_NOTICE, "transform mat %lf\n", projT.rotMatrix[2][0] );
syslog(LOG_NOTICE, "transform mat %lf\n", projT.rotMatrix[2][1] );
syslog(LOG_NOTICE, "transform mat %lf\n", projT.rotMatrix[2][2] );
syslog(LOG_NOTICE, "transform vec %lf\n", projT.transVector[0] );
syslog(LOG_NOTICE, "transform vec %lf\n", projT.transVector[1] );
syslog(LOG_NOTICE, "transform vec %lf\n", projT.transVector[2] );
syslog(LOG_NOTICE, "theT form mat %lf\n", theT.rotMatrix[0][0] );
syslog(LOG_NOTICE, "transform mat %lf\n", theT.rotMatrix[0][1] );
syslog(LOG_NOTICE, "transform mat %lf\n", theT.rotMatrix[0][2] );
syslog(LOG_NOTICE, "transform mat %lf\n", theT.rotMatrix[1][0] );
syslog(LOG_NOTICE, "transform mat %lf\n", theT.rotMatrix[1][1] );
syslog(LOG_NOTICE, "transform mat %lf\n", theT.rotMatrix[1][2] );
syslog(LOG_NOTICE, "transform mat %lf\n", theT.rotMatrix[2][0] );
syslog(LOG_NOTICE, "transform mat %lf\n", theT.rotMatrix[2][1] );
syslog(LOG_NOTICE, "transform mat %lf\n", theT.rotMatrix[2][2] );
syslog(LOG_NOTICE, "transform vec %lf\n", theT.transVector[0] );
syslog(LOG_NOTICE, "transform vec %lf\n", theT.transVector[1] );
syslog(LOG_NOTICE, "transform vec %lf\n", theT.transVector[2] );

		
  partPoint[0] = pInp->visionX;
  partPoint[1] = pInp->visionY;
  partPoint[2] = pInp->visionZ;


syslog(LOG_NOTICE, "in xyz %lf %lf %lf\n",partPoint[0],partPoint[1],partPoint[2]);

        TransformPoint ( &theT, partPoint, outputPoint );

syslog(LOG_NOTICE, "out xyz %lf %lf %lf\n",outputPoint[0],outputPoint[1],outputPoint[2]);


        TransformIntoArray( &theT, tmpDoubleArr );


syslog(LOG_NOTICE, "TIA\n" );


  

  bzero((char *) &tcp_srv_addr, sizeof(tcp_srv_addr));
  tcp_srv_addr.sin_family = AF_INET; 

  tcp_srv_addr.sin_port = htons(port);

syslog(LOG_NOTICE, "about to inet_addr\n" );

  if( (inaddr = inet_addr(pLgMaster->visionhost)) != INADDR_NONE) {
      bcopy((char *) &inaddr, (char *) &tcp_srv_addr.sin_addr, sizeof(inaddr));
      tcp_host_info.h_name = NULL;
  } else {
                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
                return;
  }

syslog(LOG_NOTICE, "about socket\n" );

  if( (fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                close (fd);
                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
                return;
  }

syslog(LOG_NOTICE, "about connect\n" );
   
        if( connect(fd,(struct sockaddr *)&tcp_srv_addr,sizeof(tcp_srv_addr)) < 0 ) {
                close (fd);
                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
                return;
  }

  memset( inbuff, 0, BUFFSIZE );
  memset( hexbuff, 0, BUFFSIZE );
  inbuff[0] = 0x30;  // kludge for kTakePicture

        index  = sizeof(int32_t);
        tmpPtr = &(inbuff[index]);

        memcpy((char *)tmpPtr, (char *)&tmpDoubleArr, sizeof(tmpDoubleArr));
        index  = sizeof(int32_t) + 12 * sizeof(double);
        tmpPtr = &(inbuff[index]);
        *(double *)(&(tmpPtr[ 0])) = partPoint[0];
        *(double *)(&(tmpPtr[ 8])) = partPoint[1];
        *(double *)(&(tmpPtr[16])) = partPoint[2];

        length = sizeof(int32_t) + 12 * sizeof(double) + 3 * sizeof(double);
        AppendCRC( (unsigned char *)inbuff, length );

        incount = length + kCRCSize;
        hexcount = 0;
        for ( i=0; i<incount; i++ ) {
            if ( (unsigned char)(inbuff[i]) >= 0x80 ) {
               hexbuff[hexcount] = 0x80;
               hexcount++;
               hexbuff[hexcount] = inbuff[i] - 0x80;
               hexcount++;
            } else {
               hexbuff[hexcount] = inbuff[i];
               hexcount++;
            }
// syslog(LOG_NOTICE, "about write %x  hex %d\n", 0xff&hexbuff[hexcount],hexcount );
        }


        total = 0;
        n = 1;
        while ( total < hexcount && n > 0 ) {
// syslog(LOG_NOTICE, "about write %d  total %d\n", hexcount, total );
            n = write( fd, &(hexbuff[total]), (hexcount-total) );
            if ( n < 0 ) {
                close( fd );
                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
                return;
            }
            total += n;
        }
        count = 0;
syslog(LOG_NOTICE, "about read \n" );
        FD_ZERO( &socks );
        FD_SET( fd, &socks );
        highsock = fd;
        timeout.tv_sec  = 1;
        timeout.tv_usec = 200000;

        readsocks = select( highsock+1
                          , &socks
                          , (fd_set *)0
                          , (fd_set *)0
                          , &timeout
                          );

        if ( readsocks > 0 && FD_ISSET( fd, &socks ) ) {

syslog(LOG_NOTICE, "about readsocks %d \n", readsocks );
            n = read(fd,linebuff,MAXLINE);
            strncpy( &(outbuff[count]), linebuff, n );
            count += n;
        }

syslog(LOG_NOTICE, "about close  read %d\n", n );

        close (fd);



#ifdef SPECIAL
    gettimeofday( &tv, &tz );
    printf( "Picture78 tv %d %d\n", tv.tv_sec, tv.tv_usec );
#endif
                pResp->hdr.status = RESPGOOD;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );
       return;

       //  PostCommand( kDarkAngle, (char *)data, kRespondExtern );

#ifdef SPECIAL
    gettimeofday( &tv, &tz );
    printf( "Picture85 tv %d %d\n", tv.tv_sec, tv.tv_usec );
#endif

}
