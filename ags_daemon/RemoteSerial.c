#include <stdint.h>
#include <stdlib.h>
#include <syslog.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <ctype.h>
#include <string.h>
#include <linux/laser_api.h>



#include <unistd.h>
#include <netdb.h>

#include <stdio.h>
#include <sys/time.h>

#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "LaserInterface.h"
#include "3DTransform.h"
#include "Protocol.h"
#include "CRCHandler.h"
#include "DoAutoFocusCmd.h"

#include "RemoteSerial.h"

struct sockaddr_in     tcp_srv_addr;
struct servent         tcp_serv_info;
struct hostent         tcp_host_info;

#define MAXLINE 1024
#define BUFFSIZE  (2048+3*512*512)

static char inbuff[BUFFSIZE];
static char hexbuff[BUFFSIZE];
static char outbuff[BUFFSIZE];


void RemoteSerial ( struct lg_master *pLgMaster
                  , char * buffer
                  , uint32_t respondToWhom
                  )
{

        int index, i;
        char * tmpPtr;
        int             fd;
        uint32_t   inaddr;
        int             port = 1234;
        int             n, count, count0;
        char            linebuff[MAXLINE];
        int length;
        int incount;
        int total;
        fd_set socks;
        int readsocks;
        int highsock = 0;
        struct timeval timeout;
        int hexcount;
        struct parse_autofocus_resp *pResp = (struct parse_autofocus_resp *)pLgMaster->theResponseBuffer;

        index  = 0;
        tmpPtr = &(buffer[index]);
        bzero((char *) &tcp_srv_addr, sizeof(tcp_srv_addr));
        tcp_srv_addr.sin_family = AF_INET; 
        tcp_srv_addr.sin_port = htons(port);

        if( (inaddr = inet_addr(pLgMaster->visionhost)) != INADDR_NONE) {
            bcopy((char *) &inaddr, (char *) &tcp_srv_addr.sin_addr, sizeof(inaddr));
            tcp_host_info.h_name = NULL;
        } else {
                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );

                return;
        }

	if( (fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                syslog( LOG_ERR, "can't create TCP socket\n");
                close (fd);

                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );

                return;
  }
   
        if( connect(fd,(struct sockaddr *)&tcp_srv_addr,sizeof(tcp_srv_addr)) < 0 ) {
                syslog( LOG_NOTICE, "can't connect to server\n");
                close (fd);
                pResp->hdr.status = RESPFAIL;
                HandleResponse (pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom );

                return;
  }

  memset( inbuff, 0, BUFFSIZE );
  memset( hexbuff, 0, BUFFSIZE );

        inbuff[0] = 0x64;
        inbuff[1] = 0xff & pLgMaster->gHeaderSpecialByte;
        index  = sizeof(int32_t);
        tmpPtr = &(inbuff[index]);

        for( i=0; i<32; i++ ) {
                tmpPtr[i] = buffer[i];
        }

        length = sizeof(int32_t) + 32;
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
        }

        total = 0;
        n = 1;
        while ( total < hexcount && n > 0 ) {
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
        memset( linebuff, 0, MAXLINE );
        memset( hexbuff, 0, MAXLINE );
        if ( pLgMaster->gHeaderSpecialByte & 0x02 ) {
          usleep( 200000 );

          count = 0;
          memset( linebuff, 0, MAXLINE );
               // try first read, which may be just a response
          if ( readsocks > 0 && FD_ISSET( fd, &socks ) ) {
            n = read(fd,linebuff,MAXLINE);
            memcpy( &(outbuff[count]), linebuff, n );
            count += n;
          }

          if ( n <= 20 ) {
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
                  memset( linebuff, 0, MAXLINE );
                  // too few bytes received
                  usleep( 200000 );
                  n = read(fd,linebuff,MAXLINE);
                  memcpy( &(outbuff[count]), linebuff, n );
                  count += n;
              }
          }



          count0 = 0;
          for ( i = 0; i < count; i++ ) {
             if ( (unsigned char)(outbuff[i]) == 0x80 ) {
                  hexbuff[count0] = outbuff[i+1] + 0x80;
                  i++;
             } else {
                  hexbuff[count0] = outbuff[i];
             }
             count0++;
          } 

            // skip over the six byte confirmation in hexbuff
            // and copy over just 32 bytes
          memcpy( &(outbuff[0]), &(hexbuff[6]), 32 );
        }

        close (fd);


       if ( pLgMaster->gHeaderSpecialByte & 0x02 ) {

                pResp->hdr.status  = kAutoFocusCmd;
                            // skip over header
                memcpy( &(pLgMaster->theResponseBuffer[4]), &(outbuff[4]), 32 );
                HandleResponse( pLgMaster
                              , (sizeof(struct parse_autofocus_resp) - kCRCSize)
                              , respondToWhom
                              );

       }
       return;
}
