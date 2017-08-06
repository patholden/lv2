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
#include <ctype.h>
#include "AppCommon.h"
#include "BoardCommDefines.h"
#include "AppErrors.h"
#include "AppStrListIDs.h"
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "L3DTransform.h"
#include "3DTransform.h"
#include "SensorSearch.h"
#include "SensorRegistration.h"
#include "LaserInterface.h"
#include "Video.h"
#include "Init.h"
#include "ROI.h"
#include "Net.h"
#include "Hobbs.h"
#include "QuickCheckManager.h"
#include "polyfunc.h"
#include "readnewcal.h"


#define MAXBUFF 1000000

int
readnewcal( struct lg_master *pLgMaster )
{
    char * in_buff;
    char * ptr;
    char * nl;
    double t0;
    int in_fd;
    int i, j;
    double tol;
    double MMD;
    int Nproj=1;
    double extra;
    int number, result;
    char teststr[256];
    char tempbuf[256];
    int Ndollar;
    int count;


    for( i=0; i<27; i++ ) {
            pLgMaster->xMirrorCorr[i]  = 0.0;
            pLgMaster->yMirrorCorr[i]  = 0.0;
            pLgMaster->xMirrorInv[i]  = 0.0;
            pLgMaster->yMirrorInv[i]  = 0.0;
    }
    pLgMaster->xMirrorCorr[2] = 1.0;
    pLgMaster->yMirrorCorr[3] = 1.0;
    pLgMaster->xMirrorInv[2] = 1.0;
    pLgMaster->yMirrorInv[3] = 1.0;



    in_fd = open( "/laser/data/mirrorcorr", O_RDONLY );
    if (in_fd < 0) {
        syslog( LOG_ERR, "error opening mirrorcorr\n" );
        return( 0 );
    }

    in_buff = (char *)calloc( MAXBUFF, sizeof(char) );
    count = read( in_fd, in_buff, MAXBUFF );
    close( in_fd );

    strcpy( pLgMaster->astring[ 1] , "000001" );
    strcpy( pLgMaster->astring[ 2] , "00000X" );
    strcpy( pLgMaster->astring[ 3] , "00000Y" );
    strcpy( pLgMaster->astring[ 4] , "0000XY" );
    strcpy( pLgMaster->astring[ 5] , "0000XX" );
    strcpy( pLgMaster->astring[ 6] , "0000YY" );
    strcpy( pLgMaster->astring[ 7] , "000XXX" );
    strcpy( pLgMaster->astring[ 8] , "000YYY" );
    strcpy( pLgMaster->astring[ 9] , "000XXY" );
    strcpy( pLgMaster->astring[10] , "000XYY" );
    strcpy( pLgMaster->astring[11] , "00XXXX" );
    strcpy( pLgMaster->astring[12] , "00YYYY" );
    strcpy( pLgMaster->astring[13] , "00YXXX" );
    strcpy( pLgMaster->astring[14] , "00YYXX" );
    strcpy( pLgMaster->astring[15] , "00YYYX" );
    strcpy( pLgMaster->astring[16] , "0XXXXX" );
    strcpy( pLgMaster->astring[17] , "0YYYYY" );
    strcpy( pLgMaster->astring[18] , "0YXXXX" );
    strcpy( pLgMaster->astring[19] , "0YYXXX" );
    strcpy( pLgMaster->astring[20] , "0YYYXX" );
    strcpy( pLgMaster->astring[21] , "0YYYYX" );
    strcpy( pLgMaster->astring[22] , "XXXXXX" );
    strcpy( pLgMaster->astring[23] , "YYYYYY" );

    
    i = 1;
    ptr = &(in_buff[0]);
    while ( *ptr && i <= count ) {
      if ( islower( *ptr ) ) {
        *ptr = toupper( *ptr );
      }
      ptr++;
      i++;
    }

#ifdef ZDEBUG
syslog( LOG_NOTICE, "about to parse mirrorcorr" );
#endif

    ptr = &(in_buff[0]);
    i = 0;
    Ndollar = 0;
    while ( ptr ) {
      nl = (char *) strchr( ptr, '\n' );
      if ( nl == NULL ) {
        ptr = 0;
      } else {

        for ( j = 1; j <= 23; j++ ) {
          strcpy( teststr, "AX" );
          strcat( teststr, pLgMaster->astring[j] );
          result = strncmp( ptr, teststr, 8 );
          if ( result == 0 ) {
            sscanf( ptr, "%s %lf", tempbuf, &t0 );
            pLgMaster->xMirrorCorr[j]  = t0;
#ifdef ZDEBUG
syslog( LOG_NOTICE, "AX %d %le", j, t0 );
#endif
          } 
        }
        for ( j = 1; j <= 23; j++ ) {
          strcpy( teststr, "AY" );
          strcat( teststr, pLgMaster->astring[j] );
          result = strncmp( ptr, teststr, 8 );
          if ( result == 0 ) {
            sscanf( ptr, "%s %lf", tempbuf, &t0 );
            pLgMaster->yMirrorCorr[j]  = t0;
#ifdef ZDEBUG
syslog( LOG_NOTICE, "AY %d %le", j, t0 );
#endif
          } 
        }

        for ( j = 1; j <= 23; j++ ) {
          strcpy( teststr, "IX" );
          strcat( teststr, pLgMaster->astring[j] );
          result = strncmp( ptr, teststr, 8 );
          if ( result == 0 ) {
            sscanf( ptr, "%s %lf", tempbuf, &t0 );
            pLgMaster->xMirrorInv[j]  = t0;
#ifdef ZDEBUG
syslog( LOG_NOTICE, "IX %d %le", j, t0 );
#endif
          } 
        }
        for ( j = 1; j <= 23; j++ ) {
          strcpy( teststr, "IY" );
          strcat( teststr, pLgMaster->astring[j] );
          result = strncmp( ptr, teststr, 8 );
          if ( result == 0 ) {
            sscanf( ptr, "%s %lf", tempbuf, &t0 );
            pLgMaster->yMirrorInv[j]  = t0;
#ifdef ZDEBUG
syslog( LOG_NOTICE, "IY %d %le", j, t0 );
#endif
          } 
        }

        if (*ptr == '$') {
          Ndollar++;
#ifdef ZDEBUG
syslog( LOG_NOTICE, "dollar %d", Ndollar );
#endif
          if (Ndollar == 5) {
             number = sscanf( ptr, "$%d", &Nproj );
             if (number < 1) {Nproj = 1;}
             pLgMaster->gProjectorSerialNumber = Nproj;
          }
          if (Ndollar == 6) {
             number = sscanf( ptr, "$%lf", &tol );
             if (number < 1) {tol = 0.03;}
             pLgMaster->gTolerance = tol;
          }
          if (Ndollar == 7) {
             number = sscanf( ptr, "$%lf", &MMD );
             if (number < 1) {MMD = 0.44;}
             gDeltaMirror = MMD;
          }
          if (Ndollar == 8) {
             number = sscanf( ptr, "$%lf", &extra );
             if (number < 1) {extra = 0.1;}
             gMirrorThickness = extra;
             pLgMaster->gHalfMirror = 0.5 * extra;
          }
        }
        ptr = nl; ptr++;

      }


   }

   free( in_buff );

   return( 0 );
}
