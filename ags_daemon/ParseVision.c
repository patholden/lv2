#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>
#include <ctype.h>
#include <linux/laser_api.h>
#include <netinet/in.h>

#include "BoardComm.h"
#include "ParseVision.h"

int
ParseVision( struct lg_master *pLgMaster
          , unsigned char * in_buff
          , int size
          )
{
     char *ptr;
     double t1;
     int length;
     char testStr[256];
     int i;
     int c;
     int first;
     int err;
     int d1,d2,d3,d4;
     int narg;

     err = 0;

     pLgMaster->l2vtransform[ 0] = 1.0;
     pLgMaster->l2vtransform[ 1] = 0.0;
     pLgMaster->l2vtransform[ 2] = 0.0;
     pLgMaster->l2vtransform[ 3] = 0.0;
     pLgMaster->l2vtransform[ 4] = 1.0;
     pLgMaster->l2vtransform[ 5] = 0.0;
     pLgMaster->l2vtransform[ 6] = 0.0;
     pLgMaster->l2vtransform[ 7] = 0.0;
     pLgMaster->l2vtransform[ 8] = 1.0;
     pLgMaster->l2vtransform[ 9] = 0.0;
     pLgMaster->l2vtransform[10] = 0.0;
     pLgMaster->l2vtransform[11] = 0.0;
     sprintf( pLgMaster->visionhost, "10.1.3.1" );

     

     ptr = (char *)(&(in_buff[0]));
     i = 0;
     while ( *ptr && i < size ) {
          if ( *ptr == '\r' ) { *ptr = '\n'; }
          c = *ptr;
          if ( isupper( c ) ) { *ptr = tolower( c ); }
          ptr++; i++;
     }


     ptr = (char *)(&(in_buff[0]));
     i = 0;
     first = 1;
     while ( *ptr && i < size ) {
        if ( first == 0 ) {
              ptr++; i++;
        }
        if ( *ptr == '\n' || first == 1 ) {
         first = 0;
         while ( *ptr == '\n' ) {  ptr++; i++; }
         if ( *ptr ) {

          strcpy( testStr, "l2vtransform01 " ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform01 %lf" , &t1 );
            pLgMaster->l2vtransform[0] = t1; 
          }
          strcpy( testStr, "l2vtransform02 " ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform02 %lf" , &t1 );
            pLgMaster->l2vtransform[1] = t1; 
          }
          strcpy( testStr, "l2vtransform03 " ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform03 %lf" , &t1 );
            pLgMaster->l2vtransform[2] = t1; 
          }
          strcpy( testStr, "l2vtransform04 " ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform04 %lf" , &t1 );
            pLgMaster->l2vtransform[3] = t1; 
          }
          strcpy( testStr, "l2vtransform05 " ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform05 %lf" , &t1 );
            pLgMaster->l2vtransform[4] = t1; 
          }
          strcpy( testStr, "l2vtransform06 " ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform06 %lf" , &t1 );
            pLgMaster->l2vtransform[5] = t1; 
          }
          strcpy( testStr, "l2vtransform07 " ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform07 %lf" , &t1 );
            pLgMaster->l2vtransform[6] = t1; 
          }
          strcpy( testStr, "l2vtransform08 " ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform08 %lf" , &t1 );
            pLgMaster->l2vtransform[7] = t1; 
          }
          strcpy( testStr, "l2vtransform09 " ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform09 %lf" , &t1 );
            pLgMaster->l2vtransform[8] = t1; 
          }
          strcpy( testStr, "l2vtransform10" ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform10 %lf" , &t1 );
            pLgMaster->l2vtransform[9] = t1; 
          }
          strcpy( testStr, "l2vtransform11" ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform11 %lf" , &t1 );
            pLgMaster->l2vtransform[10] = t1; 
          }
          strcpy( testStr, "l2vtransform12" ); length = strlen(testStr);
          if ( strncmp( ptr, testStr, length ) == 0 ) {
            sscanf( ptr , "l2vtransform12 %lf" , &t1 );
            pLgMaster->l2vtransform[11] = t1; 
          }
          strcpy( testStr, "vision_ip " );
          if ( strncmp( ptr, testStr, strlen(testStr) ) == 0 ) {
            narg = sscanf( ptr, "vision_ip %d.%d.%d.%d", &d1, &d2, &d3, &d4);
            if ( narg == 4 ) {
               memset( pLgMaster->visionhost, 0, 128 );
               sprintf( pLgMaster->visionhost, "%d.%d.%d.%d", d1, d2, d3, d4 );
               syslog( LOG_NOTICE, "visionhost %s\n", pLgMaster->visionhost );
            }
          }


        }
      }
     }
     return err;
}
