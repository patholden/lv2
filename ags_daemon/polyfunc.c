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

void
polyfunc( struct lg_master *pLgMaster
        , double xin
        , double yin
        , double *xout
        , double *yout
        )
{
      double sum, term;
      int i,j;

      sum = 0.0;
      for ( i=1; i <= 23; i++ ) {
          term = 0.0;
          if ( pLgMaster->xMirrorCorr[i] != 0 ) {
              term = pLgMaster->xMirrorCorr[i];
              for ( j=0; j <= 5; j++ ) {
                 if ( pLgMaster->astring[i][j] == 'X' ) {
                     term = term * xin;
                 }
                 if ( pLgMaster->astring[i][j] == 'Y' ) {
                     term = term * yin;
                 }
              } 
          }
          sum += term;
      }
      *xout = sum;

      sum = 0.0;
      for ( i=1; i <= 23; i++ ) {
          term = 0.0;
          if ( pLgMaster->yMirrorCorr[i] != 0 ) {
              term = pLgMaster->yMirrorCorr[i];
              for ( j=0; j <= 5; j++ ) {
                 if ( pLgMaster->astring[i][j] == 'X' ) {
                     term = term * xin;
                 }
                 if ( pLgMaster->astring[i][j] == 'Y' ) {
                     term = term * yin;
                 }
              } 
          }
          sum += term;
      }
      *yout = sum;
}

void
invpolyfunc( struct lg_master *pLgMaster
        , double xin
        , double yin
        , double *xout
        , double *yout
        )
{
      double sum, term;
      int i,j;

      sum = 0.0;
      for ( i=1; i <= 23; i++ ) {
          term = 0.0;
          if ( pLgMaster->xMirrorInv[i] != 0 ) {
              term = pLgMaster->xMirrorInv[i];
              for ( j=0; j <= 5; j++ ) {
                 if ( pLgMaster->astring[i][j] == 'X' ) {
                     term = term * xin;
                 }
                 if ( pLgMaster->astring[i][j] == 'Y' ) {
                     term = term * yin;
                 }
              } 
          }
          sum += term;
      }
      *xout = sum;

      sum = 0.0;
      for ( i=1; i <= 23; i++ ) {
          term = 0.0;
          if ( pLgMaster->yMirrorInv[i] != 0 ) {
              term = pLgMaster->yMirrorInv[i];
              for ( j=0; j <= 5; j++ ) {
                 if ( pLgMaster->astring[i][j] == 'X' ) {
                     term = term * xin;
                 }
                 if ( pLgMaster->astring[i][j] == 'Y' ) {
                     term = term * yin;
                 }
              } 
          }
          sum += term;
      }
      *yout = sum;
}

