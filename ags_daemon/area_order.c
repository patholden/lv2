#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <math.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "allace4.h"
#include "permutes.h"
#include "L3DTransform.h"
#include "area_order.h"
#include "heap.h"

void area_order(struct lg_master *pLgMaster, int  numin, doubleInputPoint *inPt, int *num_out, permutes *Combos,
		int32_t * reversedex, int32_t * found)
{
    double x[4], y[4];
    int count, i, j, k, l;
    double * area_array;
    int32_t   * area_index;
    double area;
    int ncombo;

    area_array = (double *)calloc( 200000, sizeof(double) );
    area_index = (int32_t *)calloc( 200000, sizeof(int32_t) );

    count = 1;
    for ( i=0; i < numin; i++ ) {
      if ( found[i] == 0 ) continue;
      for ( j=i; j < numin; j++ ) {
        if ( found[j] == 0 ) continue;
        for ( k=j; k < numin; k++ ) {
          if ( found[k] == 0 ) continue;
          for ( l=k; l < numin; l++ ) {
            if ( found[l] == 0 ) continue;
            if ( i!=j && i!=k && i!=l && j!=k && j!=l && k!=l ) {
              Combos[count].index1 = i;
              Combos[count].index2 = j;
              Combos[count].index3 = k;
              Combos[count].index4 = l;
              x[0] =  inPt[i].xRad; y[0] =  inPt[i].yRad;
              x[1] =  inPt[j].xRad; y[1] =  inPt[j].yRad;
              x[2] =  inPt[k].xRad; y[2] =  inPt[k].yRad;
              x[3] =  inPt[l].xRad; y[3] =  inPt[l].yRad;
              area = allace4( x, y );
              Combos[count].area = area;
              area_array[count] = area;
              area_index[count] = count;
              count++;
            }
          }
        }
      }
    }
    ncombo = count - 1;

    heap( ncombo, area_array, area_index );

    for ( i=1; i <= ncombo; i++ ) {
         reversedex[1+ncombo-i] = area_index[i];
    }
    *num_out = ncombo;
    return;
}
