#include <stdint.h>
#include <math.h>
#include <stdio.h>

#define MINANGSEP  0.5

#include "shoelace4.h"
#include "allace4.h"

int sort_per[24][4] = { { 0, 1, 2, 3 }
                      , { 0, 1, 3, 2 }
                      , { 0, 2, 1, 3 }
                      , { 0, 2, 3, 1 }
                      , { 0, 3, 1, 2 }
                      , { 0, 3, 2, 1 }
                      , { 1, 0, 2, 3 }
                      , { 1, 0, 3, 2 }
                      , { 1, 2, 0, 3 }
                      , { 1, 2, 3, 0 }
                      , { 1, 3, 0, 2 }
                      , { 1, 3, 2, 0 }
                      , { 2, 0, 1, 3 }
                      , { 2, 0, 3, 1 }
                      , { 2, 1, 0, 3 }
                      , { 2, 1, 3, 0 }
                      , { 2, 3, 0, 1 }
                      , { 2, 3, 1, 0 }
                      , { 3, 0, 1, 2 }
                      , { 3, 0, 2, 1 }
                      , { 3, 1, 0, 2 }
                      , { 3, 1, 2, 0 }
                      , { 3, 2, 0, 1 }
                      , { 3, 2, 1, 0 }
                      };


double
allace4( double *xin, double *yin )
{
   int index, a, b, c, d;
   double area, maxarea;
   double x[4], y[4];

   maxarea = 0.0;

   for ( index = 0; index < 24; index++ ) {
        a = sort_per[index][0];
        b = sort_per[index][1];
        c = sort_per[index][2];
        d = sort_per[index][3];

        x[0] = xin[a];  y[0] = yin[a];
        x[1] = xin[b];  y[1] = yin[b];
        x[2] = xin[c];  y[2] = yin[c];
        x[3] = xin[d];  y[3] = yin[d];

        area = shoelace4( x, y );
        if ( area > maxarea ) {
            maxarea = area;
        }
    }
    return( maxarea );
}

