#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <syslog.h>
#include "segpoly.h"
#include "segtest.h"

int segpoly(int npol, double *xp, double *yp, double xin, double yin, double xout,
	    double yout, double *xseg, double *yseg, double *B_u)
{
      int i, j, c;
      double x1,y1,x2,y2,x3,y3,x4,y4;
      double x_inter, y_inter;
      double sqr, dist, min_dist;
      double x_del, y_del;
      double B_test;
      int itest;

      c = 0;
      min_dist = 10000000000.0;

      for (i = 0, j = npol-1; i < npol; j = i++) {
          x1 = xp[i];
          y1 = yp[i];
          x2 = xp[j];
          y2 = yp[j];
          x3 = xin;
          y3 = yin;
          x4 = xout;
          y4 = yout;
          itest = segtest(x1,y1,x2,y2,x3,y3,x4,y4,&x_inter,&y_inter,&B_test);
          if ( itest ) {
               x_del = x_inter - xin;
               y_del = y_inter - yin;
               sqr = x_del*x_del + y_del*y_del;
               dist = sqrt( sqr );
	       syslog(LOG_NOTICE, "dist %lf xy %lf %lf  ij %d %d",
		      dist, x_inter, y_inter, i, j);
               if ( dist < min_dist ) {
                   min_dist = dist;
                   *xseg = x_inter;
                   *yseg = y_inter;
                   *B_u = B_test;
                   c = 1;
               }
          }
      }
      return(c);
}
