#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <values.h>
#include "segtest.h"

int segtest(double x1, double y1, double x2, double y2,
	    double x3, double y3, double x4, double y4,
	    double *x_inter, double *y_inter, double *B_u)
{
    double denom, Anumer, Bnumer, Au, Bu;

    *x_inter =  0.0;
    *y_inter =  0.0;
    *B_u     = -1.0;

    denom  = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1);
    Anumer = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3);
    Bnumer = (x2-x1)*(y1-y3) - (y2-y1)*(x1-x3);
    if (fabs(denom) < DBL_MIN)
      return(0);
    Au = Anumer / denom;
    Bu = Bnumer / denom;
    *x_inter = x1 + Au * (x2 - x1);
    *y_inter = y1 + Au * (y2 - y1);
    if ((Au>=0.0) && (Au<=1.0) && (Bu>=0.0) && (Bu<=1.0))
      {
	*B_u = Bu;
	return(1);
      }
    return(0);
}
