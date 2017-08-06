#include <stdint.h>

#include "pnpoly.h"

int
pnpoly(int npol, double *xp, double *yp, double x, double y)
{
      int i, j, c;

      c = 0;

      for (i = 0, j = npol-1; i < npol; j = i++) {
        if ((((yp[i] <= y) && (y < yp[j])) ||
             ((yp[j] <= y) && (y < yp[i]))) &&
            (x < (xp[j] - xp[i]) * (y - yp[i]) / (yp[j] - yp[i]) + xp[i]))
          c = !c;
      }
      return c;
}
