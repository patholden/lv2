#include <stdint.h>
#ifndef SEGPOLY_H
#define SEGPOLY_H

extern
int
segpoly( int npol
       , double *xp
       , double *yp
       , double xin
       , double yin
       , double xout
       , double yout
       , double *xseg
       , double *yseg
       , double *B_u
       )
;

#endif
