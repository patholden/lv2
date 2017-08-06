#ifndef POLYFUNC_H
#define POLYFUNC_H

#include "BoardComm.h"

extern
void
polyfunc( struct lg_master *pLgMaster
        , double xin
        , double yin
        , double *xout
        , double *yout
        )
;

extern
void
invpolyfunc( struct lg_master *pLgMaster
        , double xin
        , double yin
        , double *xout
        , double *yout
        )
;

extern
char astring[27][8];

#endif
