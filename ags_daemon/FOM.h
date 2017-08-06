#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "BoardComm.h"

#ifndef FOM_H
#define FOM_H

void DoFOM ( uint32_t respondToWhom );
void DosuperFOM(struct lg_master *pLgMaster, uint32_t respondToWhom);

#define  MAXTRANSNUM  11000

extern double gFOM;
extern double gFOMavg;
extern double gDiffX[MAXTRANSNUM];
extern double gDiffY[MAXTRANSNUM];
extern int32_t gNumberOfPoints;
extern double gChisqr;

#endif
