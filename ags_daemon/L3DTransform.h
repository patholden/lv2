#ifndef LEDTRNSFORM_H
#define LEDTRNSFORM_H
/*   $Id: L3DTransform.h,v 1.1 1999/08/17 20:17:57 ags-sw Exp $   */

typedef struct { double oldLoc[3], xRad, yRad; }
  doubleInputPoint;

typedef struct { double rotMatrix[3][3], transVector[3]; }
  doubleTransform;

extern double minL3Distance;
 int32_t gBestTargetNumber;

unsigned char FindBestTransform( struct lg_master *pLgMaster
                               , doubleInputPoint *iPt
                               , doubleTransform *tr
                               , double deltaXHeight
                               , double tolerance
                               , double * bestCosine
                               );	

#endif // LEDTRNSFORM_H
