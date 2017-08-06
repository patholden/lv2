#include <stdint.h>
/*
 *  $Id: chisqr.h,v 1.2 2007/04/02 08:47:41 pickle Exp pickle $
 */


double chisqr(struct lg_master *pLgMaster, double params[]);
extern double chiX[kNumberOfFlexPoints];
extern double chiY[kNumberOfFlexPoints];
extern double chiZ[kNumberOfFlexPoints];
extern double chiXfoundAngle[kNumberOfFlexPoints];
extern double chiYfoundAngle[kNumberOfFlexPoints];
