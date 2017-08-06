#include <stdint.h>
/* $Id: Video.h,v 1.4 2000/09/28 19:41:55 ags-sw Exp $ */

void    PerformVideoCheck ( struct lg_master *pLgMaster, int respondFlag );

void TwoDInvert( double VM[2][2], double IM[2][2] );

extern   double g_xFODmin;
extern   double g_xFODmax;
extern   double g_yFODmin;
extern   double g_yFODmax;

extern   int  g_FODflag;

extern   int gVideoCount;
extern   int gVideoCheck;
extern   uint32_t gVideoPreDwell;


extern double gMinIntenStep;
extern double gLastInten;

extern double gXbeam;
extern double gYbeam;

extern double VidMatrix[2][2];
extern double InvMatrix[2][2];

extern char webhost[128];
