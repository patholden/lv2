#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>

#include "tan_init.h"

double *Xtans;
double *Ytans;
int    *Yflag;
int     gNtanpoly;

void
tan_init( void )
{
 Xtans = (double *)calloc( 1024, sizeof(double) );
 Ytans = (double *)calloc( 1024, sizeof(double) );
 Yflag = (int *)calloc( 1024, sizeof(int) );



   Xtans[0] = 0.616629;
   Ytans[0] = -0.554309;
   Yflag[0] = 0;
   Xtans[1] = 0.601776;
   Ytans[1] = -0.479975;
   Yflag[1] = 0;
   Xtans[2] = 0.588992;
   Ytans[2] = -0.420413;
   Yflag[2] = 0;
   Xtans[3] = 0.578087;
   Ytans[3] = -0.363206;
   Yflag[3] = 0;
   Xtans[4] = 0.568909;
   Ytans[4] = -0.307956;
   Yflag[4] = 0;
   Xtans[5] = 0.561333;
   Ytans[5] = -0.254315;
   Yflag[5] = 0;
   Xtans[6] = 0.555259;
   Ytans[6] = -0.201965;
   Yflag[6] = 0;
   Xtans[7] = 0.550611;
   Ytans[7] = -0.15062;
   Yflag[7] = 0;
   Xtans[8] = 0.547331;
   Ytans[8] = -0.100012;
   Yflag[8] = 0;
   Xtans[9] = 0.545379;
   Ytans[9] = -0.0498864;
   Yflag[9] = 0;
   Xtans[10] = 0.54473;
   Ytans[10] = 0;
   Yflag[10] = 0;
   Xtans[11] = 0.545379;
   Ytans[11] = 0.0498864;
   Yflag[11] = 0;
   Xtans[12] = 0.547331;
   Ytans[12] = 0.100012;
   Yflag[12] = 0;
   Xtans[13] = 0.550611;
   Ytans[13] = 0.15062;
   Yflag[13] = 0;
   Xtans[14] = 0.555259;
   Ytans[14] = 0.201965;
   Yflag[14] = 0;
   Xtans[15] = 0.561333;
   Ytans[15] = 0.254315;
   Yflag[15] = 0;
   Xtans[16] = 0.568909;
   Ytans[16] = 0.307956;
   Yflag[16] = 0;
   Xtans[17] = 0.578087;
   Ytans[17] = 0.363206;
   Yflag[17] = 0;
   Xtans[18] = 0.588992;
   Ytans[18] = 0.420413;
   Yflag[18] = 0;
   Xtans[19] = 0.601776;
   Ytans[19] = 0.479975;
   Yflag[19] = 0;
   Xtans[20] = 0.616629;
   Ytans[20] = 0.554309;
   Yflag[20] = 0;
   Xtans[21] = -0.616629;
   Ytans[21] = 0.554309;
   Yflag[21] = 0;
   Xtans[22] = -0.601776;
   Ytans[22] = 0.479975;
   Yflag[22] = 0;
   Xtans[23] = -0.588992;
   Ytans[23] = 0.420413;
   Yflag[23] = 0;
   Xtans[24] = -0.578087;
   Ytans[24] = 0.363206;
   Yflag[24] = 0;
   Xtans[25] = -0.568909;
   Ytans[25] = 0.307956;
   Yflag[25] = 0;
   Xtans[26] = -0.561333;
   Ytans[26] = 0.254315;
   Yflag[26] = 0;
   Xtans[27] = -0.555259;
   Ytans[27] = 0.201965;
   Yflag[27] = 0;
   Xtans[28] = -0.550611;
   Ytans[28] = 0.15062;
   Yflag[28] = 0;
   Xtans[29] = -0.547331;
   Ytans[29] = 0.100012;
   Yflag[29] = 0;
   Xtans[30] = -0.545379;
   Ytans[30] = 0.0498864;
   Yflag[30] = 0;
   Xtans[31] = -0.54473;
   Ytans[31] = 0;
   Yflag[31] = 0;
   Xtans[32] = -0.545379;
   Ytans[32] = -0.0498864;
   Yflag[32] = 0;
   Xtans[33] = -0.547331;
   Ytans[33] = -0.100012;
   Yflag[33] = 0;
   Xtans[34] = -0.550611;
   Ytans[34] = -0.15062;
   Yflag[34] = 0;
   Xtans[35] = -0.555259;
   Ytans[35] = -0.201965;
   Yflag[35] = 0;
   Xtans[36] = -0.561333;
   Ytans[36] = -0.254315;
   Yflag[36] = 0;
   Xtans[37] = -0.568909;
   Ytans[37] = -0.307956;
   Yflag[37] = 0;
   Xtans[38] = -0.578087;
   Ytans[38] = -0.363206;
   Yflag[38] = 0;
   Xtans[39] = -0.588992;
   Ytans[39] = -0.420413;
   Yflag[39] = 0;
   Xtans[40] = -0.601776;
   Ytans[40] = -0.479975;
   Yflag[40] = 0;
   Xtans[41] = -0.616629;
   Ytans[41] = -0.554309;
   Yflag[41] = 0;

   gNtanpoly = 42;


}

