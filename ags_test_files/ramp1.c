/*************************************************************
 *                                                           *
 * Assembly Guidance Systems X86_64 test                     *
 * Copyright 2015.                                           *
 *                                                           *
 * Test Name:  ramp2                                         *
 * Command  :  "ramp2"                                       *
 * Input    :    NONE.                                       *
 * Description:  This test writes a single 4000-point        *
 *               sinusoidal to X/Y IO registers.             *
 *                                                           *
 ************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <math.h>

#define   LG_BASE            0x380
#define   LG_IO_X0           0x380
#define   LG_IO_X1           0x381
#define   LG_IO_X2           0x382
#define   LG_IO_X3           0x383
#define   LG_IO_Y0           0x384
#define   LG_IO_Y1           0x385
#define   LG_IO_Y2           0x386
#define   LG_IO_Y3           0x387

int
main( int argc, char ** argv )
{
   int i,j;
   unsigned char hi, lo;
   unsigned short w;
   double ampl, pi, phase, jr;
   int step = 1;

   pi = 4.0 * atan2( 1.0, 1.0 );
   if (ioperm(LG_BASE, 32, 1)) {perror("ioperm"); exit(EXIT_FAILURE);}

   for ( j = 0; j < 4000; j +=  step )
     {
       jr = (double)j;
       phase = jr  * ( (2.0 * pi) / 2000.0 ) ; 
       ampl  = (16383 * sin( phase ) + 0x2000);   // 0x4000 shifts to positive wave
       hi = ( 0xFF00 & (int)ampl ) >> 8;
       lo = ( 0x00FF & (int)ampl );
       outb( 0x80, LG_IO_X0 );
       outb( lo , LG_IO_X2 );
       outb( hi , LG_IO_X3 );
       outb( lo , LG_IO_Y2 );
       outb( hi , LG_IO_Y3 );
       printf( "%10.4lf %10.4lf  %2x %2x\n", phase, ampl, hi, lo);
       outb( 0x00, LG_IO_X0 );
       outb( 0x80, LG_IO_X0 );
       usleep(1);
     }
   exit(EXIT_SUCCESS);
}
