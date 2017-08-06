/*************************************************************
 *                                                           *
 * Assembly Guidance Systems X86_64 test                     *
 * Copyright 2015.                                           *
 *                                                           *
 * Test Name:  ramp                                          *
 * Command  :  "ramp"                                        *
 * Input    :    NONE.                                       *
 * Description:  This test writes multiple sinusoidal        *
 *               patterns to X/Y IO registers.  4 patterns   *
 *               are looped 100 times through.               *
 *                                                           *
 ************************************************************/
#include <stdlib.h>
#include <stdio.h>
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
   int sign;
   unsigned char hi, lo;
   unsigned short w;
   double ampl, pi, phase, jr;
   int step = 64;

   pi = 4.0 * atan2( 1.0, 1.0 );
   if (ioperm(LG_BASE, 32, 1)) {perror("ioperm"); exit(EXIT_FAILURE);}

   for ( i = 0; i < 100; i ++ ) {
     for ( j = -32000; j < 32000; j +=  step ) {
       sign = -1;
       usleep( 100 );
       hi = ( j & 0xFF00 ) >> 8;
       lo = ( j & 0x00FF );
       jr = (double)j;
       phase = jr  * ( (2.0 * pi) / 32000.0 ) ; 
       ampl  = sign * 32000.0 * sin( phase );
       hi = ( 0xFF00 & (int)ampl ) >> 8;
       lo = ( 0x00FF & (int)ampl );
       // printf( "%10.4lf %10.4lf  %2x %2x\n", phase, ampl, hi, lo );
       outb( 0x00, LG_IO_X0 );
       outb( 0x00, LG_IO_X1 );
       outb( lo  , LG_IO_X2 );
       outb( hi  , LG_IO_X3 );
       hi = ( j & 0xFF00 ) >> 8;
       lo = ( j & 0x00FF );
       outb( 0x00, LG_IO_Y0 );
       outb( 0x00, LG_IO_Y1 );
       outb( lo  , LG_IO_Y2 );
       outb( hi  , LG_IO_Y3 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
     } 
     for ( j = 32000; j > -32000; j-=  step ) {
       sign = 1;
       usleep( 100 );
       hi = ( j & 0xFF00 ) >> 8;
       lo = ( j & 0x00FF );
       jr = (double)j;
       phase = jr  * ( (2.0 * pi) / 32000.0 ) ; 
       ampl  = sign * 32000.0 * sin( phase );
       hi = ( 0xFF00 & (int)ampl ) >> 8;
       lo = ( 0x00FF & (int)ampl );
       // printf( "%10.4lf %10.4lf  %2x %2x\n", phase, ampl, hi, lo );
       outb( 0x00, LG_IO_X0 );
       outb( 0x00, LG_IO_X1 );
       outb( lo  , LG_IO_X2 );
       outb( hi  , LG_IO_X3 );
       hi = ( j & 0xFF00 ) >> 8;
       lo = ( j & 0x00FF );
       outb( 0x00, LG_IO_Y0 );
       outb( 0x00, LG_IO_Y1 );
       outb( lo  , LG_IO_Y2 );
       outb( hi  , LG_IO_Y3 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
     } 
     for ( j = -32000; j < 32000; j +=  step ) {
       sign = -1;
       usleep( 100 );
       hi = ( j & 0xFF00 ) >> 8;
       lo = ( j & 0x00FF );
       jr = (double)j;
       phase = jr  * ( (2.0 * pi) / 32000.0 ) ; 
       ampl  = sign * 32000.0 * sin( phase );
       w  = 0xFFFF & (int)ampl;
       // printf( "%10.4lf %10.4lf  %2x %2x\n", phase, ampl, hi, lo );
       outw( 0x0000, LG_IO_X0 );
       outw( w     , LG_IO_X2 );
       w  = 0xFFFF & j;
       outw( 0x0000, LG_IO_Y0 );
       outw( w     , LG_IO_Y2 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
     } 
     for ( j = 32000; j > -32000; j-=  step ) {
       sign =  1;
       usleep( 100 );
       hi = ( j & 0xFF00 ) >> 8;
       lo = ( j & 0x00FF );
       jr = (double)j;
       phase = jr  * ( (2.0 * pi) / 32000.0 ) ; 
       ampl  = sign * 32000.0 * sin( phase );
       phase = jr  * ( (2.0 * pi) / 32000.0 ) ; 
       ampl  = sign * 32000.0 * sin( phase );
       w  = 0xFFFF & (int)ampl;
       // printf( "%10.4lf %10.4lf  %2x %2x\n", phase, ampl, hi, lo );
       outw( 0x0000, LG_IO_X0 );
       outw( w     , LG_IO_X2 );
       w  = 0xFFFF & j;
       outw( 0x0000, LG_IO_Y0 );
       outw( w     , LG_IO_Y2 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
     } 
   } 
   exit(EXIT_SUCCESS);
}
