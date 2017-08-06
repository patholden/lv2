/*************************************************************
 *                                                           *
 * Assembly Guidance Systems X86_64 test                     *
 * Copyright 2015.                                           *
 *                                                           *
 * Test Name:  xy                                            *
 * Command  :  "xy <XXXX> <YYYY>"                            *
 * Input    :    12-bit hex value for coordinate.            *
 * Description:  This test writes alternation 1's & 0's      *
 *               patterns to X/Y IO registers.  First        *
 *               pattern is 0xAAAA, second is 0x5555.        *
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
   unsigned int xl, yl;

   if ( argc < 2 ) {
      printf( "usage: %s  <x in 16bit hex> <y in 16bit hex>\n", argv[0] );
      exit(EXIT_FAILURE);
   }

   sscanf( argv[1], "%x", &xl );
   sscanf( argv[2], "%x", &yl );

   printf( "x %5x    y %5x\n", xl, yl );

   if (ioperm(LG_BASE, 32, 1)) {perror("ioperm"); exit(1);}

       hi = ( xl & 0xFF00 ) >> 8;
       lo = ( xl & 0x00FF );

       outb( 0x00, LG_IO_X0 );
       outb( 0x00, LG_IO_X1 );
       outb( lo  , LG_IO_X2 );
       outb( hi  , LG_IO_X3 );
       hi = ( yl & 0xFF00 ) >> 8;
       lo = ( yl & 0x00FF );
       outb( 0x00, LG_IO_Y0 );
       outb( 0x00, LG_IO_Y1 );
       outb( lo  , LG_IO_Y2 );
       outb( hi  , LG_IO_Y3 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
       outb( 0x40, LG_IO_X0 );
       exit(EXIT_SUCCESS);
}
