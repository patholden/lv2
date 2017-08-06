/*************************************************************
 *                                                           *
 * Assembly Guidance Systems X86_64 test                     *
 * Copyright 2015.
 *                                                           *
 * Test Name:  xytest                                        *
 * Command  :  "xytest"                                      *
 * Input    :    NONE.                                       *
 * Description:  This test writes alternation 1's & 0's      *
 *               patterns to X/Y IO registers.  First        *
 *               pattern is 0xAAAA, second is 0x5555.        *
 *                                                           *
 ************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>

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
   int i;

   if (ioperm(LG_BASE, 32, 1)) {perror("ioperm"); exit(EXIT_FAILURE);}

   for ( i = 0; i < 1000; i ++ ) {
       sleep( 1 );
       printf( "AAAA %6d\n", i );
       outb( 0x00, LG_IO_X0 );
       outb( 0xAA, LG_IO_X1 );
       outb( 0xAA, LG_IO_X2 );
       outb( 0xAA, LG_IO_X3 );
       outb( 0x00, LG_IO_Y0 );
       outb( 0xAA, LG_IO_Y1 );
       outb( 0xAA, LG_IO_Y2 );
       outb( 0xAA, LG_IO_Y3 );
       outb( 0x00, LG_IO_X0 );
       outb( 0xFF, LG_IO_X0 );
       outb( 0x00, LG_IO_X0 );
       sleep( 1 );
       printf( "5555 %6d\n", i );
       outb( 0x00, LG_IO_X0 );
       outb( 0x55, LG_IO_X1 );
       outb( 0x55, LG_IO_X2 );
       outb( 0x55, LG_IO_X3 );
       outb( 0x00, LG_IO_Y0 );
       outb( 0x55, LG_IO_Y1 );
       outb( 0x55, LG_IO_Y2 );
       outb( 0x55, LG_IO_Y3 );
       outb( 0x00, LG_IO_X0 );
       outb( 0xFF, LG_IO_X0 );
       outb( 0x00, LG_IO_X0 );
   } 
   exit(EXIT_SUCCESS);
}
