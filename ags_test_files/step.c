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
   int i,j;
   int sign;
   unsigned char hi, lo;
   unsigned short w;
   int step = 1;

   if (ioperm(LG_BASE, 32, 1)) {perror("ioperm"); exit(EXIT_FAILURE);}

   for ( i = 0; i < 100; i ++ ) {
     for ( j = 1; j < 32000; j *= 2 ) {
       hi = ( j & 0xFF00 ) >> 8;
       lo = ( j & 0x00FF );
       outb( 0x00, LG_IO_X0 );
       outb( 0x00, LG_IO_X1 );
       outb( lo  , LG_IO_X2 );
       outb( hi  , LG_IO_X3 );
       outb( 0x00, LG_IO_Y0 );
       outb( 0x00, LG_IO_Y1 );
       outb( lo  , LG_IO_Y2 );
       outb( hi  , LG_IO_Y3 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
       outb( 0x40, LG_IO_X0 );
       fprintf( stderr, " j  %04x\n", j & 0xffff );
       sleep( 2 );
       
       hi = 0;
       lo = 0;
       outb( 0x00, LG_IO_X0 );
       outb( 0x00, LG_IO_X1 );
       outb( lo  , LG_IO_X2 );
       outb( hi  , LG_IO_X3 );
       outb( 0x00, LG_IO_Y0 );
       outb( 0x00, LG_IO_Y1 );
       outb( lo  , LG_IO_Y2 );
       outb( hi  , LG_IO_Y3 );
       outb( 0x40, LG_IO_X0 );
       outb( 0xC0, LG_IO_X0 );
       outb( 0x40, LG_IO_X0 );
       usleep( 50000 );
     } 
   } 
   exit(EXIT_SUCCESS);
}
