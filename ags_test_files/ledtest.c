#include <stdio.h>
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
#define   LG_IO_SERVO        0x380
#define   LG_IO_OPTIC        0x388
#define   LG_IO_BOARD        0x38A
#define   LG_IO_TCPIP        0x38C
#define   LG_IO_LIGHT        0x390
#define   LG_IO_BOOST        0x391
#define   LG_IO_ERROR        0x392


int
main( int argc, char ** argv )
{
   int i;

   if (ioperm(LG_BASE, 32, 1)) {perror("ioperm"); exit(1);}
   outb( 0x00, LG_IO_OPTIC );

   for ( i = 0; i < 1000; i ++ ) {
       usleep( 500000 );

       printf( "0x01  bit\n" );
       outb( 0x01, LG_IO_OPTIC );
       usleep( 500000 );

       printf( "0x02  bit\n" );
       outb( 0x02, LG_IO_OPTIC );
       usleep( 500000 );

       printf( "0x04  bit\n" );
       outb( 0x04, LG_IO_OPTIC );
       usleep( 500000 );

       printf( "0x08  bit\n" );
       outb( 0x08, LG_IO_OPTIC );
       usleep( 500000 );

       printf( "0x10  comm error LED\n" );
       outb( 0x10, LG_IO_OPTIC );
       usleep( 500000 );

       printf( "0x20  bit\n" );
       outb( 0x20, LG_IO_OPTIC );
       usleep( 500000 );

       printf( "0x40  search power bit\n" );
       outb( 0x40, LG_IO_OPTIC );
       usleep( 500000 );

       printf( "0x80  bit\n" );
       outb( 0x80, LG_IO_OPTIC );
       usleep( 500000 );

       outb( 0x00, LG_IO_OPTIC );
       printf( "off  %6d\n", i );
   } 

}
