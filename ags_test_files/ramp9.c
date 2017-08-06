/*************************************************************
 *                                                           *
 * Assembly Guidance Systems X86_64 test                     *
 * Copyright 2015.                                           *
 *                                                           *
 * Test Name:  ramp9                                         *
 * Command  :  "ramp9"                                       *
 * Input    :    NONE.                                       *
 * Description:  This test writes a single 4000-point        *
 *               sinusoidal to X/Y IO registers.             *
 *                                                           *
 ************************************************************/
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>

int main( int argc, char ** argv )
{
  unsigned char hi, lo;
  int  i;
  short j;

   if (ioperm(LG_BASE, LASER_REGION, 1)) {perror("ioperm"); exit(EXIT_FAILURE);}
   outb( STROBE_OFF_LASER_OFF, LG_IO_CNTRL1);

   for (i=0; i < 0x10000; i++)
     {
       for (j = 0; j <= 0xFFFF; j++) {
	 hi = j >> 8;
	 lo = j & 0xFF;
	 printf( "\n%2x%2x", hi, lo );
	 // Write XY data, data is applied to DAC input after lo byte is written
	 // so sequence is important.  hi byte then lo byte. 
	 outb( STROBE_OFF_LASER_ON, LG_IO_CNTRL1);
	 outb(hi, LG_IO_XH);
	 outb(lo, LG_IO_XL);
	 outb(hi, LG_IO_YH);
	 outb(lo, LG_IO_YL);
	 usleep(1);                  // Let data WRITE to DAC input register operation take place
	 // Strobe bit 0->1 latches data,
	 // Strobe bit 1->0 writes data to DAC
	 outb(STROBE_ON_LASER_ON, LG_IO_CNTRL1);
       }
       for (j = 0xFFFF; j >= 0; j--) {
	 hi = j >> 8;
	 lo = j & 0xFF;
	 printf( "\n%2x%2x", hi, lo );
	 // Write XY data, data is applied to DAC input after lo byte is written
	 // so sequence is important.  hi byte then lo byte. 
	 outb( STROBE_OFF_LASER_ON, LG_IO_CNTRL1);
	 outb(hi, LG_IO_XH);
	 outb(lo, LG_IO_XL);
	 outb(hi, LG_IO_YH);
	 outb(lo, LG_IO_YL);
	 usleep(1);                  // Let data WRITE operation take place
	 // Strobe bit 0->1 latches data,
	 // Strobe bit 1->0 writes data to DAC
	 outb(STROBE_ON_LASER_ON, LG_IO_CNTRL1);
       }
     }
   exit(EXIT_SUCCESS);
}
