/*************************************************************
 *                                                           *
 * Assembly Guidance Systems X86_64 test                     *
 * Copyright 2015.                                           *
 *                                                           *
 * Test Name:  xy_io                                         *
 * Command  :  "xy_io"                                       *
 * Input    :    NONE.                                       *
 * Description:  This test exercises the kernel laser device *
 *               driver.  It continuously sends ioctl to     *
 *               write xy sinusoidal pattern to DACs with    *
 *               default timer interval of 100 usec.         *
 *               Pattern is send once a second.              *
 *                                                           *
 ************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <stddef.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/io.h>
#include <math.h>
#include <linux/laser_api.h>
#include <string.h>

unsigned short lg_out_data[MAX_LG_BUFFER];

int main( int argc, char ** argv )
{
  //  union  lg_info  loc_info;
  int j, x_index, y_index;
  int  fd_laser;
  double ampl, pi, phase, jr;

  pi = 4.0 * atan2( 1.0, 1.0 );
  //  memset(&loc_info, 0, sizeof(loc_info));
  
  fd_laser = open( "/dev/laser", O_RDWR );
  if (fd_laser <= 0)
    {
      perror( "Unable to open /dev/laser" );
      exit(EXIT_FAILURE);
    }

  // Prep buffer to send to laser driver
  for ( x_index = 0; x_index < MAX_LG_BUFFER; x_index++)
    {
      y_index = x_index + 4;
      jr = (double)x_index;
      phase = jr  * ( (2.0 * pi) / 2000.0 );
      ampl  = (16383 * sin( phase ) + 16384);   // 0x4000 shifts to positive wave
      lg_out_data[x_index] = lg_out_data[y_index] = (int)ampl >> 8 | ((int)ampl & 0xFF);
    }
#if 0
  //FIXME---PAH---NEEDS UPDATE TO NEW API
  // Initialize laser driver
   ioctl( fd_laser, LGSTOP );
   ioctl( fd_laser, LGFASTCLOCK );
   ioctl( fd_laser, LGSTOPPULSE );

   // Write sinusoid pattern continuously until user hits ctrl-c
   while (1)
     {
       write(fd_laser, &lg_out_data, sizeof(lg_out_data));
       loc_info.num_points = sizeof(lg_out_data)/8;
       ioctl(fd_laser, LGLDWRTCNT, &loc_info);
       ioctl(fd_laser, LGQUICKCHECK);
       sleep(1);
     }
#endif
   exit(EXIT_SUCCESS);
}
