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

static inline void lg_write_io_to_dac(unsigned short xval, unsigned short yval, int lg_laser_set);

static inline void lg_write_io_to_dac(unsigned short xval, unsigned short yval, int lg_laser_set)
{
  unsigned char xlo, xhi, ylo, yhi;
  unsigned char ctrl_on = 0;
  unsigned char ctrl_off = 0;

  // Isolate byte values
  xlo = xval & 0xFF;
  xhi = (xval & 0xFF00) >> 8;
  ylo = yval & 0xFF;
  yhi = (yval & 0xFF00) >> 8;

  // Strobe-bit high to load in registers
  if (lg_laser_set)
    {
      ctrl_on = STROBE_ON_LASER_ON;
      ctrl_off = STROBE_OFF_LASER_ON;
    }
  else
    {
      ctrl_on = STROBE_ON_LASER_OFF;
      ctrl_off = STROBE_OFF_LASER_OFF;
    }      
  // Write XY data, WRITE occurs in DAC input buffer when low byte is written
  outb( STROBE_OFF_LASER_ON, LG_IO_CNTRL1);
  outb(xhi, LG_IO_XH);
  outb(xlo, LG_IO_XL);
  outb(yhi, LG_IO_YH);
  outb(ylo, LG_IO_YL);
  // Latch Y data & write to DAC
  usleep(1);                  // Let data WRITE to DAC input register operation take place
  // Strobe bit 0->1 latches data,
  // Strobe bit 1->0 writes data to DAC
  outb( ctrl_on, LG_IO_CNTRL1);
  return;
}

int
main( int argc, char ** argv )
{
  unsigned short xdata, ydata;
  
  if (ioperm(LG_BASE, LASER_REGION, 1)) {perror("ioperm"); exit(EXIT_FAILURE);}

  while (1)
    {
      // Write 0x0000, 0x4000, 0x7FFF first without laser, then with laser
      xdata = ydata = 0x0000;
      printf( "\nY-hi %2x,  Y-lo %2x\n", (ydata >> 8), (ydata & 0xFF) );
      printf( "\nX-hi %2x,  X-lo %2x\n", (xdata >> 8), (xdata & 0xFF) );
      lg_write_io_to_dac(xdata, ydata, 0);
      sleep(1);
      xdata = ydata = 0x4000;
      printf( "\nY-hi %2x,  Y-lo %2x\n", (ydata >> 8), (ydata & 0xFF) );
      printf( "\nX-hi %2x,  X-lo %2x\n", (xdata >> 8), (xdata & 0xFF) );
      lg_write_io_to_dac(xdata, ydata, 0);
      sleep(1);
      xdata = ydata = 0x7FFF;
      printf( "\nY-hi %2x,  Y-lo %2x\n", (ydata >> 8), (ydata & 0xFF) );
      printf( "\nX-hi %2x,  X-lo %2x\n", (xdata >> 8), (xdata & 0xFF) );
      lg_write_io_to_dac(xdata, ydata, 0);
      sleep(1);
      xdata = ydata = 0x4000;
      printf( "\nY-hi %2x,  Y-lo %2x\n", (ydata >> 8), (ydata & 0xFF) );
      printf( "\nX-hi %2x,  X-lo %2x\n", (xdata >> 8), (xdata & 0xFF) );
      lg_write_io_to_dac(xdata, ydata, 1);
      sleep(1);
      xdata = ydata = 0x0000;
      printf( "\nY-hi %2x,  Y-lo %2x\n", (ydata >> 8), (ydata & 0xFF) );
      printf( "\nX-hi %2x,  X-lo %2x\n", (xdata >> 8), (xdata & 0xFF) );
      lg_write_io_to_dac(xdata, ydata, 1);
      sleep(1);
      xdata = ydata = 0x4000;
      printf( "\nY-hi %2x,  Y-lo %2x\n", (ydata >> 8), (ydata & 0xFF) );
      printf( "\nX-hi %2x,  X-lo %2x\n", (xdata >> 8), (xdata & 0xFF) );
      lg_write_io_to_dac(xdata, ydata, 1);
      sleep(1);
      xdata = ydata = 0x7FFF;
      printf( "\nY-hi %2x,  Y-lo %2x\n", (ydata >> 8), (ydata & 0xFF) );
      printf( "\nX-hi %2x,  X-lo %2x\n", (xdata >> 8), (xdata & 0xFF) );
      lg_write_io_to_dac(xdata, ydata, 1);
      sleep(1);
      xdata = ydata = 0x4000;
      printf( "\nY-hi %2x,  Y-lo %2x\n", (ydata >> 8), (ydata & 0xFF) );
      printf( "\nX-hi %2x,  X-lo %2x\n", (xdata >> 8), (xdata & 0xFF) );
      lg_write_io_to_dac(xdata, ydata, 1);
      sleep(1);
    }
   exit(EXIT_SUCCESS);
}
