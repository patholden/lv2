/*************************************************************
 *                                                           *
 * Assembly Guidance Systems X86_64 test                     *
 * Copyright 2015.                                           *
 *                                                           *
 * Test Name:  sinusoid1                                     *
 * Command  :  "sinusoid1"                                   *
 * Input    :    NONE.                                       *
 * Description:  Test to generate a 30hz sinusoid @ 22K      *
 *               samples per second for 1 second.  Loop      *
 *               provides a continuous wave output.          *
 *               Each sample maps position to a wave that    *
 *               cycles at 30 Hz. Each cycle on a sine       *
 *               wave's domain goes from 0 to 2 * PI,        *
 *               and we're going to do that 30 times         *
 *               or once in 22000/30 samples.                *
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
#include <linux/laser_api.h>
#include <sys/ioctl.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include "sinusoid.h"

#define LTC1597_BIPOLAR_OFFSET_MAX   0xFFFF
#define LTC1597_BIPOLAR_OFFSET_ZERO  0x8000
#define LTC1597_BIPOLAR_OFFSET_PLUS  0x8000
#define LTC1597_BIPOLAR_OFFSET_NEG   0x7FFF
#define LTC1597_BIPOLAR_MAX_INP_VAL1 0x7FFF
#define LTC1597_BIPOLAR_MAX_INP_VAL2 0x8000

static void lg_get_xydata_ltcval(int16_t *output_val, int16_t input_val)
{
  if (!input_val)
    *output_val = LTC1597_BIPOLAR_OFFSET_ZERO;
  else if ((input_val == LTC1597_BIPOLAR_MAX_INP_VAL1)
	   || (input_val == LTC1597_BIPOLAR_MAX_INP_VAL2))
    *output_val = LTC1597_BIPOLAR_OFFSET_MAX;
  else if (input_val < 0)
    *output_val = (input_val & LTC1597_BIPOLAR_OFFSET_NEG);
  else
    *output_val = input_val | LTC1597_BIPOLAR_OFFSET_PLUS;
  return;
}
int
main( int argc, char ** argv )
{
  int16_t sample;               // Sample position in wave
  int16_t dacvolts = 0;      // DAC voltage
  int16_t dac_val=0;
  int8_t  hi, lo;
  int  i;

  if (ioperm(LG_BASE, LASER_REGION, 1)) {perror("ioperm"); exit(EXIT_FAILURE);}

  while (1)
    {
      for (sample = 0; sample <= SAMPLERATE; sample++)
	{
	  // convert from [-1.0,1.0] level to signed int [-32768,32767]:
	  dacvolts = convert_level_to_signed_16_bit(sample);
	  lg_get_xydata_ltcval((int16_t *)&dac_val, dacvolts);
	  
	  // Calculate hi & low bytes to send to FPGA->DAC
	  hi = dacvolts >> 8;
	  lo = dacvolts & 0xFF;
	  printf("\n%04x", dacvolts);

	  outb(LASERENABLE|BRIGHTBEAM, LG_IO_CNTRL2);  // Apply CNTRL2 settings
	  // Write XY data, data is applied to DAC input after lo byte is written
	  // so sequence of write to high & low bytes is important,
	  // i.e.  hi byte then lo byte. 
	  outb(hi, LG_IO_XH);
	  outb(lo, LG_IO_XL);
	  outb(hi, LG_IO_YH);
	  outb(lo, LG_IO_YL);
	  // Strobe bit 0->1 latches data,
	  outb(0xC0, LG_IO_CNTRL1);
	  // Let data settle before going to DAC
	  usleep(1);
	  // Strobe bit 1->0 writes data to DAC
	  outb(0x40, LG_IO_CNTRL1);
	}
    }
   exit(EXIT_SUCCESS);
}
