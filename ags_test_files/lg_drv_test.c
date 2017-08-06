/*************************************************************
 *                                                           *
 * Assembly Guidance Systems X86_64 test                     *
 * Copyright 2015.                                           *
 *                                                           *
 * Test Name:  lg_drv_test                                   *
 * Command  :  "lg_drv_test"                                 *
 * Input    :    NONE.                                       *
 * Description:  Uses sinusoid1 data generation to push      *
 *               data through lg IOCTL.  See sinusoid1.c     *
 *               for details on how points are generated.    *
 *                                                           *
 ************************************************************/
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <stddef.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "sinusoid.h"

uint8_t lg_in_data[MAX_TGFIND_BUFFER];

int
main( int argc, char ** argv )
{
  struct cmd_rw cmd_data;
  struct lg_xydata *pXYData;
  int laser_fd;
  uint16_t sample;               // Sample position in wave
  uint16_t dacvolts = 0;      // DAC voltage
  unsigned char hi, lo;
  int  i;
  int  rc;
  int  qcCount=0;
  
  // Init buffers
  memset((char *)&cmd_data, 0, sizeof(struct cmd_rw_base));
  memset((char *)&lg_in_data[0], 0, sizeof(lg_in_data));
    
  // Open LG laser device for writing
  laser_fd = open("/dev/laser", O_RDWR);
  if (!laser_fd)
    {
      fprintf(stderr,"\nlaser device %d not opened,errno %d\n", laser_fd, errno);
      exit(EXIT_FAILURE);
    }
  fprintf(stderr,"\n Test LGGETQCOUNTER");
  rc = ioctl(laser_fd, LGGETQCCOUNTER, (int *)&qcCount);
  if (rc < 0)
    fprintf(stderr,"\nIOCTL: GETQCCNTR rc %d, errno %d", rc,errno);
  else
    fprintf(stderr,"\nIOCTL: GETQCCNTR qcCount %d", qcCount);  
  sleep(1);
  
  // ioctl write-command-data test
  cmd_data.base.hdr.cmd = CMDW_SETQCCOUNTER;
  cmd_data.base.hdr.length = sizeof(uint32_t);
  cmd_data.base.cmd_data.dat32.val32 = -1;
  rc = write(laser_fd, (char *)&cmd_data, sizeof(struct cmd_rw_base));
  if (rc < 0)
    fprintf(stderr,"\nWRITE-CMD: SETQCCNT ERROR rc%d, errno %d\n", rc, errno);
  else
    fprintf(stderr,"\nWRITE-CMD: SETQCCNT qcval %d\n", cmd_data.base.cmd_data.dat32.val32);
  sleep(1);
  // Write test.  Use sinusoid1 for setting up sine data points.
  for (sample = 0; sample <= SAMPLERATE; sample++)
    {
      pXYData = (struct lg_xydata *)&cmd_data.xydata[sample];
      // convert from [-1.0,1.0] level to signed int [-32768,32767]:
      dacvolts = convert_level_to_signed_16_bit(sample);
      pXYData->xdata = dacvolts;
      pXYData->ydata = dacvolts;
      pXYData->ctrl_flags = BEAMONISSET | LASERENBISSET | BRIGHTBEAMISSET;
    }
  cmd_data.base.hdr.cmd = CMDW_BUFFER;
  cmd_data.base.hdr.test_mode = 1;
  cmd_data.base.hdr.length = sizeof(struct lg_xydata) * SAMPLERATE;
  rc = write(laser_fd, (char *)&cmd_data, sizeof(struct cmd_rw));
  if (rc < 0)
    fprintf(stderr,"\nWRITE-CMD: SINE err %d, errno %d\n", rc, errno);
  else
    fprintf(stderr,"\nWRITE-CMD: SINE Success!  count %d\n", rc);
  sleep(1);
  // Read test
  rc = read(laser_fd, (char *)&lg_in_data[0],sizeof(lg_in_data));
  if (rc < 0)
    fprintf(stderr,"\nREAD-LG: err %d, errno %d\n", rc, errno);
  else
    fprintf(stderr,"\nREAD-LG:  Success!  count %d x0 %d, x1%d\n", rc,lg_in_data[0], lg_in_data[1]);
  sleep(1);
  
  close(laser_fd);
  exit(EXIT_SUCCESS);
}

  
