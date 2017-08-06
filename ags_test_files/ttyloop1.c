/*************************************************************
 *                                                           *
 * Assembly Guidance Systems X86_64 test                     *
 * Copyright 2015.                                           *
 *                                                           *
 * Test Name:  ttyloop1                                      *
 * Command  :  "ttyloop1"                                    *
 * Input    :    NONE.                                       *
 * Description:  This test exercises the kernel laserguide   *
 *               lgttyS1 device                              *
 *                                                           *
 *               driver.  It writes and reads back a string  *
 *               to the device once a second.   Status is    *
 *               returned to user on success or failure.     *
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
#include <termios.h>
#include <sys/io.h>
#include <linux/laser_api.h>
#include <string.h>

#define TEST_BUFF_LEN   128
char   *test_buff_out;

int main( int argc, char ** argv )
{
  size_t  test_len=0;
  fd_set  rdset, wrset;
  int     fd_serial;
  int     n, baud;
  int     count=0, index;
  char    cdata_in=0,cdata_out=0;
  int 	  pos=0;
  char    cursor[4]={'/','-','\\','|'};

  
  fd_serial = open("/dev/lgttyS1", O_RDWR | O_NONBLOCK);
  if (fd_serial <= 0)
    {
      perror( "Unable to open /dev/lgttyS1" );
      exit(EXIT_FAILURE);
    }

  // Initialize input & output test buffers
  test_buff_out = malloc(TEST_BUFF_LEN);
  if (!test_buff_out)
    {
      perror("malloc failed for outbuff");
      close(fd_serial);
      exit(EXIT_FAILURE);
    }
  memset(test_buff_out, 0, TEST_BUFF_LEN);
  sprintf(test_buff_out, "Hello from lgttyS1\n");
  test_len = strlen(test_buff_out);

  // Start loopback test
  while (1)
    {
       count++;
       n = 0;
       fprintf(stderr, "\nLGTTYS1 (PC I/O Port) Loop: %d ", count);
       for (index=0; index < test_len; index++)
	 {
	   cdata_out = test_buff_out[index];
	   n = write(fd_serial, &cdata_out, 1);
	   if ((n != 1) && (errno == EINTR))
	     {
	       perror("WRITEfail");
	       close(fd_serial);
	       exit(EXIT_FAILURE);
	     }
	   usleep(1000);   // 1 msec wait for data 
	   n = read(fd_serial, &cdata_in, 1);
	   if ( n < 0 ) {
	     if ( errno != EAGAIN && errno != EWOULDBLOCK )
	       {
		 perror("READfail");
		 close(fd_serial);
		 exit(EXIT_FAILURE);
	       }
	   }
	   if (cdata_in != cdata_out)
	     {
	       fprintf(stderr, "\n byte IO mismatch out=%x, in=%x, byte# %d",cdata_out, cdata_in, index);
	       close(fd_serial);
	       exit(EXIT_FAILURE);
	     }
	   // Advance buffer index
	   if ((cdata_out != 0) && (cdata_in != 0))
	   {
              fprintf(stderr,"%c\b", cursor[pos]);
              pos = (pos+1) % 4;
	   }
	}
    }
  free(test_buff_out);
  exit(EXIT_SUCCESS);
}
