#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <syslog.h>
#include <sys/io.h>
#include <string.h>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "DoAutoFocusCmd.h"
#include "Protocol.h"
#include "RemoteSerial.h"

void DoAutoFocusCmd(struct lg_master *pLgMaster, unsigned char *buffer)
{
     int AFtoWrite;
     int af_index;
     int write_count=0;
     int read_count=0;
     int num_read=0;
     int countDown = 0;
     char cdata_in=0;
     char cdata_out=0;
     
     struct parse_autofocus_parms *pInp = (struct parse_autofocus_parms *)buffer;
     struct parse_autofocus_resp *pResp = (struct parse_autofocus_resp *)pLgMaster->theResponseBuffer;

     // Init response buffer
     memset(pResp, 0, sizeof(struct parse_autofocus_resp));

     // Get length of input buffer & validate length
     AFtoWrite = strlen((const char *)pInp->inp_data );
     if ((AFtoWrite <= 0) || (AFtoWrite > 32) || (pLgMaster->af_serial < 0))
       {
	 syslog(LOG_ERR, " Bad AFWriteLen %d", AFtoWrite);
	 pResp->hdr.status = RESPFAIL;
	 HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp) - kCRCSize), kRespondExtern);
	 return;
       }

     // for LASER mode
     // check if remote computer is to be used
     if ( (pLgMaster->gHeaderSpecialByte & 0x01) && (pLgMaster->projector_mode == PROJ_LASER) ) {

           RemoteSerial( pLgMaster
                       , (char *)buffer
                       , kRespondExtern
                       );

           return;
     }

     // Write buffer out to ttyS2
     // Need to read one byte at a time due to FPGA limitations for serial port design
     // A single operation at 115200 takes 69 usec, need to honor that time with usleep
     if ( pLgMaster->af_serial > 0 ) {
       for (af_index = 0; af_index < AFtoWrite; af_index++)
       {
	 cdata_out = pInp->inp_data[af_index];
	 write_count = write(pLgMaster->af_serial, (void *)&cdata_out, 1);
	 if (write_count != 1)
	   {
	     syslog(LOG_ERR, "AFWRITE: WRITE fail error/count %x, errno %d, err-count %x",
		    write_count, errno, write_count);
	     pResp->hdr.status = RESPFAIL;
	     HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp) - kCRCSize), kRespondExtern);
	     return;
	   }
	 usleep(100);    // Let each write operation complete before trying next character.
       }
     }
     
     // response expected?
     if ( pLgMaster->gHeaderSpecialByte & 0x02 )
       {
	 // initialize
	 countDown = MAX_DATA;
	 
	 // Wait 1msec to get data back
	 usleep(250000);

	 // Need to read one byte at a time due to FPGA limitations for serial port design
	 for (af_index = 0; af_index < MAX_DATA; af_index++)
	   {
	     read_count = read(pLgMaster->af_serial, &cdata_in, 1);

	     if (read_count == 1)
	       {
		 pLgMaster->gAFInputBuffer[af_index] = cdata_in;
		 num_read++;
		 // carriage return is end of message
		 if (cdata_in == 0x0D)
		   break;
		 countDown = MAX_DATA;
		 usleep(100);    // Let each read operation complete before trying next character.
	       }
	     else
	       {
		 if ((cdata_in == -1) && (errno == EOF))
		   break;

		 if (read_count < 0)
		   {
		     if ( errno != EAGAIN && errno != EWOULDBLOCK )
		       {
			 syslog(LOG_ERR,"AFREAD: FAIL: total_bytes %x, rc %x, read val %2x,errno %d",
				read_count,num_read, (int)cdata_in,errno);
			 pResp->hdr.status = RESPFAIL;
			 HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp) - kCRCSize), kRespondExtern);
			 return;
		       }
		     else
		       {
			 // didn't read any characters, see if we exhausted our reads
			 if (--countDown == 0)
			   break;
			 
			 // reset index
			 af_index--;

			 // sleep a bit
			 usleep(1000);
		       }
		   }
	       }
	   }

	 if ((num_read > 0) && (num_read <= AUTOFOCUS_SIZE))
	   {
	     memcpy(pResp->resp_data, pLgMaster->gAFInputBuffer, num_read);
	     pResp->hdr.status = kAutoFocusCmd;
	     HandleResponse(pLgMaster, (sizeof(struct parse_autofocus_resp) - kCRCSize), kRespondExtern);
	   }
       }

     return;
}

