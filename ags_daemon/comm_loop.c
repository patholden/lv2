#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#define _GNU_SOURCE   // Needed for polling conditions
#include <signal.h>
#include <poll.h>
#include <sys/io.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/ip_icmp.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include <syslog.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "Protocol.h"
#include "LaserCmds.h"
#include "SystemMaint.h"
#include "CRCHandler.h"
#include "SystemSpecifics.h"
#include "AppCommon.h"

#define MAXPENDING  5   // Max # connection requests
#define AGS_PORT  1234
#define PACKETSIZE  64
#define POLL_SIZE 4
#define MAXTRIES 100


extern int
pingClient( char* peerAddr );

extern int
DoOnePing( char * hostip, uint8_t * buf, size_t bufsize );

uint16_t
checksum(void *b, int len);

void
LogReceivedData(unsigned char *recvDataPtr);

struct packet
{
    struct icmphdr hdr;
    char msg[PACKETSIZE-sizeof(struct icmphdr)];
};

struct message_length_str
{
  unsigned char cmd;
  unsigned char fill[3];
  int           messageLength;
};

int CommConfigSockfd(struct lg_master *pLgMaster)
{    
  int                sockaddr_len = sizeof(struct sockaddr_in);
  int                error = 0;
  int                on = 1;
  
  // Initialize buffers to 0
  memset(&pLgMaster->webhost_addr, 0, sockaddr_len);

  // Just in case this is a re-config, check for open socketfd & close first
  if ((pLgMaster->socketfd >= 0) || (pLgMaster->datafd >= 0))
    {
      if (pLgMaster->datafd >= 0)
	close(pLgMaster->datafd);
      if (pLgMaster->socketfd >= 0)
	close(pLgMaster->socketfd);
      pLgMaster->socketfd = -1;
      pLgMaster->datafd = -1;
    }

  // Set up comm interface with PC Host
  pLgMaster->socketfd = socket(PF_INET, SOCK_STREAM, IPPROTO_IP);
  if (pLgMaster->socketfd < 0)
    {
      perror("server socket: ");
      return(-1);
    }
  error = setsockopt(pLgMaster->socketfd, SOL_SOCKET, TCP_NODELAY, &on, sizeof(on));
  if (error < 0)
    {
      syslog(LOG_ERR,"COMMCFGSCK: setsockopt failed for socket %d",pLgMaster->socketfd);
      return(-2);
    }
  pLgMaster->webhost_addr.sin_family = AF_INET;
  pLgMaster->webhost_addr.sin_port = htons(AGS_PORT);
  pLgMaster->webhost_addr.sin_addr.s_addr = INADDR_ANY;
  error = bind(pLgMaster->socketfd, (struct sockaddr *)&pLgMaster->webhost_addr, sockaddr_len);
  if (error < 0)
    {
      perror("COMMCFGSCK: Bind failed ");
      return(-2);
    }
  error = listen(pLgMaster->socketfd, MAXPENDING);
  if (error < 0)
    {
      perror("COMMCFGSCK: connect failed ");
      return(-3);
    }

  pLgMaster->enet_retry_count++;
  pLgMaster->serial_ether_flag = 2;
  return(0);
}

int CommInit(struct lg_master *pLgMaster)
{
  int            error = 0;
  
  // Always check integrity of master struct
  if (!pLgMaster)
    {
      perror("Need master data configured!");
      return(-1);
    }

  // Try to open Comms interface to PC host
  if ((error = CommConfigSockfd(pLgMaster)) < 0)
    {
      perror("opensock");
      return(error);
    }
  syslog(LOG_NOTICE, "PC host comm ethernet port initialized");

  // Try to open front end PC serial port
  pLgMaster->pc_serial = open("/dev/lgttyS1", O_RDWR | O_NONBLOCK);
  if (pLgMaster->pc_serial <= 0)
    {
      syslog(LOG_ERR,"open PC front end serial port /dev/lgttyS1 failed");
      return(-5);
    }
  syslog(LOG_NOTICE, "PC host comm serial port lgttyS1 initialized");
  // Try to open AutoFocus serial port
  pLgMaster->af_serial = open("/dev/lgttyS2", O_RDWR | O_NONBLOCK);
  if (pLgMaster->af_serial <= 0)
    {
      syslog(LOG_ERR,"open Auto Focus serial port /dev/lgttyS2 failed");
      return(-5);
    }

  syslog(LOG_NOTICE, "AutoFocus port lgttyS2 initialized");
  // This sets the correct data presentation on send/recv from webhost
  pLgMaster->gHEX = 1;
  return(0);
}
static int ProcEnetPacketsFromHost(struct lg_master *pLgMaster)
{
    // poll for ethernet data,
    // a new ethernet connection
    // and, possibly, a serial connection

    struct message_length_str *messageLengthStrPtr = 0;
    struct pollfd poll_fd[3];
    int           data_len = 0;
    int           error = 0; 
    int           i = 0;
    int           messageLength = 0;
    int           messageLengthStrSize = sizeof(struct message_length_str);
    int           pollTimeout1 = 300;
    int           pollTimeout2 = 3;
    int           pollTimeout3 = 3;
    int           total_count = 0;
    unsigned char doMessageLengthCheck = true;
    unsigned char *last_buff = 0;
    unsigned char *parse_buff = 0;
    unsigned char *recv_data = 0;
    uint32_t      parsed_count = 0;
   
    if (!pLgMaster)
      return(-1);

    if (pLgMaster->serial_ether_flag != 2)
      return(-2);

     // initialize poll buffers
    memset((char *)&poll_fd, 0, 3*sizeof(struct pollfd));

     // initialized polling for ethernet data
    poll_fd[0].fd = pLgMaster->datafd;
    poll_fd[0].events = POLLIN | POLLHUP;

     // initialized polling for new ethernet connection
    poll_fd[1].fd = pLgMaster->socketfd;
    poll_fd[1].events = POLLIN | POLLHUP;

    error = poll((struct pollfd *)&poll_fd, 2, pollTimeout1);  // try old .2 seconds

    if (error < 1)
      return(0);

    // return if no events had occured
    if (poll_fd[0].revents == 0 && poll_fd[1].revents == 0)
      return(0);

     // exit with return value -1
     // if another connection comes in
     // so that the old connection is deleted
     // and a new connection can be set up

    if (poll_fd[1].revents & POLLIN) {
      return(-1);
    }

    last_buff = (unsigned char *)malloc(COMM_RECV_MAX_SIZE);

    if (!last_buff)
      return(-4);

    recv_data = (unsigned char *)malloc(COMM_RECV_MAX_SIZE);
   
    if (!recv_data)
      return(-3);

    // Prep buffers for accepting/receiving new packet
    memset(recv_data, 0, COMM_RECV_MAX_SIZE);
    memset(last_buff, 0, COMM_RECV_MAX_SIZE);

    for (;;)
    {
      // We've got our socket fd for client, now look for data

      parse_buff = (unsigned char *)(recv_data + total_count);

      data_len = recv(pLgMaster->datafd, parse_buff, (COMM_RECV_MAX_SIZE - total_count), 0);

      if (data_len <= 0)
  	break;
  
      total_count+= data_len;

      if ((messageLength > 0) && (total_count >= messageLength))
       	goto L1;
      
      if (!total_count || (total_count >= COMM_RECV_MAX_SIZE))
      {
	syslog(LOG_DEBUG, "Data not ready on receive, total_count %d, errno %d", total_count, errno);
	break;
      }
	
      // do message length check?
      if (doMessageLengthCheck == true)
      {
	// yes, make sure we have all of the message length info
	if (total_count >= messageLengthStrSize)
  	{
	  // set messsage length structure pointer
	  messageLengthStrPtr = (struct message_length_str *)recv_data;

	  // message length passed in?
	  if (messageLengthStrPtr->cmd == kMessageLength)
	  {    
	    // yes, extract it
	    messageLength = messageLengthStrPtr->messageLength;

	    // adjust message length (don't include message length info)
	    messageLength-= messageLengthStrSize;
 	    
 	    // adjust total count (don't include message length info)
	    total_count-= messageLengthStrSize;
     
	    // any data to move?
	    if (total_count > 0)
            {
	      // yes, log received data
	      // LogReceivedData(recv_data);
	      
	      // move data down (NOTE: DON'T USE memcpy, IT WON'T WORK, KNOWN ISSUE)
	      memmove(recv_data, recv_data + messageLengthStrSize, total_count);

 	      // log changed received data
	      // LogReceivedData(recv_data);

	      // adjust pollTimeout2
	      pollTimeout2 = 50;

	      // adjust pollTimeout3
	      pollTimeout3 = 10;
            }
	  }

	  doMessageLengthCheck = false;
	}
      }
 
      // all done if we got all of message
      if ((messageLength > 0) && (total_count >= messageLength))
        goto L1;

      error = poll((struct pollfd *)&poll_fd, 1, pollTimeout2);

      if (error < 1)
      	break;
            
      if (!(poll_fd[0].revents & POLLIN))
      	break;
    }

    // Try ten more times
    for ( i = 0; i < 10 ; i++ )
    {
      error = poll((struct pollfd *)&poll_fd, 1, pollTimeout3);
      
      if (error == 1)
      {
	if ((poll_fd[0].revents & POLLIN))
	{
	  // Try reading in one more buffer of data
	  data_len = recv(pLgMaster->datafd, last_buff, (COMM_RECV_MAX_SIZE - total_count), 0);

	  if (data_len > 0)
	  {
	    parse_buff = (unsigned char *)(recv_data + total_count);

	    memcpy(parse_buff, last_buff, data_len);

	    total_count+= data_len;
	       
	    if ((messageLength > 0) && (total_count >= messageLength))
	      goto L1;
	  }
        }
      }
    }

    // syslog(LOG_DEBUG, "END OF TEN-MORE-TIMES LOOP");
  
    if (total_count == 0)
      {
	free(last_buff);
	free(recv_data);
	return(-1);
      }

L1:

    // syslog(LOG_DEBUG, "AT LABEL L1");
    
    // Process incoming data
    error = parse_data(pLgMaster, recv_data, total_count, &parsed_count);

    if (error < 0)
      {
	syslog(LOG_ERR, "Bad parse, error %d",error);
	free(recv_data);
	free(last_buff);
	return(error);
      }

    free(last_buff);
    free(recv_data);

    return(0);
}
static int IsOkToSend(struct lg_master *pLgMaster)
{
    struct pollfd  poll_fd;

    memset((char *)&poll_fd, 0, sizeof(struct pollfd));
    poll_fd.fd = pLgMaster->datafd;
    poll_fd.events = POLLOUT;

    if (poll_fd.revents | POLLOUT)
      return(0);
    return(-1);
}
void SendConfirmation(struct lg_master *pLgMaster, unsigned char theCommand)
{
    unsigned char    gOutputBuffer[sizeof(struct send_cnfm)];
    unsigned char    gOutputHex[(sizeof(struct send_cnfm) * 2)];
    int              i, count, sent;
    struct           send_cnfm *pOut=(struct send_cnfm *)&gOutputBuffer[0];;

    if (IsOkToSend(pLgMaster))
      return;
  
    // Need to initialize buffers to be used
    memset(gOutputBuffer, 0, sizeof(struct send_cnfm));
    memset(gOutputHex, 0, (sizeof(struct send_cnfm) * 2));

    pOut->cmd   = theCommand;
    pOut->flags = pLgMaster->gHeaderSpecialByte;
    pOut->seq_num = htons(pLgMaster->seqNo);
    AppendCRC((unsigned char *)gOutputBuffer, (sizeof(struct send_cnfm)-kCRCSize));

    if (pLgMaster->gHEX == 1 ) {
      count = 0; 
      for (i=0; i<COMM_CONFIRM_LEN; i++)
	{
	  if (gOutputBuffer[i] >= 0x80)
	    {
	      gOutputHex[count] = 0x80;
	      count++;
	      gOutputHex[count] = gOutputBuffer[i] - 0x80;
	      count++;
	    }
	  else
	    {
	      gOutputHex[count] = gOutputBuffer[i];
	      count++;
	    }
	}
      sent = send(pLgMaster->datafd, gOutputHex, count, MSG_NOSIGNAL);
      if (sent <=0)
	syslog(LOG_ERR,"bad send on confirmation: errno %d", errno);
    }

    if (pLgMaster->gHEX == 0)
      {
	if ((pLgMaster->serial_ether_flag == 2) && (pLgMaster->datafd >= 0)) 
	  sent = send(pLgMaster->datafd, gOutputBuffer, COMM_CONFIRM_LEN, MSG_NOSIGNAL);
	if (sent <=0)
	  syslog(LOG_ERR,"Bad send on confirmation: errno %d", errno);
      }
    return;
}

void SendA3(struct lg_master *pLgMaster)
{
    struct k_header gOutputBuffer;
    int sent=0;

    if (IsOkToSend(pLgMaster))
      return;
  
    memset ((char *)&gOutputBuffer, 0, sizeof(struct k_header));
    gOutputBuffer.status = 0xA3;

    if ((pLgMaster->serial_ether_flag == 2)  && (pLgMaster->datafd >= 0))
      sent = send(pLgMaster->datafd, (char *)&gOutputBuffer, sizeof(struct k_header), MSG_NOSIGNAL );
      syslog(LOG_ERR,"Sending A3: errno %d", errno);
    if (sent <= 0)
      syslog(LOG_ERR,"Bad send on A3: errno %d", errno);
    return;
}


void HandleResponse (struct lg_master *pLgMaster, int32_t lengthOfResponse, uint32_t gRespondToWhom )
{
    unsigned char  *gOutputHEX;
    int            i, count, remain, sent;

#ifdef COMMDEBUG
syslog(LOG_DEBUG, "entering HandleResponse %p  len %d  2whom %d", pLgMaster, lengthOfResponse, gRespondToWhom );
#endif

    if (IsOkToSend(pLgMaster))
      return;
  
    // FIXME--PAH--Here for compilation but should it be removed?
    // gRespondToWhom isn't used atm.
    gRespondToWhom = gRespondToWhom;
    gOutputHEX = malloc((lengthOfResponse * 2)+4);   // message length plus CRC in HEX
#ifdef COMMDEBUG
syslog(LOG_DEBUG, "HandleR gOutputHex %p", gOutputHEX );
#endif
    if (!gOutputHEX)
      {
	perror("\nHANDLERESP: Bad Malloc():");
	return;
      }
    memset(gOutputHEX, 0, (lengthOfResponse * 2)+4);   // message length plus CRC in HEX
#ifdef COMMDEBUG
syslog(LOG_DEBUG, "HandleR gOutputHex %p about to append", gOutputHEX );
#endif
  
    AppendCRC(pLgMaster->theResponseBuffer, lengthOfResponse);
#ifdef COMMDEBUG
syslog(LOG_DEBUG, "HandleR gHEX %x", pLgMaster->gHEX );
#endif
    if (pLgMaster->gHEX == 1)
      {
	count = 0; 
	for (i=0; i<(lengthOfResponse+kCRCSize); i++)
	  {
	    if (pLgMaster->theResponseBuffer[i] >= 0x80)
	      {
		gOutputHEX[count] = 0x80;
		count++;
		gOutputHEX[count] = pLgMaster->theResponseBuffer[i] - 0x80;
		count++;
	      }
	    else
	      {
		gOutputHEX[count] = pLgMaster->theResponseBuffer[i];
		count++;
	      }
	  }
	remain = count;
#ifdef COMMDEBUG
syslog(LOG_DEBUG, "HandleR remain %d gOutputHEX %p", remain, gOutputHEX );
#endif
	if (pLgMaster->serial_ether_flag == 2)
	  {
	    while (remain > 0)
	      {
		sent = send( pLgMaster->datafd, gOutputHEX, remain, MSG_NOSIGNAL );
		if (sent <= 0)
		  {
		    syslog(LOG_ERR,"Bad send on Response: errno %d", errno);
		    if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
		      break;
		  }
		remain -= sent;
	      }
	  }
#ifdef COMMDEBUG
syslog(LOG_DEBUG, "HandleR remain %d gOutputHEX %p", remain, gOutputHEX );
#endif
      }

    else if (pLgMaster->gHEX == 0)
      {
	remain = lengthOfResponse+2;
	if (pLgMaster->serial_ether_flag == 2)
	  {
	    while(remain > 0)
	      {
		sent = send(pLgMaster->datafd, pLgMaster->theResponseBuffer, remain, MSG_NOSIGNAL);
		if (sent <= 0)
		  {
		    syslog(LOG_ERR,"Bad send on Response: errno %d", errno);
		    if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
		      break;
		  }
		remain -= sent;
	      }
	  }
      }
#ifdef COMMDEBUG
syslog(LOG_DEBUG, "HandleResponse about to free %p", gOutputHEX );
#endif
    free(gOutputHEX);
#ifdef COMMDEBUG
syslog(LOG_DEBUG, "leaving HandleResponse" );
#endif
    return;
}

int
pingClient(char* peerAddr )
{
  uint8_t buffer[256];
  // int return_code;
  uint32_t inaddr;
  int  bytes;

  // Make sure our PC webhost has been configured!
  if ((inaddr = inet_addr(peerAddr)) == INADDR_NONE)
    {
      perror("\nBad WebHost IP: ");
      // ***debug*** return(-1);
    }
   
   bytes = DoOnePing( peerAddr, buffer, sizeof(buffer) ); 
   if (bytes <= 0) {
     syslog(LOG_ERR, "Ping to %s FAILED return %d", peerAddr, bytes );
     return(-1);
   }
   return(0);
}


int DoProcEnetPackets(struct lg_master *pLgMaster)
{
    struct sockaddr_in client_addr;
    struct sockaddr_in haddr;
    socklen_t          sockaddr_len = sizeof(struct sockaddr_in);
    int                error=0;
    int                on=1;

    // Initialize buffers
    memset(&client_addr, 0, sockaddr_len);
    memset(&haddr, 0, sockaddr_len);

    // Accept connection with host
    if (pLgMaster->datafd >= 0)
      close(pLgMaster->datafd);
    pLgMaster->datafd = accept(pLgMaster->socketfd, (struct sockaddr *)&client_addr, (socklen_t *)&sockaddr_len);
    if (pLgMaster->datafd < 0)
      {
	syslog(LOG_ERR,"COMMLOOP:  unable to accept data from %s", pLgMaster->webhost);
	return(-1);
      }
    error = setsockopt(pLgMaster->datafd, SOL_SOCKET, TCP_NODELAY, &on, sizeof(on));
    if (error < 0)
      {
	syslog(LOG_ERR,"COMMCFGSCK: setsockopt failed for socket %d",pLgMaster->socketfd);
	return(-2);
      }
    // Get host's IP address
    error = getpeername(pLgMaster->datafd, (struct sockaddr *)&haddr, &sockaddr_len);
    if (error < 0)
      {
	syslog(LOG_ERR,"COMMCFGSCK: getpeername failed");
	return(-2);
      }
    strcpy(pLgMaster->webhost, inet_ntoa(haddr.sin_addr));

    syslog(LOG_NOTICE, "receiving data from %s, recvfd %x", pLgMaster->webhost, pLgMaster->datafd);
    doSetLinkLED(pLgMaster);
    while (pLgMaster->datafd >= 0)
      {
	error = ProcEnetPacketsFromHost(pLgMaster);
	if (error < 0)
	  return(error);
	else
	  {
	    // Honor ping-heartbeat so partner knows we're alive
	    if ((pingClient(pLgMaster->webhost)) == 0)
	      {
		pLgMaster->ping_count = 0;
	      }
	    else
             {
	        pLgMaster->ping_count++;
	     }
             // wait for three failed pings
             if ( pLgMaster->ping_count >= 3 ) 
		return(-1);
	    
	  }
	DoSystemPeriodic(pLgMaster);
      }
    // Should never get here
    return(0);
}


int
DoOnePing( char * hostip, uint8_t * buf, size_t bufsize )
{
/*
 *  DoOnePing just sends one ICMP ping packet
 *  and tried to receive any return message
 */

        const int            val  =   2;
        int                  i, sd;
        struct packet        pckt;
        socklen_t            len;
        struct sockaddr_in   addr;
        struct protoent      *protocol;
        int                  bytes = 0;
        in_addr_t            inaddr;

        int try;
        struct pollfd poll_set[POLL_SIZE];
        int numfds = 0;
        int status;

        inaddr = inet_addr( hostip );

        len   =  sizeof(struct sockaddr_in);

        addr.sin_family        =  AF_INET;
        addr.sin_port          =  0;

        memcpy( (void *)&(addr.sin_addr.s_addr)
              , (void *)&inaddr
              , sizeof(inaddr));


        protocol = getprotobyname("ICMP");

        // open a socket for an ICMP ping packet
        sd    =  socket(PF_INET, SOCK_RAW, protocol->p_proto);
        if (sd < 0)
        {
                syslog(LOG_DEBUG, "DoOnePing socket error");
                return(-1);
        }

        // set Time-To-Live
        if (setsockopt(sd, SOL_IP, IP_TTL, &val, sizeof(val)) != 0)
        {
                syslog(LOG_DEBUG, "DoOnePing Set TTL option error");
                close( sd );
                return(-1);
        }

        // set socket status flag
        if (fcntl(sd, F_SETFL, O_NONBLOCK) != 0)
        {
                syslog(LOG_DEBUG, "DoOnePing Request nonblocking I/O error");
                close( sd );
                return(-1);
        }

        // assemble ICMP ping packet
        //
        memset( (void *)(&pckt), 0, sizeof(pckt) );
        pckt.hdr.type               =  ICMP_ECHO;
        pckt.hdr.un.echo.id         =  321;  // should be set to something better
        for (i = 0; i < sizeof(pckt.msg)-1; i++)
                        pckt.msg[i] = i+'0';
        pckt.msg[i]                 =  0;
        pckt.hdr.un.echo.sequence   =  1;
        pckt.hdr.checksum           =  checksum(&pckt, sizeof(pckt));

        // send off ICMP ping packet
        //
        if (sendto(sd, &pckt, sizeof(pckt), 0, (struct sockaddr*)&addr, sizeof(addr)) <= 0) {
                syslog(LOG_DEBUG, "DoOnePing sending ping failed");
                close( sd );
                return(-1);
        }


        len  =  sizeof(struct sockaddr_in);

        memset( buf, 0, bufsize );


        // set up a poll for response
        numfds = 0;
        poll_set[0].fd = sd;
        poll_set[0].events = POLLIN;
        numfds++;
         
        try = 0;
        bytes = 0;
        
        while ( try < MAXTRIES && bytes == 0 ) {
            try++;
            status = poll( poll_set, numfds, 2 );
            if ( (poll_set[0].revents & POLLIN) && status >= 1 ) {
               usleep(1000);
               bytes = recvfrom(sd, buf, bufsize, 0, (struct sockaddr*)&addr, &len);
            }
       }

       // syslog(LOG_DEBUG, "DoOnePing bytes %d  try %d\n", bytes, try );

       close( sd );

       return ( bytes );
}


uint16_t checksum(void *b, int len)
{
        uint16_t   *buf = b;
        uint32_t     sum=0;

        for(sum=0; len>1; len-=2)
        {
                sum += *buf++;
        }
        if (len == 1)
        {
                sum += *(uint8_t*)buf;
        }
        sum     =  (sum >> 16) + (sum & 0xFFFF);
        sum     += (sum >> 16);
        return(~sum);
}

void LogReceivedData(unsigned char *ptr)
{

  int  ii;

  syslog(LOG_DEBUG, " ");
	    
  ii = 0; 
  syslog( LOG_DEBUG ,
       "recv_data %5d " " %02x %02x %02x %02x" " %02x %02x %02x %02x" " %02x %02x %02x %02x" " %02x %02x %02x %02x"
     , ii
     , ptr[ii +  0] , ptr[ii +  1] , ptr[ii +  2] , ptr[ii +  3]
     , ptr[ii +  4] , ptr[ii +  5] , ptr[ii +  6] , ptr[ii +  7]
     , ptr[ii +  8] , ptr[ii +  9] , ptr[ii + 10] , ptr[ii + 11]
     , ptr[ii + 12] , ptr[ii + 13] , ptr[ii + 14] , ptr[ii + 15]
     );
     
  ii = 16; 
  syslog( LOG_DEBUG ,
       "recv_data %5d " " %02x %02x %02x %02x" " %02x %02x %02x %02x" " %02x %02x %02x %02x" " %02x %02x %02x %02x"
     , ii
     , ptr[ii +  0] , ptr[ii +  1] , ptr[ii +  2] , ptr[ii +  3]
     , ptr[ii +  4] , ptr[ii +  5] , ptr[ii +  6] , ptr[ii +  7]
     , ptr[ii +  8] , ptr[ii +  9] , ptr[ii + 10] , ptr[ii + 11]
     , ptr[ii + 12] , ptr[ii + 13] , ptr[ii + 14] , ptr[ii + 15]
     );

  ii = 32; 
  syslog( LOG_DEBUG ,
       "recv_data %5d " " %02x %02x %02x %02x" " %02x %02x %02x %02x" " %02x %02x %02x %02x" " %02x %02x %02x %02x"
     , ii
     , ptr[ii +  0] , ptr[ii +  1] , ptr[ii +  2] , ptr[ii +  3]
     , ptr[ii +  4] , ptr[ii +  5] , ptr[ii +  6] , ptr[ii +  7]
     , ptr[ii +  8] , ptr[ii +  9] , ptr[ii + 10] , ptr[ii + 11]
     , ptr[ii + 12] , ptr[ii + 13] , ptr[ii + 14] , ptr[ii + 15]
     );

  ii = 48; 
  syslog( LOG_DEBUG ,
       "recv_data %5d " " %02x %02x %02x %02x" " %02x %02x %02x %02x" " %02x %02x %02x %02x" " %02x %02x %02x %02x"
     , ii
     , ptr[ii +  0] , ptr[ii +  1] , ptr[ii +  2] , ptr[ii +  3]
     , ptr[ii +  4] , ptr[ii +  5] , ptr[ii +  6] , ptr[ii +  7]
     , ptr[ii +  8] , ptr[ii +  9] , ptr[ii + 10] , ptr[ii + 11]
     , ptr[ii + 12] , ptr[ii + 13] , ptr[ii + 14] , ptr[ii + 15]
     );

  ii = 64; 
  syslog( LOG_DEBUG ,
       "recv_data %5d " " %02x %02x %02x %02x" " %02x %02x %02x %02x" " %02x %02x %02x %02x" " %02x %02x %02x %02x"
     , ii
     , ptr[ii +  0] , ptr[ii +  1] , ptr[ii +  2] , ptr[ii +  3]
     , ptr[ii +  4] , ptr[ii +  5] , ptr[ii +  6] , ptr[ii +  7]
     , ptr[ii +  8] , ptr[ii +  9] , ptr[ii + 10] , ptr[ii + 11]
     , ptr[ii + 12] , ptr[ii + 13] , ptr[ii + 14] , ptr[ii + 15]
     );
}
