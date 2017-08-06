/*
 *  static char rcsid[]="$Id";
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <syslog.h>
#include <time.h>
#include <linux/laser_api.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "Video.h"
#include "Net.h"

struct sockaddr_in     tcp_srv_addr;
struct servent         tcp_serv_info;
struct hostent         tcp_host_info;

#define MAXLINE 2048
#define BUFFSIZE  (2048+3*512*512)

static int net_fd = 0;

int OpenNet (struct lg_master *pLgMaster)
{
  uint32_t       inaddr;
  int             port = 80;
  
     /*
      *  just in case, close any open net file descriptors
      */
  if ( net_fd ) {
    close (net_fd);
    net_fd = 0;
  }

  bzero((char *) &tcp_srv_addr, sizeof(tcp_srv_addr));
  tcp_srv_addr.sin_family = AF_INET; 

  tcp_srv_addr.sin_port = htons(port);

  if( (inaddr = inet_addr(pLgMaster->webhost)) != INADDR_NONE) {
      bcopy((char *) &inaddr, (char *) &tcp_srv_addr.sin_addr, sizeof(inaddr));
      tcp_host_info.h_name = NULL;
  }
  if ((net_fd = socket(AF_INET, SOCK_STREAM, 0)) <= 0)
    {
      syslog(LOG_ERR, "can't create TCP socket\n");
      close (net_fd);
      return(-1);
    }
  if (connect(net_fd,(struct sockaddr *)&tcp_srv_addr,sizeof(tcp_srv_addr)) < 0)
    {
      syslog(LOG_ERR, "can't connect to server\n");
      close (net_fd);
      return(-1);
    }
  return(0);
}

int GetNetVideo(char *inbuff, int incount, char *outbuff)
{
  char            linebuff[MAXLINE];
  int             n, count;

  write( net_fd, inbuff, incount );
  count = 0;
  n = read(net_fd,linebuff,MAXLINE);
  if ( n > 0 ) {
    strncpy( &(outbuff[count]), linebuff, n );
    count = n;
  }
  outbuff[count] = 0;

  return(count);
}
