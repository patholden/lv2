/*
 * static char rcsid[]="$Id: Web.c,v 1.1 1997/06/06 18:09:22 ags-sw Exp $";
 */
#include <stdint.h>
#include <syslog.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "Video.h"
#include "Web.h"

struct sockaddr_in     tcp_srv_addr;
struct servent         tcp_serv_info;
struct hostent         tcp_host_info;

#define MAXLINE 1024
#define BUFFSIZE  (2048+3*512*512)


int GetWebVideo(struct lg_master *pLgMaster, char *inbuff, int incount, char *outbuff)
{
  int             fd;
  uint32_t   inaddr;
  int             port = 80;
  int             n, count;
  char            linebuff[MAXLINE];
  
  bzero((char *) &tcp_srv_addr, sizeof(tcp_srv_addr));
  tcp_srv_addr.sin_family = AF_INET; 

  tcp_srv_addr.sin_port = htons(port);

  if( (inaddr = inet_addr(pLgMaster->webhost)) != INADDR_NONE) {
      bcopy((char *) &inaddr, (char *) &tcp_srv_addr.sin_addr, sizeof(inaddr));
      tcp_host_info.h_name = NULL;
  }
  if( (fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      syslog(LOG_ERR, "can't create TCP socket");
      close (fd);
      return(-1);
  }
   
  if( connect(fd,(struct sockaddr *)&tcp_srv_addr,sizeof(tcp_srv_addr)) < 0 ) {
      syslog(LOG_ERR, "can't connect to server");
      close (fd);
      return(-1);
  }

  write( fd, inbuff, incount );
  count = 0;
  while( ((n = read(fd,linebuff,MAXLINE)) > 0) && (count<(BUFFSIZE-MAXLINE)) ){
    strncpy( &(outbuff[count]), linebuff, n );
    count += n;
  }

  close (fd);

  outbuff[count] = 0;

  return(count);
}
