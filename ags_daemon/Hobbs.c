#include <stdint.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>

#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "Protocol.h"
#include "Hobbs.h"
#include "CRCHandler.h"

time_t start_display;
time_t end_display;

static int ReadHobbsFromFile(FILE *filenum, struct lg_master *pLgMaster);
static int ReadHobbsFromFile(FILE *filenum, struct lg_master *pLgMaster);

static int WriteHobbsToFile(FILE *filenum, struct lg_master *pLgMaster)
{
  char   hobbsBuffer[HOBBS_MAX_BUFFLEN];
  size_t num_write;
  size_t bytes_to_write;

  memset(hobbsBuffer, 0, sizeof(hobbsBuffer));
  if (!filenum ||  !pLgMaster)
    return(-1);

  // Get Hobbs counters and put into buffer
  sprintf(hobbsBuffer, "%10ld\r\n%10ld\r\n%10ld\r\n%10ld\r\n",
	  pLgMaster->hobbs.hobbs_time,
	  pLgMaster->hobbs.xscanner_time,
	  pLgMaster->hobbs.yscanner_time,
	  pLgMaster->hobbs.laser_time);
  bytes_to_write = strlen(hobbsBuffer);
  if (bytes_to_write <= 0)
    return(-2);
  
  // Write counters out to file
  num_write = fwrite(hobbsBuffer, bytes_to_write, 1, filenum);
  if (num_write <= 0)
    return(-3);
  return(0);
}

int HobbsCountersInit(struct lg_master *pLgMaster)
{
  FILE  *filefd;
  long   file_size;
  int    rc=0;

  filefd = fopen("/laservision/data/hobbs", "w+");
  if (!filefd)
    return(-1);

  fseek(filefd, 0, SEEK_SET);
  file_size = ftell(filefd);
  // If zero-length then new file
  if (!file_size)
    {
      rc = WriteHobbsToFile(filefd, pLgMaster);
      if (rc)
	syslog(LOG_ERR,"Unable to write Hobbs counters to file");
    }
  else
    {
      // Read in data from file
      rc = ReadHobbsFromFile(filefd, pLgMaster);
      if (rc)
	syslog(LOG_ERR,"Unable to retrieve Hobbs counters from file");
    }
  fclose(filefd);
  return(rc);
}

void StartHobbs(struct lg_master *pLgMaster)
{
     end_display = 0;
     start_display = time(NULL);    
}


// FIXME---PAH---NEED TO PUSH LASER, X & Y SCANNER TIMES TO DRIVER & USE IOCTL
// TO GET THEM.
void EndHobbs(struct lg_master *pLgMaster)
{
  time_t delta_time;

  end_display = time(NULL);
  if ( start_display > 0 ) {
    delta_time = end_display - start_display;
    // only count display intervals int32_ter that one second
    if ( delta_time > 1 ) {
      pLgMaster->hobbs.hobbs_time += delta_time;
      pLgMaster->hobbs.xscanner_time += delta_time;
      pLgMaster->hobbs.yscanner_time += delta_time;
      pLgMaster->hobbs.laser_time += delta_time;
    }
  }

  // FIXME---PAH---WRITE NEEDS TO MOVE TO BACKGROUND TASK
  WriteHobbs(pLgMaster);  // write hobbs times to hobbs file
  start_display = 0;
  return;
}

  // just work from RAM copy
static int ReadHobbsFromFile(FILE *filenum, struct lg_master *pLgMaster)
{ 
  char   *pBuff;
  size_t length;
  size_t read_count;
  time_t projector_time;
  time_t xscanner_time;
  time_t yscanner_time;
  time_t laser_time;
  
  // Figure out length of file
  fseek(filenum, 0L, SEEK_SET);
  fseek(filenum, 0L, SEEK_END);
  length = ftell(filenum);
  fseek(filenum, 0L, SEEK_SET);
  if (!length)
    return(-2);
  // grab buffer to work with
  pBuff = malloc(length);
  if (!pBuff)
    return(-3);
  
  // Read in data
  read_count = fread(pBuff, length, 1, filenum);
  fclose(filenum);
  if (read_count <=0)
    return(-4);
  
  // Get counters from file & populate lg struct
  sscanf(pBuff,
	 "%[^0123456789]s\r\n%[^0123456789]s\r\n%[^0123456789]s\r\n%[^0123456789]s\r\n", 
	 (char *)&projector_time,
	 (char *)&xscanner_time,
	 (char *)&yscanner_time,
	 (char *)&laser_time);

  // Update struct from file last recorded times
  pLgMaster->hobbs.hobbs_time += projector_time;
  pLgMaster->hobbs.xscanner_time += xscanner_time;
  pLgMaster->hobbs.yscanner_time += yscanner_time;
  pLgMaster->hobbs.laser_time += laser_time;
  return(0);
}

void DoHobbsSet(struct lg_master *pLgMaster, struct parse_hobbsset_parms *pInp, uint32_t respondToWhom)
{
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;

  pResp->hdr.status = RESPGOOD;
  switch(pInp->inp_hobbsid)
    {
    case PARSE_HOBBS_HOBBS:
      pLgMaster->hobbs.hobbs_time = pInp->inp_hobbsval;
      break;
    case PARSE_HOBBS_XSCAN:
      pLgMaster->hobbs.xscanner_time = pInp->inp_hobbsval;
      break;
    case PARSE_HOBBS_YSCAN:
      pLgMaster->hobbs.yscanner_time = pInp->inp_hobbsval;
      break;
    case PARSE_HOBBS_LASER:
      pLgMaster->hobbs.laser_time = pInp->inp_hobbsval;
      break;
    default:
      pResp->hdr.status = RESPFAIL;
      break;
    }
  HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), respondToWhom);
  return;
}

void DoHobbsGet( struct lg_master *pLgMaster, struct parse_hobbsget_parms *pInp, uint32_t respondToWhom )
{
  struct parse_hobbsget_resp *pResp;
  uint32_t return_size;

  // Set up response buffer by clearing it
  pResp = (struct parse_hobbsget_resp *)pLgMaster->theResponseBuffer;
  memset((char *)pResp, 0, sizeof(struct parse_hobbsget_resp *));

  // Assume we're going to have a good response
  return_size = sizeof(struct parse_hobbsget_resp)-kCRCSize;
  pResp->hdr.status = RESPGOOD;

  // Set appropriate value to master struct val.  it will be periodically
  // backed up to file via background task
  switch(pInp->inp_hobbsid)
    {
    case PARSE_HOBBS_HOBBS:
      pResp->resp_hobbscount = pLgMaster->hobbs.hobbs_time;
      break;
    case PARSE_HOBBS_XSCAN:
      pResp->resp_hobbscount = pLgMaster->hobbs.xscanner_time;
      break;
    case PARSE_HOBBS_YSCAN:
      pResp->resp_hobbscount = pLgMaster->hobbs.yscanner_time;
      break;
    case PARSE_HOBBS_LASER:
      pResp->resp_hobbscount = pLgMaster->hobbs.laser_time;
      break;
    default:
      pResp->hdr.status = RESPFAIL;
      break;
    }

  if (pResp->hdr.status == RESPFAIL)
    return_size = sizeof(struct parse_basic_resp) - kCRCSize;

  HandleResponse(pLgMaster, return_size, respondToWhom);
  return;
}

int WriteHobbs(struct lg_master *pLgMaster)
{
  FILE  *filefd;
  int    rc;
  
  filefd = fopen("/laservision/data/hobbs", "w+");
  if (!filefd)
    return(-1);
  rc = WriteHobbsToFile(filefd, pLgMaster);
  fclose(filefd);
  return(rc);
}
