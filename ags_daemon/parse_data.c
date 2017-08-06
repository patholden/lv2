#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "Protocol.h"
#include "CRCHandler.h"
#include "LaserCmds.h"
#include "DoAutoFocusCmd.h"
#include "QuickCheckManager.h"
#include "DoTakePicture.h"
#include "ChangeDisplayPeriod.h"
#include "ChangeTransformTolerance.h"
#include "RightOnCert.h"
#include "FracCert.h"
#include "ShowTargets.h"
#include "FlexCalWithFeedback.h"
#include "FlexCalculateTransform.h"
#include "CalculateTransform.h"
#include "FlexFullRegWithFeedback.h"
#include "FullRegWithFeedback.h"
#include "RightOnFullReg.h"
#include "Init.h"
#include "ChangeTransformTolerance.h"
#include "ChangeDisplayPeriod.h"
#include "RefreshRate.h"
#include "GetTargetsUsed.h"
#include "FOM.h"
#include "Hobbs.h"
#include "LaserFlex.h"
#include "CalibXY.h"
#include "SetBit.h"
#include "DoFindOneTarget.h"
#include "Files.h"
#include "L2VtakePicture.h"

uint32_t cmdState = 1;
static char     *gDisplayDataBuffer = 0;
static int32_t  nTargets;

int parse_data(struct lg_master *pLgMaster, unsigned char *data, uint32_t data_len, uint32_t *rawindex)
{
  struct parse_basic_resp *pResp=(struct parse_basic_resp *)pLgMaster->theResponseBuffer;
  unsigned char *pInp;
  uint32_t       index;
  uint32_t       i,cmdSize;
  uint32_t       dataLength=0;
  uint32_t       dataOffset;
  uint32_t       number;
#ifdef SPECIAL
  struct timeval  tv;
  struct timezone tz;
#endif
#ifdef AGS_DEBUG
  uint32_t       *pCrc;
#endif
#ifdef ZDEBUG
  unsigned char * ptr;
  int ii;
#endif
  
  //  FIXME--PAH  int magic, magic_too, flag;

  if (!pLgMaster || !pLgMaster->gInputBuffer || !pLgMaster->gRawBuffer || !pLgMaster->theResponseBuffer || !data || (data_len > kMaxDataLength))
    {
      syslog(LOG_ERR, "PARSEDATA:  Received Bad input");
      return(-1);
    }
  pLgMaster->gParametersBuffer = &pLgMaster->gInputBuffer[4];  // Pointer to input data less k_header
  pInp = pLgMaster->gParametersBuffer;

  if (cmdState == 1)
    {
      cmdState = 0;
      *rawindex = 0;
      memset(pLgMaster->gRawBuffer, 0, kMaxDataLength);
      memcpy(pLgMaster->gRawBuffer, data, data_len);
      *rawindex += data_len;
    }
  else if (cmdState == 0)
    {
      memcpy(&pLgMaster->gRawBuffer[*rawindex], data, data_len );
      *rawindex += data_len;
    }
  // If we get A1 or A2 from host, need to allow 
  if (*rawindex < COMM_RECV_MIN_LEN)
    {
      // 0xA2 is a special emergency -- reset byte
      if (*pLgMaster->gRawBuffer == 0xA1)
	{
	  pLgMaster->gotA1 = 1;
	  return(0);
	}
      else if (*pLgMaster->gRawBuffer == 0xA2)
	{
	  SlowDownAndStop(pLgMaster);
	  if (pLgMaster->gotA1)
	    {
	      syslog(LOG_ERR, "received 0xA1 & 0xA2, sending 0xA3" );
	      SendA3(pLgMaster);
	      pLgMaster->gotA1 = 0;  // Start all over for A1->A2->A3 resync
	      
	    }
	  else
	    {
	      syslog(LOG_ERR, "received 0xA2 with no 0xA1 predecessor");
	      cmdState = 1;
	    }
	  return(0);
	}
      else
	{
	  syslog(LOG_ERR, "PARSEDATA:  Extraneous data received %d, len %d",
		  *pLgMaster->gRawBuffer, *rawindex);
	  cmdState = 1;
	  return(0);
	}
    }

  // Falling through A1/A2 check here with length less minimum payload size fails
  if (*rawindex < COMM_RECV_MIN_LEN)
    {
      syslog(LOG_ERR, "PARSEDATA:  Bad input length %x", *rawindex);
      cmdState = 1;
      return(0);
    }
  if (pLgMaster->gHEX == 1 ) {
    index = 0;
    for ( i=0; i < *rawindex; i++ ) {
      if ((pLgMaster->gRawBuffer[i] == 0x80) && (i < (*rawindex-1)))
	{
	  pLgMaster->gInputBuffer[index] = 0x80 + pLgMaster->gRawBuffer[i+1];
	  i++;
	  index++;
	}
      else if ((pLgMaster->gRawBuffer[i] == 0x80) && (i == (*rawindex-1)))
	syslog(LOG_ERR, "PARSEDATA:  cannot have 0x80 as end byte");
      else
	{
	  pLgMaster->gInputBuffer[index] = pLgMaster->gRawBuffer[i];
	  index++;
	}
    }
  }
  if (pLgMaster->gHEX == 0)
    {
      memcpy( pLgMaster->gInputBuffer, pLgMaster->gRawBuffer, *rawindex );
      index = *rawindex;
    }
  pLgMaster->newCommand  = pLgMaster->gInputBuffer[0];
  pLgMaster->gHeaderSpecialByte = pLgMaster->gInputBuffer[1];
  pLgMaster->seqNo    =  (pLgMaster->gInputBuffer[2] << 8) + pLgMaster->gInputBuffer[3];

#ifdef AGS_DEBUG
  // Track all commands incoming
  syslog(LOG_NOTICE, "PARSEDATA:  newCommand rcvd %02x,  seqNo %04x, cmdLen = %d",
	 pLgMaster->newCommand, pLgMaster->seqNo,index);
#endif
#ifdef ZDEBUG
  ptr = pLgMaster->gInputBuffer;
  ii = 0;
  if ( index >= 16  ) {
            syslog( LOG_DEBUG ,
                "parse %5d " " %2x %2x %2x %2x" " %2x %2x %2x %2x" " %2x %2x %2x %2x" " %2x %2x %2x %2x"
               , ii
               , ptr[ii +  0] , ptr[ii +  1] , ptr[ii +  2] , ptr[ii +  3]
               , ptr[ii +  4] , ptr[ii +  5] , ptr[ii +  6] , ptr[ii +  7]
               , ptr[ii +  8] , ptr[ii +  9] , ptr[ii + 10] , ptr[ii + 11]
               , ptr[ii + 12] , ptr[ii + 13] , ptr[ii + 14] , ptr[ii + 15]
               );

     
  }
  ii = 16;
  if ( index >= 32  ) {
            syslog( LOG_DEBUG ,
                "parse %5d " " %2x %2x %2x %2x" " %2x %2x %2x %2x" " %2x %2x %2x %2x" " %2x %2x %2x %2x"
               , ii
               , ptr[ii +  0] , ptr[ii +  1] , ptr[ii +  2] , ptr[ii +  3]
               , ptr[ii +  4] , ptr[ii +  5] , ptr[ii +  6] , ptr[ii +  7]
               , ptr[ii +  8] , ptr[ii +  9] , ptr[ii + 10] , ptr[ii + 11]
               , ptr[ii + 12] , ptr[ii + 13] , ptr[ii + 14] , ptr[ii + 15]
               );

     
  }
#endif

  switch(pLgMaster->newCommand) {
    // 0x02  kSTOP
  case kStop:
    if ( index >=(int)(kSizeOfCommand + 2)) {
      cmdState = 1;
      if ( kCRC_OK ==
	   CheckCRC(pLgMaster->gInputBuffer, kSizeOfCommand))
	{
	  SendConfirmation (pLgMaster, kStop);
	  SearchBeamOff(pLgMaster);
	  DoStopCmd(pLgMaster, 0);
	}
      else
	{
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
	}
    }
    break;
  case kDisplayChunksStart:
    cmdSize = kSizeOfCommand +kSizeOfDisplayChunksStartParameters;
    if ( index >= cmdSize + 2 ) {
      cmdState = 1;
      if (kCRC_OK ==
	   CheckCRC(pLgMaster->gInputBuffer, cmdSize))
	{
	  SendConfirmation (pLgMaster, kDisplayChunksStart);
	  DoDisplayChunksStart(pLgMaster, (struct parse_chunkstart_parms *)pInp, kRespondExtern);
	}
      else
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
    }
    break;
  case kDisplayChunksData:
    cmdSize = kSizeOfCommand
      + offsetof(struct parse_chunkdata_parms, chunk_apt_data);
#ifdef ZDEBUG
syslog( LOG_DEBUG, "chunksdata index %d  cmdsize %d", index, cmdSize );
#endif
    if (index > cmdSize) {
      // NOTE INCOMING PARMS (UINT32) ARE BIG-ENDIAN
      dataLength = htonl(((struct parse_chunkdata_parms *)pInp)->chunk_len);
      dataOffset = htonl(((struct parse_chunkdata_parms *)pInp)->chunk_offset);
      cmdSize += dataLength;
#ifdef ZDEBUG
syslog( LOG_DEBUG, "chunksdata datalen %d  offset %d  cmdsize %d", dataLength, dataOffset, cmdSize );
#endif
      // Check to see if this is a restart
      if (pLgMaster->gDataChunksLength && pLgMaster->gTransmitLengthSum && (dataOffset == 0))
	{
	  // Restarting AddChunks command, wipe out data collected so far
	  // but need to preserve total length from ChunkStart command.
	  ResetPlyCounter(pLgMaster);
	  pLgMaster->gTransmitLengthSum = 0;
	  memset(pLgMaster->gDataChunksBuffer, 0, kMaxDataLength);
	}
      if (index >= cmdSize + 2)
	{
	  cmdState = 1;
	  if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				  (kSizeOfCommand
				   + offsetof(struct parse_chunkdata_parms, chunk_apt_data)
				   + dataLength)))
	    {
	      SendConfirmation (pLgMaster, kDisplayChunksData);
	      AddDisplayChunksData (pLgMaster, dataLength, dataOffset,
				    (char *)((struct parse_chunkdata_parms *)pInp)->chunk_apt_data, kRespondExtern);
	    }
	  else
	    {
#ifdef AGS_DEBUG
	      pCrc = (uint32_t *)((char *)pLgMaster->gInputBuffer + (kSizeOfCommand
								     + offsetof(struct parse_chunkdata_parms, chunk_apt_data)
								     + dataLength));
	      
	      syslog(LOG_DEBUG,"ADDCHUNKS CRC ERROR, CRC = %x",*pCrc);
#endif
	      SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
	    }
	}
    }
    break;
  case kFlexDisplay:
    dataLength = ((struct parse_flexdisp_parms *)pInp)->inp_dataLength;
    if (index > 8)
      {
	cmdSize = kSizeOfCommand + kSizeOfFlexDisplayParameters + dataLength;
	if (index >= cmdSize + 2)
	  {
	    cmdState = 1;
	    if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				    (kSizeOfCommand +
				     kSizeOfFlexDisplayParameters +
				     dataLength)))
	      {
		SendConfirmation (pLgMaster, kDisplay);
		gDisplayDataBuffer =
		  (char *)&(pLgMaster->gParametersBuffer[kSizeOfFlexDisplayParameters]);
		DoFlexDisplay(pLgMaster,
			      dataLength,
			      (struct parse_flexdisp_parms *)pInp,
			      gDisplayDataBuffer);
	      }
	    else
	    SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
	  }
      }
    break;
  case kDisplayKitVideo:
    if (index > 8)
      {
	dataLength = ((struct parse_dispkitvid_parms *)pInp)->inp_aptlen;
	cmdSize = kSizeOfCommand + kSizeOfDisplayKitVideoParameters + dataLength;
      if (index >= cmdSize + 2)
	{
	  cmdState = 1;
	  if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				  (kSizeOfCommand 
				   + kSizeOfDisplayKitVideoParameters
				   + dataLength)))
	    {
	      SendConfirmation(pLgMaster, kDisplayKitVideo);
	      gDisplayDataBuffer = (char *)&pLgMaster->gParametersBuffer[kSizeOfDisplayKitVideoParameters];
	      DoDisplayKitVideo(pLgMaster,
				dataLength,
				(unsigned char *)((struct parse_dispkitvid_parms *)pInp)->inp_transform,
				gDisplayDataBuffer,
				kRespondExtern);
	    }
	  else
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
	}
      }
    break;
  case kDisplayChunksDo:
    cmdSize = kSizeOfCommand + kSizeOfDisplayChunksDoParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand +
				 kSizeOfDisplayChunksDoParameters)))
	  {
	    SendConfirmation (pLgMaster, kDisplayChunksDo);
	    DoDisplayChunks(pLgMaster,
			    (struct parse_chunksdo_parms *)pLgMaster->gParametersBuffer,
			    kRespondExtern);
	  }
	else
	SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kFlexDisplayChunksDo:
    cmdSize = kSizeOfCommand + kSizeOfFlexDisplayChunksDoParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand +
				 kSizeOfFlexDisplayChunksDoParameters)))
	  {
	    SendConfirmation(pLgMaster, kFlexDisplayChunksDo);
	    DoFlexDisplayChunks(pLgMaster,
				(struct parse_chunkflex_parms *)pLgMaster->gParametersBuffer,
				kRespondExtern );
	  }
	else
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kDisplaySeveral:
    cmdSize = kSizeOfCommand + kSizeOfDisplaySeveralParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
      
	// Note this input is in BIG endian format on the wire
	number = ntohl(((struct parse_dispsev_parms *)pInp)->num_seq);
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand +
				 kSizeOfDisplaySeveralParameters)))
	  {
	    if ((pLgMaster->gHeaderSpecialByte & 0x80) && (number > kMaxNumberOfPlies))
	      {
		memset(pResp, 0, sizeof(struct parse_basic_resp));
		pResp->hdr.status = RESPFAIL;
		pResp->hdr.errtype = htons(RESPTOOMANYPLIES);
		HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp) -kCRCSize), kRespondExtern);
	      }
	    else
	      {
		SendConfirmation (pLgMaster, kDisplaySeveral);
		SetDisplaySeveral(pLgMaster, number, kRespondExtern);
	      }
	  }
      }
    break;
  case kFileGetStart:
    cmdSize = kSizeOfCommand + kSizeOfFileGetStartParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfFileGetStartParameters )))
	  {
	    SendConfirmation(pLgMaster, kFileGetStart);
	    DoFileGetStart(pLgMaster, (char *)pLgMaster->gParametersBuffer, kRespondExtern);
	  }
	else
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kFileGetData:
    cmdSize = kSizeOfCommand + kSizeOfFileGetDataParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfFileGetDataParameters)))
	  {
	    SendConfirmation(pLgMaster, kFileGetData);
	    DoFileGetData (pLgMaster, (char *)pLgMaster->gParametersBuffer, kRespondExtern );
	  }
	else
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kFilePutStart:
    cmdSize = kSizeOfCommand + kSizeOfFilePutStartParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfFilePutStartParameters )))
	  {
	    SendConfirmation (pLgMaster, kFilePutStart);
	    DoFilePutStart (pLgMaster, (char *)pLgMaster->gParametersBuffer, kRespondExtern );
	  }
	else
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kFilePutData:
    if (index >= (int)(offsetof(struct parse_putdata_parms, inp_buffer)))
      {
	dataLength = ((struct parse_putdata_parms *)pInp)->inp_buf_numbytes;
	cmdSize = kSizeOfCommand + kSizeOfFilePutDataParameters + dataLength;
#ifdef ZDEBUG
syslog(LOG_NOTICE, "parse FilePutData  datalength %d size %d", dataLength, cmdSize );
#endif
      if (index >= cmdSize + 2)
	{
	  cmdState = 1;
	  if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				  (kSizeOfCommand
				   + kSizeOfFilePutDataParameters
				   + dataLength)))
	    {
	      SendConfirmation (pLgMaster, kFilePutData);
	      HandleFilePutData(pLgMaster, (char *)pLgMaster->gParametersBuffer,
				kRespondExtern );
	    }
	else
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
	}
      }
    break;
  case kFilePutDone:
    cmdSize = kSizeOfCommand + kSizeOfFilePutDoneParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfFilePutDoneParameters)))
	  {
	    SendConfirmation (pLgMaster, kFilePutDone);
	    DoFilePutDone (pLgMaster, (char *)pLgMaster->gParametersBuffer, kRespondExtern );
	  }
	else
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kGoAngle:
    cmdSize = kSizeOfCommand + kSizeOfGoAngleParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfGoAngleParameters)))
	  {
	    SendConfirmation (pLgMaster, kGoAngle);
	    DoGoAngle(pLgMaster, (struct parse_goangle_parms *)pInp, kRespondExtern);
	}
      else
	SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
    }
    break;
  case kDarkAngle:
    cmdSize = kSizeOfCommand + kSizeOfDarkAngleParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfDarkAngleParameters)))
	  {
	    SendConfirmation(pLgMaster, kDarkAngle);
	    DarkAngle(pLgMaster, (struct parse_dkangle_parms *)pLgMaster->gParametersBuffer, kRespondExtern );
	  }
	else
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kFindOneTarget:
    cmdSize = kSizeOfCommand + kSizeOfFindOneTargetParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfFindOneTargetParameters)))
	  {
	    SendConfirmation(pLgMaster, kFindOneTarget);
	    DoFindOneTarget(pLgMaster, (struct parse_findonetgt_parms *)pInp, kRespondExtern);
	  }
	else
	  SendConfirmation (pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kQuickCheckCount:
    cmdSize = kSizeOfCommand + kSizeOfQuickCheckCountParameters;
    // special byte used to specify
    // minimum number of target to fail
    // for QuickCheck
    // to change the gHeaderSpecialByte must be valid
    if ((pLgMaster->gHeaderSpecialByte > 0)
	&& (pLgMaster->gHeaderSpecialByte < kNumberOfFlexPoints))
      gQuickFailNumber = pLgMaster->gHeaderSpecialByte;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfQuickCheckCountParameters)))
	  {
	    SendConfirmation (pLgMaster, kQuickCheckCount);
	    // Kludge
	    usleep( 2000 );
	    // Kludge
	    DoQCcount(pLgMaster, (char *)pLgMaster->gParametersBuffer, kRespondExtern );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kQuickCheckTimer:
    cmdSize = kSizeOfCommand + kSizeOfQuickCheckTimerParameters;
    // special byte used to specify
    // minimum number of target to fail
    // for QuickCheck
    // to change the gHeaderSpecialByte must be valid
    if ((pLgMaster->gHeaderSpecialByte > 0)
	 && (pLgMaster->gHeaderSpecialByte < kNumberOfFlexPoints))
      gQuickFailNumber = pLgMaster->gHeaderSpecialByte;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfQuickCheckTimerParameters)))
	  {
	    SendConfirmation (pLgMaster, kQuickCheckTimer);
	    DoQCtimer(pLgMaster, (char *)pLgMaster->gParametersBuffer, kRespondExtern);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
   case kSetBit:
    cmdSize = kSizeOfCommand + kSizeOfSetBitParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer, cmdSize))
	  {
	    SendConfirmation(pLgMaster, kSetBit);
	    DoSetBit(pLgMaster, (struct parse_setbit_parms *)pLgMaster->gParametersBuffer, kRespondExtern);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kQuickCheck:
    cmdSize = kSizeOfCommand + kSizeOfQuickCheckParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfQuickCheckParameters)))
	  {
	    SendConfirmation(pLgMaster, kQuickCheck);
	    DoQuickCheck (pLgMaster, (struct parse_qkcheck_parms *)pLgMaster->gParametersBuffer, kRespondExtern);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kTakePicture:
#ifdef SPECIAL
    gettimeofday( &tv, &tz );
    syslog(LOG_NOTICE, "parse661 tv %d %d", tv.tv_sec, tv.tv_usec );
#endif
    cmdSize = kSizeOfCommand + kSizeOfTakePictureParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
      if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
			      (kSizeOfCommand + kSizeOfTakePictureParameters)))
	{
	  SendConfirmation(pLgMaster, kTakePicture);
#ifdef SPECIAL
	  gettimeofday( &tv, &tz );
	  syslog(LOG_NOTICE,"parse675 tv %d %d", tv.tv_sec, tv.tv_usec);
#endif
	  // Referred to as MOVECAMERA in LaserGuide
	  syslog(LOG_NOTICE,"about to DoTakePicture" );
          if ( pLgMaster->projector_mode == PROJ_LASER )
            {
            L2VtakePicture ( pLgMaster
                           , (struct parse_takepic_parms *)pLgMaster->gParametersBuffer
                           , kRespondExtern
                           );
            }
          else
            {
            DoTakePicture (pLgMaster, (struct parse_takepic_parms *)pLgMaster->gParametersBuffer, kRespondExtern );
            }

#ifdef SPECIAL
	  gettimeofday( &tv, &tz );
	  syslog(LOG_NOTICE, "parse678 tv %d %d\n", tv.tv_sec, tv.tv_usec);
#endif
	}
      else
	SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kCalibrateXY:
    cmdSize = kSizeOfCommand + kSizeOfCalibrateXYParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfCalibrateXYParameters)))
	  {
	    SendConfirmation(pLgMaster, kCalibrateXY);
	    CalibXY(pLgMaster, (struct parse_calibxy_parms *)pLgMaster->gParametersBuffer, kRespondExtern);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kFlexQuickCheck:
    cmdSize = kSizeOfCommand + kSizeOfFlexQuickCheckParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfFlexQuickCheckParameters)))
	  {
	    SendConfirmation(pLgMaster, kFlexQuickCheck);
	    DoFlexQuickCheck (pLgMaster, (struct parse_flexqkchk_parms *)pInp, kRespondExtern );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kThresholdQuickCheck:
    cmdSize = kSizeOfCommand + kSizeOfThresholdQuickCheckParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfThresholdQuickCheckParameters)))
	  {
	    SendConfirmation(pLgMaster, kThresholdQuickCheck);
	    DoThresholdQuickCheck (pLgMaster, (struct parse_thqkchk_parms *)pLgMaster->gParametersBuffer, kRespondExtern );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kHobbsSet:
    cmdSize = kSizeOfCommand + kSizeOfHobbsSetParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfHobbsSetParameters)))
	  {
	    SendConfirmation(pLgMaster, kHobbsSet);
	    DoHobbsSet (pLgMaster, (struct parse_hobbsset_parms *)pLgMaster->gParametersBuffer, kRespondExtern );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kHobbsGet:
    cmdSize = kSizeOfCommand + kSizeOfHobbsGetParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfHobbsGetParameters)))
	  {
	    SendConfirmation(pLgMaster, kHobbsGet);
	    DoHobbsGet (pLgMaster, (struct parse_hobbsget_parms *)pLgMaster->gParametersBuffer, kRespondExtern );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kEtherAngle:
    cmdSize = kSizeOfCommand + kSizeOfEtherAngleParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfEtherAngleParameters)))
	  {
	    SendConfirmation(pLgMaster, kEtherAngle);
	    DoEtherAngle(pLgMaster, (struct parse_ethangle_parms *)pInp, kRespondExtern);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kTFS:     /* Three-Finger-Salute   i.e. reboot */
    cmdSize = kSizeOfCommand;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer, kSizeOfCommand))
	  {
	    syslog(LOG_ERR, "I'm melting!" );
	    SendConfirmation(pLgMaster, kTFS);
	    doClearReadyLED(pLgMaster);
	    doClearLinkLED(pLgMaster);

	    usleep( 2000000L );
	    //	  reboot( magic, magic_too, flag );
	    syslog(LOG_ERR, "I'm freezing!" );
	    /* just in case */
	    usleep( 2000000L );
            system("echo b > /proc/sysrq-trigger" );
	    //	  reboot( flag );
	    //	  reboot( flag );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case ksuperFOM:
    cmdSize = kSizeOfCommand;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer, kSizeOfCommand))
	  {
	    SendConfirmation(pLgMaster, ksuperFOM);
	    DosuperFOM (pLgMaster, kRespondExtern );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kGetTargetsUsed:
    cmdSize = kSizeOfCommand;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer, cmdSize))
	{
	  SendConfirmation(pLgMaster, kGetTargetsUsed);
	  GetTargetsUsed (pLgMaster, kRespondExtern );
	}
      else
	SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kGetDacs:
     cmdSize = kSizeOfCommand;
     if (index >= cmdSize + 2)
       {
 	cmdState = 1;
 	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer, kSizeOfCommand))
 	  {
 	    SendConfirmation(pLgMaster, kGetDacs);
 	    DoGetDacs(pLgMaster, kRespondExtern );
 	  }
 	else
 	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
       }
     break;
  case kRefreshRate:
    cmdSize = kSizeOfCommand;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer, kSizeOfCommand))
	  {
	    SendConfirmation(pLgMaster, kRefreshRate);
	    DoRefreshRate ( pLgMaster, kRespondExtern );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kChangeDisplayPeriod:
    cmdSize = kSizeOfCommand + kSizeOfChangeDisplayPeriodParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer, cmdSize))
	  {
	    SendConfirmation(pLgMaster, kChangeDisplayPeriod);
	    DoChangeDisplayPeriod (pLgMaster, (struct parse_chngdisp_parms *)pLgMaster->gParametersBuffer );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kChangeTransformTolerance:
    cmdSize = kSizeOfCommand + kSizeOfChangeTransformToleranceParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer, cmdSize))
	  {
	    SendConfirmation(pLgMaster, kChangeTransformTolerance);
	    DoChangeTransformTolerance(pLgMaster, (struct parse_chngxfrmtol_parms *)pLgMaster->gParametersBuffer);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kReInit:
    cmdSize = kSizeOfCommand;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer, kSizeOfCommand))
	  {
	    SendConfirmation(pLgMaster, kReInit);
	    DoReInit(pLgMaster, kRespondExtern );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kRightOnReg:
    cmdSize = kSizeOfCommand + kSizeOfRightOnRegParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfRightOnRegParameters)))
	  {
	    SendConfirmation(pLgMaster, kRightOnReg);
	    RightOnFullReg ( pLgMaster,
			     (struct parse_rightondofullreg_parms *)pLgMaster->gParametersBuffer,
			     kRespondExtern );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kRegWithFeedback:
    cmdSize = kSizeOfCommand + kSizeOfRegWithFeedbackParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfRegWithFeedbackParameters)))
	  {
	    SendConfirmation(pLgMaster, kRegWithFeedback);
	    FullRegWithFeedback(pLgMaster,
				(struct parse_rightondofullregwithfeedback_parms *)pLgMaster->gParametersBuffer,
				kRespondExtern);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kFlexRegWithFeedback:
    cmdSize = kSizeOfCommand + kSizeOfFlexRegWithFeedbackParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand+kSizeOfFlexRegWithFeedbackParameters)))
	  {
	    SendConfirmation(pLgMaster, kFlexRegWithFeedback);
	    FlexFullRegWithFeedback(pLgMaster,
				    (struct parse_flexrightondofullregwithfeedback_parms *)pLgMaster->gParametersBuffer,
				    kRespondExtern);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kCalculateTransform:
    cmdSize = kSizeOfCommand + kSizeOfCalculateTransformParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfCalculateTransformParameters)))
	  {
	    SendConfirmation(pLgMaster, kCalculateTransform);
	    CalculateTransform(pLgMaster, (struct parse_clcxfrm_parms *)pLgMaster->gParametersBuffer,
			     kRespondExtern );
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kFlexCalculateTransform:
    cmdSize = kSizeOfCommand + kSizeOfFlexCalculateTransformParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfFlexCalculateTransformParameters)))
	  {
	    SendConfirmation(pLgMaster, kFlexCalculateTransform);
	    FlexCalculateTransform(pLgMaster,
				   (struct parse_flexcalxfrm_parms *)pLgMaster->gParametersBuffer,
				   kRespondExtern);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kFlexCalWithFeedback:
    cmdSize = kSizeOfCommand + kSizeOfFlexCalWithFeedbackParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfFlexCalWithFeedbackParameters)))
	  {
	    SendConfirmation(pLgMaster, kFlexCalWithFeedback);
	    FlexCalWithFeedback(pLgMaster,
				(struct parse_flexcalxfdbk_parms *)pLgMaster->gParametersBuffer,
				kRespondExtern);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kShowTargets:
    if (index >= (int)(2*(sizeof(uint32_t))))
      {
	// FIXME---PAH---NEEDS CMD/RESP STRUCT
	nTargets = *((int32_t *)pLgMaster->gParametersBuffer);
	dataLength = nTargets * kSizeOf1TargetData;
	cmdSize = kSizeOfCommand + kSizeOfShowTargetsParameters + dataLength;
      if (index >= cmdSize + 2)
	{
	  cmdState = 1;
	  if (kCRC_OK ==CheckCRC(pLgMaster->gInputBuffer,
				 (kSizeOfCommand + kSizeOfShowTargetsParameters
				  + dataLength)))
	    {
	      SendConfirmation(pLgMaster, kShowTargets);
	      DoShowTargets( pLgMaster, (struct parse_showtgt_parms *)pLgMaster->gParametersBuffer, kRespondExtern );
	    }
	  else
	    SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
	}
      }
    break;
  case kRightOnCert:
    cmdSize = kSizeOfCommand + kSizeOfRightOnCertParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfRightOnCertParameters)))
	  {
	    SendConfirmation(pLgMaster, kRightOnCert);
	    RightOnCert(pLgMaster, (struct parse_rtoncert_parms *)pLgMaster->gParametersBuffer, kRespondExtern );
	  }
      else
	SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kFracCert:
    cmdSize = kSizeOfCommand + kSizeOfFracCertParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfFracCertParameters)))
	  {
	    SendConfirmation(pLgMaster, kFracCert);
	    FracCert(pLgMaster, (struct parse_fraccert_parms *)pLgMaster->gParametersBuffer, kRespondExtern );
	  }
      else
	SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kAutoFocusCmd:
    cmdSize = kSizeOfCommand + kSizeOfAutoFocusCmdParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfAutoFocusCmdParameters)))
	  {
	    SendConfirmation(pLgMaster, kAutoFocusCmd);
	    DoAutoFocusCmd(pLgMaster, pLgMaster->gParametersBuffer);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  case kDimAngle:
    cmdSize = kSizeOfCommand + kSizeOfDimAngleParameters;
    if (index >= cmdSize + 2)
      {
	cmdState = 1;
	if (kCRC_OK == CheckCRC(pLgMaster->gInputBuffer,
				(kSizeOfCommand + kSizeOfDimAngleParameters)))
	  {
	    DimAngle(pLgMaster, (struct parse_dimangle_parms *)pLgMaster->gParametersBuffer, kRespondExtern);
	    SendConfirmation(pLgMaster, kDimAngle);
	  }
	else
	  SendConfirmation(pLgMaster, kCRC16NoMatchMsg);
      }
    break;
  default:
    cmdState = 1;
    break;
  }
  return(0);
}
