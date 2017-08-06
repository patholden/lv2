//static char rcsid[] = "$Id: Main.c,v 1.29 2001/01/03 18:02:20 ags-sw Exp ags-sw $";

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <math.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "parse_data.h"
#include "comm_loop.h"
#include "AppCommon.h"
#include "SystemMaint.h"
#include "UserMaint.h"
#include "CRCHandler.h"
#include "LaserCmds.h"
#include "LaserInterface.h"
#include "LaserPattern.h"
#include "AngleCorrections.h"
#include "SensorSearch.h"
#include "Init.h"
#include "FOM.h"
#include "Hobbs.h"
#include "DoCoarseScan.h"
#include "Video.h"
#include "3DTransform.h"
#include "APTParser.h"
#include "DoAutoFocusCmd.h"
#include "Files.h"
#include "compile.h"
#include "asciiID.h"

//Statics for this file
struct ags_bkgd_thread_struct thread_data[MAX_NUM_THREADS];
pthread_mutex_t          lock;
const char *ags_banner =
    "AGS LaserGuide "  AGS_ASCII_ID  "  " AGS_UNIX_TIME;


static	int	InitUserStuff (struct lg_master *pLgMaster);
static	int32_t	exit_check( void );
static void	CloseUserStuff(struct lg_master *pLgMaster);

char * gQCsaveData=0;
uint32_t *scandata=0;
uint32_t gQClengthOfData;
unsigned char                   gHeaderSpecialByte = 0;

// This points to all the config data used by this application
// Make it global so debugger can see (like GDB)
// Should consider using sqlite to store all data.....
struct lg_master *pConfigMaster=0;

int  gSearchCurrentSensor;
int gSuperFineCount = 256;
int gSuperFineSkip = 1;

int gVideoCount = 0;
int gVideoCheck = 0;

int32_t gQuickCheck = 0;

uint32_t gVideoPreDwell = 0;

int gForceTransform = 0;
int termination_flag=0;

static void sigact_handler(void)
{
  termination_flag = 1;
}
static int main_capture_signals(void)
{
  struct   sigaction sigact;

  return(0);
  memset(&sigact, 0, sizeof(struct sigaction));

  sigact.sa_handler = (void *)&sigact_handler;
  sigact.sa_flags  = SA_SIGINFO;
  sigfillset(&sigact.sa_mask);
  
  if (((sigaction(SIGINT, &sigact, 0)) < 0) || 
      ((sigaction(SIGQUIT, &sigact, 0)) < 0) ||
      ((sigaction(SIGTERM, &sigact, 0)) < 0) ||
      ((sigaction(SIGKILL, &sigact, 0)) < 0))
    return(-1);

  return(0);
}
static void ags_cleanup(struct lg_master *pLgMaster)
{
  // Do housekeeping before exiting program
  CloseUserStuff (pLgMaster );
  if (gQCsaveData)
    free(gQCsaveData);
  doClearLinkLED(pLgMaster);
  doSetReadyLED(pLgMaster);
  if (pLgMaster->socketfd >= 0)
    close(pLgMaster->socketfd);
  if (pLgMaster->datafd >= 0)
    close(pLgMaster->datafd);
  pLgMaster->socketfd = -1;
  pLgMaster->datafd = -1;
  LGMasterFree(pLgMaster);
  free(pLgMaster);
  return;
}

static void *ags_bkgd_proc(void *p_threadarg)
{
  struct ags_bkgd_thread_struct *p_my_data;
  struct lg_master *pHobbsMaster;

  while(1)
    {
      p_my_data = (struct ags_bkgd_thread_struct *) p_threadarg;
      if (!p_my_data->pLgMaster)
	{
	  syslog(LOG_ERR,"\ninvalid-input");
	  return(NULL);
	}
      pHobbsMaster = p_my_data->pLgMaster;
      if (p_my_data->time_to_update)
	{
	  WriteHobbs(pHobbsMaster);
	  p_my_data->time_to_update = 0;   // Reset flag
	}
      sleep(600);  // Check every 10 minutes
    }
  // Should never get here
  return(NULL);
}
int main ( int argc, char **argv )
{
  pthread_t     thread[MAX_NUM_THREADS];
  int     error=0;


  openlog("agsd", 0, LOG_USER);
  syslog(LOG_NOTICE, "%s", ags_banner);

  /* defaults */
  gBeamLinearRangeX = GBEAMLNRNG_DEF;	
  gBeamLinearRangeY = GBEAMLNRNG_DEF;	
  gSlowExponent = 8;
  gFastExponent = 11;
  gSlowBabies = 1U << gSlowExponent;
  gFastBabies = 1U << gFastExponent;
  gFOM        = -1;
  gMaxPiledPts= 9;
//
//  try board initialization here
//
  pConfigMaster = malloc(sizeof(struct lg_master));
  if (!pConfigMaster)
    {
      syslog(LOG_ERR, "LG_MASTER_MALLOC Failed\n");
      closelog();
      exit(EXIT_FAILURE);
    }
  HugeInit();
  error = LGMasterInit(pConfigMaster);
  if (error)
    {
      syslog(LOG_ERR,"\nMain: Can't initialize master struct, err %d",error);
      free(pConfigMaster);
      closelog();
      exit(EXIT_FAILURE);
    }
  error = InitBoard(pConfigMaster);
  if (error)
    {
      syslog(LOG_ERR,"\nMain: Can't start board laser-fd %d, err %d, errno %d",pConfigMaster->fd_laser, error, errno);
      free(pConfigMaster);
      closelog();
      exit(EXIT_FAILURE);
    }
  syslog(LOG_NOTICE,"\nMAIN: INITBOARD complete for /dev/laser, fd %d",pConfigMaster->fd_laser);
  error = ConfigDataInit(pConfigMaster);
  if (error)
    {
      syslog(LOG_ERR,"\nConfigDataInit: Can't get config data, err %d", error);
      free(pConfigMaster);
      closelog();
      exit(EXIT_FAILURE);
    }

  error = InitVision(pConfigMaster);
  if (error)
    {
      syslog(LOG_ERR,"\nInitVision: Can't get vision data, err %d", error);
    }

  // start background thread for doing updates
#if 0
  if ((pthread_mutex_init(&lock, NULL)) != 0)
    {
      closelog();
      exit(EXIT_FAILURE);
    }
#endif
  // Init thread variables
  memset((char *)&thread_data[0], 0, sizeof(thread_data));
  memset((char *)&thread, 0, sizeof(thread));
  
  error = CommInit(pConfigMaster);
  if (error)
    {
      syslog(LOG_ERR,"\nMain: Can't initialize Ethernet/Serial Interfaces");
      free(pConfigMaster);
      closelog();
      exit(EXIT_FAILURE);
    }
	    
  if (pConfigMaster->gHEX == 0)
    syslog(LOG_NOTICE, "old binary serial I/O\n");
  else if (pConfigMaster->gHEX == 1)
    syslog(LOG_NOTICE, "limited HEX serial I/O in ASCII hex\n");
  else if (pConfigMaster->gHEX == 2)
    syslog(LOG_NOTICE, "full HEX serial I/O in ASCII hex\n" );

  syslog(LOG_NOTICE, "tol %f", pConfigMaster->gArgTol);
  syslog(LOG_NOTICE, " period %d",  pConfigMaster->gPeriod );
  syslog(LOG_NOTICE, " QC %d",  pConfigMaster->gQCcount );
  syslog(LOG_NOTICE, " X %f",  gBeamLinearRangeX );
  syslog(LOG_NOTICE, " Y %f",  gBeamLinearRangeY );
  syslog(LOG_NOTICE, " baud %d",  GBAUD_DEFAULT);
  syslog(LOG_NOTICE, "   gSearchStepPeriod %d", pConfigMaster->gSrchStpPeriod);
  syslog(LOG_NOTICE, "   gFinePeriod %d", pConfigMaster->gFinePeriod);
  syslog(LOG_NOTICE, "   gCoarsePeriod %d", pConfigMaster->gCoarsePeriod);
  syslog(LOG_NOTICE, "   gSenseThreshold %d",  pConfigMaster->gSenseThreshold );
  syslog(LOG_NOTICE, "   gSenseThresholdCoarse %d",  pConfigMaster->gSenseThresholdCoarse );
  syslog(LOG_NOTICE, "   gSenseThresholdFine %d",  pConfigMaster->gSenseThresholdFine );
  syslog(LOG_NOTICE, "   gSenseThresholdSuperFine %d",  pConfigMaster->gSenseThresholdSuperFine );
  syslog(LOG_NOTICE, "   gTargetDrift %d",  gTargetDrift );
  syslog(LOG_NOTICE, " coarse step 0x%x",   pConfigMaster->gCoarse2SearchStep );
  syslog(LOG_NOTICE, " spirals %d",   pConfigMaster->gNumberOfSpirals );
  syslog(LOG_NOTICE, " factor %d",   pConfigMaster->gCoarse2Factor );
  syslog(LOG_NOTICE, " attempt %d",   gNumberOfSensorSearchAttempts );
  SensorInitLog();
  syslog(LOG_NOTICE, " factor spiral %d",   pConfigMaster->gSpiralFactor );
  syslog(LOG_NOTICE, " hatch %d",           pConfigMaster->gHatchFactor );
  syslog(LOG_NOTICE, " SuperFineCount %d", gSuperFineCount);
  syslog(LOG_NOTICE, " SuperFineSkip %d", gSuperFineSkip );
  syslog(LOG_NOTICE, " SuperFineFactor %d",   getSuperFineFactor());
  syslog(LOG_NOTICE, "fast %x",   gFastBabies );
  syslog(LOG_NOTICE, " (expo %d), ",       gFastExponent );
  syslog(LOG_NOTICE, "slow %x",   gSlowBabies );
  syslog(LOG_NOTICE, " (expo %d) ",       gSlowExponent );
  syslog(LOG_NOTICE, "dwell %d ",   pConfigMaster->gDwell );
  syslog(LOG_NOTICE, "MaxPiledPts %d ",   gMaxPiledPts );
  syslog(LOG_NOTICE, "MaxCos %f ",   gMaxCos );
  syslog(LOG_NOTICE, "LongToShort %f ",   gLongToShort );
  syslog(LOG_NOTICE, "CurveInterpolation %f ",   gCurveMin );
  syslog(LOG_NOTICE, "QuickCheck %d ",   gQuickCheck );
  syslog(LOG_NOTICE, "MaxQuickSearches %d ", kMaxQuickSearches);
  syslog(LOG_NOTICE, "MaxDiffMag %f ",   gMaxDiffMag );
  syslog(LOG_NOTICE, "MultipleSweeps %d ", pConfigMaster->gMultipleSweeps );

  doClearLinkLED(pConfigMaster);
  StopPulse(pConfigMaster); 
  if ((InitUserStuff (pConfigMaster)) < 0)
    {
      syslog(LOG_ERR,"\nINIT: Malloc failure");
      ags_cleanup(pConfigMaster);
      closelog();
      exit(EXIT_FAILURE);
    }
  syslog(LOG_NOTICE, " projector_mode %d \n", pConfigMaster->projector_mode );
  if ( pConfigMaster->projector_mode == PROJ_LASER ) {
      syslog(LOG_NOTICE, " projector_mode %d - laser \n", pConfigMaster->projector_mode );
  }
  if ( pConfigMaster->projector_mode == PROJ_VISION ) {
      syslog(LOG_NOTICE, " projector_mode %d - vision \n", pConfigMaster->projector_mode );
  }
  SlowDownAndStop (pConfigMaster);
  SetQCcounter(pConfigMaster, 0);
  SearchBeamOff(pConfigMaster);

  // Set up program to capture user-entered signals
  if ((main_capture_signals()) < 0)
    {
      syslog(LOG_ERR,"\nSIGACT failure");
      ags_cleanup(pConfigMaster);
      closelog();
      exit(EXIT_SUCCESS);
    }
  // The main loop should never really exit.
  syslog(stderr, " \n");
  syslog(stderr, "Waiting to Accept packets");
  // Before jumping into main loop, start background thread
  // will is only used for updating hobbs counters once every 2 hours
  thread_data[0].pLgMaster = pConfigMaster;
  if (pthread_create(&thread[0], NULL, ags_bkgd_proc, (void *) &thread_data[0]))
    {
      syslog(stdout, "Error creating thread\n");
      ags_cleanup(pConfigMaster);
      closelog();
      exit(EXIT_FAILURE);
    }
  doClearReadyLED(pConfigMaster);
  while (exit_check()) {

      // if ( TestEtherOrSerial( pConfigMaster ) == 0 )
      //    continue;
      
      pConfigMaster->serial_ether_flag = 2;

//      if (pConfigMaster->serial_ether_flag == 1) {
// 	  error = DoProcSerial(pConfigMaster);
//      }
      if (pConfigMaster->serial_ether_flag == 2) {
	  error = DoProcEnetPackets(pConfigMaster);
	  if (error < 0)
	    {
	      syslog(LOG_ERR, "COMMLOOP: Read data failure, err = %x", error);
              doClearLinkLED(pConfigMaster);
	      if (pConfigMaster->datafd >= 0)
		{
		  close(pConfigMaster->datafd);
		  pConfigMaster->datafd = -1;
		}
	      else
		{
		  // Try to open Comms interface to PC host
		  if ((CommConfigSockfd(pConfigMaster)) < 0)
		    {
		      syslog(LOG_ERR, "COMMLOOP: Unable to restart Comms, shutting down\n");
		      ags_cleanup(pConfigMaster);
		      closelog();
		      exit(EXIT_FAILURE);
		    }
		}
	    }
	}
  }

  // Clean up AGS specific stuff before exiting
  doSetReadyLED(pConfigMaster);
  doClearLinkLED(pConfigMaster);
  syslog(LOG_NOTICE, "Shutting down do to System or User interrupt");
  ags_cleanup(pConfigMaster);
  closelog();
  exit(EXIT_SUCCESS);
}

int InitUserStuff (struct lg_master *pLgMaster)
{
    gQCsaveData = (char *)malloc((kSizeOfCommand + kMaxParametersLength + kMaxDataLength + kCRCSize));
    scandata = (uint32_t *)calloc(67*67, sizeof(uint32_t));
    if (!gQCsaveData || !scandata)
      return(-1);
  
    FlashLed(pLgMaster, 5);
    SearchBeamOff(pLgMaster);
    InitCRCHandler();
    InitLaserInterface(pLgMaster);
    if (InitLaserPattern())
      return(-1);
    InitAPTParser();
    InitSensorSearch();
    return(0);
}

int32_t exit_check( void )
{
  // Check for user-input trying to kill this program
  if (termination_flag)   // Check for Ctrl-C, do proper shut down
    return(0);

  return(1);
}

static void CloseUserStuff (struct lg_master *pLgMaster)
{

/*	Close_HAL_CELL_Interface (  ); */
        CloseSensorSearch (  );
        CloseAPTParser (  );
        CloseLaserPattern (  );
	CloseLaserInterface (  );
	CloseCRCHandler (  );
        ReleaseBoard(pLgMaster);
}
