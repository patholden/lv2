#ifndef BOARDCOMM_H
#define BOARDCOMM_H
/*   $Id: BoardComm.h,v 1.20 2001/01/03 17:48:53 ags-sw Exp pickle $  */


#define CURRENT_FPGA_VERSION 0x102
#define DONTRESPOND  1      // command will NOT send response to PC host
#define SENDRESPONSE  2     // command will send response to PC host
#define  MAX_NUM_THREADS 1
#define STEPDELAY   500
#define NUM_HOBBS_COUNTERS  4
#define MAX_TARGETSFLEX   24    // kNumberOfFlexTargets
#define MAX_TARGETSUSED   128
#define PARSE_HOBBS_HOBBS 1
#define PARSE_HOBBS_XSCAN 2
#define PARSE_HOBBS_YSCAN 3
#define PARSE_HOBBS_LASER 4
#define PROJ_VISION  1
#define PROJ_LASER   2
#define TGFIND_BUFF_SIZE  MAX_TGFIND_BUFFER * sizeof(int16_t)
#define MAX_DOSENSE_RETRIES 20
#define DOSENSE_LEVEL       30
#define DOSENSE_MID       0x300
#define DOSENSE_MIN_SAMPLES   20
#define GRIDPOS  30.0
#define GRIDNEG  -30.0
#define GRIDSPAN  60.0
#define MINBINVAL  -0x7FFF
#define MAXBINVAL  0x7FFF
#define DBLRND     0.5
#define COARSESEARCH  1
#define FINESEARCH    2
#define SUPERSEARCH   3
#define kMaxUnsigned      0xFFFF
#define kMaxSigned        0x7FFF
#define kMinSigned        (-kMaxSigned)
#define kBinaryCenter	   0.50
#define kCoarseFactorDef   8
#define kCoarseFactorMin   1
#define kCoarseSrchStpDef  8
#define kCoarseSrchStpMin  1
#define kHatchFactorDef    128
#define kCoarseFactor      8
#define kSIX	0
#define kFOUR	0
#define	kMaxNumberOfCALIBAngles	4000
#define	kGridPointsX	61
#define	kGridPointsY	61
#define	kGridOriginX	-30.0
#define	kGridStepX		1.0
#define	kGridOriginY	-30.0
#define	kGridStepY		1.0
#define	kDefaultDeltaMirror .50
#define kXMirrorAngularRange  30.0	
#define kYMirrorAngularRange  30.0	

extern  uint32_t  gRespondToWhom;
struct displayData
{
  struct lg_xydata  *pattern;
  int32_t           *sensorAngles;
  uint32_t          numberOfSensorSets;
};

// The following header is fixed for incoming commands from
// host PC
struct cmd_inhdr {
  unsigned char theCommand;
  unsigned char specialByte;
  uint16_t      seqNo;
};
// k_header is used for responding to caller with basic status
struct k_header {
  union {
    struct {
      unsigned char cmd;
      unsigned char flags;
      uint16_t      seq_num;
    };
    struct {
      unsigned char status;
      unsigned char fill;
      uint16_t      errtype;
    };
    struct {
      unsigned char status1;
      unsigned char errtype1;
      uint16_t      errtype2;
    };
    struct {
      uint32_t      hdr;
    };
    struct {
      unsigned char status2;
      unsigned char qcply_byte1;
      uint16_t      unused;
    };
    struct {
      unsigned char status3;
      unsigned char fill3A;
      unsigned char numTransforms;
      unsigned char fill3B;
    };
  };
} __attribute__ ((packed));
struct version_info {
  char       *pVersions;
  uint32_t   version_size;
  uint32_t   isVersInit;
};

struct lg_master {
  int32_t            foundTarget[MAX_TARGETSFLEX];
  int32_t            gColinear[MAX_TARGETSFLEX];
  int32_t            gCoplanar[MAX_TARGETSFLEX];
  struct sockaddr_in webhost_addr;
  struct hobbs_ctrs hobbs;
  struct version_info   vers_data;
  struct k_header gOutOfRange;
  struct lg_xydata gSaveXY;
  struct lg_xydata gCheckXY;
  char            webhost[128];
  char            visionhost[128];
  unsigned char   gBestTargetArray[MAX_TARGETSUSED];
  double          l2vtransform[12];
  double          gArgTol;
  double          ping_count;
  double          gTolerance;
  double          dmax;
  double          gHalfMirror;
  unsigned char   *gInputBuffer;
  char            *gDataChunksBuffer;
  char            *gSensorBuffer;
  char            *gAFInputBuffer;
  unsigned char   *gRawBuffer;
  unsigned char   *gParametersBuffer;
  unsigned char   *theResponseBuffer;
  int16_t         *gScan;
  uint16_t        *coarsedata;
  uint16_t        *gLsort;
  int             af_serial;
  int             pc_serial;
  int             socketfd;
  int             datafd;
  int             fd_laser;
  int             serial_ether_flag;
  int             projector_mode;
  unsigned long   gProjectorSerialNumber;
  uint32_t        gHEX;
  uint32_t        rcvdStopCmd;
  uint32_t        enet_retry_count;
  uint32_t        gotA1;
  uint32_t        gDataChunksLength;
  uint32_t        gBuiltPattern;
  uint32_t        gSavePeriod;
  uint32_t        gPeriod;
  uint32_t        gDisplayFlag;
  uint32_t        gSrchStpPeriod;
  uint32_t        gCoarsePeriod;
  uint32_t        gFinePeriod;
  uint32_t        gSearchType;
  uint32_t        gSenseThreshold;
  uint32_t        gSenseThresholdCoarse;
  uint32_t        gSenseThresholdFine;
  uint32_t        gSenseThresholdSuperFine;
  uint32_t        gTransmitLengthSum;
  uint32_t        gBestTargetNumber;
  uint32_t        gPlysToDisplay;
  uint32_t        gPlysReceived;
  uint32_t        LongOrShortThrowSearch;
  uint32_t        gNumberOfSpirals;
  uint32_t        gSpiralFactor;
  uint32_t        gDwell;
  uint32_t        gMultipleSweeps;
  int32_t         gQCcount;
  int32_t         gCoarse2Factor;
  int32_t         saveCoarse2Factor;
  int32_t         gHatchFactor;
  int32_t         gCoplanarCount;
  int16_t         gCoarse2SearchStep;
  int16_t         gXcheck;
  int16_t         gYcheck;
  uint16_t        seqNo;
  uint8_t         gCALIBFileOK;
  uint8_t         newCommand;
  uint8_t         gHeaderSpecialByte;
  uint8_t         RFUpad;
  uint8_t         gAbortDisplay;
  uint8_t         gResetComm;
  uint8_t         current_target;
  char            astring[27][8];
  double          xMirrorCorr[27];
  double          yMirrorCorr[27];
  double          xMirrorInv[27];
  double          yMirrorInv[27];

#ifdef ZDEBUG
  int             debug_count;
#endif
#ifdef NEW_TFIND
  int             fd_lv2;
#endif
};

typedef struct ags_bkgd_thread_struct
{
  int              thread_instance;  // (debug)field to distinguish between threads.
  struct lg_master *pLgMaster;      // pointer to master data structure
  int              time_to_update;  // determines when to do backup, set by timer from Main loop
} AGS_BKGD_THREAD;

double ArrayToDouble(double inp_data);
void RandomSegment(void);
int32_t InitQCtimer( void );
int32_t GetQCtimer( void );
void SetROIsearch( void );
void ClearROIsearch(void);
int CheckStopFlag(void);
void PostCommand(struct lg_master *pLgMaster, uint32_t theCommand, char * data,
		 uint32_t respondToWhom);
void SetHighBeam(struct lg_xydata *pDevXYData);
void SetLowBeam(struct lg_xydata *pDevXYData);
void SetDarkBeam(struct lg_xydata *pDevXYData);
int doWriteDevCmdNoData(struct lg_master *pLgMaster, uint32_t command);
int doWriteDevCmd32(struct lg_master *pLgMaster, uint32_t command, uint32_t write_val);
int doWriteCmdMove(struct lg_master *pLgMaster, struct lg_move_data *pMove, uint32_t command);
int doWriteDevDelta(struct lg_master *pLgMaster, struct lg_xydelta *pDelta);
int doWriteDevPoints(struct lg_master *pLgMaster, struct lg_xydata *pXYData);
void StopPulse(struct lg_master *pLgMaster);
int CDRHflag(struct lg_master *pLgMaster);
int32_t GetQCcounter(struct lg_master *pLgMaster);
int SetQCcounter(struct lg_master *pLgMaster, int count);
int initQCcounter(struct lg_master *pLgMaster);
int stopQCcounter(struct lg_master *pLgMaster);
int resetQCcounter(struct lg_master *pLgMaster);
int SearchBeamOff(struct lg_master *pLgMaster);
int SearchBeamOn(struct lg_master *pLgMaster);
int InitBoard(struct lg_master *pLgMaster);
void ReleaseBoard (struct lg_master *pLgMaster);
int JustDoDisplay(struct lg_master *pLgMaster, char * wr_ptr, int patternLength );
void FlashLed(struct lg_master *pLgMaster, int numFlash);         
void SlowDownAndStop(struct lg_master *pLgMaster);
int setROIlength(struct lg_master *pLgMaster, int32_t half_pattern);
int ROIoff(struct lg_master *pLgMaster);
int doLGSTOP(struct lg_master *pLgMaster);
int doROIOff(struct lg_master *pLgMaster);
int doDevDisplay(struct lg_master *pLgMaster, uint32_t ptn_len, uint32_t do_restart);
int doStartPulse(struct lg_master *pLgMaster, struct lg_pulse_data *lg_pulsedata);
int doSetROI(struct lg_master *pLgMaster, uint32_t write_val);
int doSetReadyLED(struct lg_master *pLgMaster);
int doClearReadyLED(struct lg_master *pLgMaster);
int doSetSearchBeam(struct lg_master *pLgMaster);
int doClearSearchBeam(struct lg_master *pLgMaster);
void doClearLinkLED(struct lg_master *pLgMaster);
void doSetLinkLED(struct lg_master *pLgMaster);
int doClearShutterENB(struct lg_master *pLgMaster);
int doSetShutterENB(struct lg_master *pLgMaster);
int doSetXOffset(struct lg_master *pLgMaster, uint32_t xoff);
int doSetYOffset(struct lg_master *pLgMaster, uint32_t yoff);
void ZeroLGoffset(struct lg_master *pLgMaster);
void GoToRaw(struct lg_master *pLgMaster, struct lg_xydata *pRawData);
void GoToPulse(struct lg_master *pLgMaster, struct lg_xydata *pPulseData,
	       uint32_t pulseoffvalue, uint32_t pulseonvalue);
int move_dark(struct lg_master *pLgMaster, struct lg_xydata *pNewData);
int move_lite(struct lg_master *pLgMaster, struct lg_xydata *pNewData);
void move_one_dark(struct lg_master *pLgMaster, struct lg_xydata *pDarkData);
void ResumeDisplay(struct lg_master *pLgMaster);
int32_t GetQCflag(struct lg_master *pLgMaster);
int DoLevelSearch(struct lg_master *pLgMaster, struct lg_xydata *pSrchData,
		  struct lg_xydelta *pDeltaData, uint32_t nPoints, int16_t *c_out,uint32_t do_coarse);
int DoLineSearch(struct lg_master *pLgMaster, struct lg_xydata *pSrchData,
		 struct lg_xydelta *pDeltaData, uint32_t nPoints);
void PostCmdDisplay(struct lg_master *pLgMaster, struct displayData *p_dispdata, int32_t do_response, uint32_t respondToWhom);
void PostCmdEtherAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData);
void PostCmdGoAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData, uint32_t respondToWhom);
void PostCmdDarkAngle(struct lg_master *pLgMaster, struct lg_xydata *pAngleData);
void limitXY(int16_t currentX, int16_t currentY, int16_t *eolXNeg, int16_t *eolXPos, int16_t *eolYNeg,
	     int16_t * eolYPos, int16_t delta);
uint32_t  get_num_disp_points(void);
int doSTOPCMD(struct lg_master *pLgMaster);
#endif // BOARDCOMM_H
