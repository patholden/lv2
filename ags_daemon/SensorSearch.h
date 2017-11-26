#include <stdint.h>
/*   $Id: SensorSearch.h,v 1.12 1999/07/29 18:36:01 ags-sw Exp $  */

#ifndef SENSORSEARCH_H
#define SENSORSEARCH_H

#define kStopWasDone            -1
#define kCoarseNotFound         -2
#define kFineNotFound           -3
#define kXtooSmallToSearch      -4
#define kXtooLargeToSearch      -5
#define kYtooSmallToSearch      -6
#define kYtooLargeToSearch      -7
#define kSuperFineNotFound      -8
#define kSuperFineTooFew        -9
#define kSuperFineFound          2
#define kNumberDrift            12
#define kMaxQuickSearches        3

#define SENSE_BUF_SIZE          sizeof(int16_t) * 500000

#define kTargetReturnsFileFormat 1

extern
void SetFineLevelResults(int16_t firstX,int16_t firstY, int16_t lastX, int16_t lastY,
                                int16_t *MinX, int16_t *MinY, int16_t *MaxX, int16_t *MaxY);


extern void limitCalc(int16_t centerX, int16_t centerY, int16_t *eolXNeg,
               int16_t *eolXPos, int16_t *eolYNeg, int16_t *eolYPos, int32_t nSteps);

extern
void AdjustOneXYSet(int16_t tmpX, int16_t tmpY, int16_t *eolX, int16_t *eolY, int16_t delX, int16_t delY, uint16_t sample);


extern
void AdjustXYLimit(int16_t *eolXPos, int16_t *eolXNeg, int16_t *eolYPos, int16_t *eolYNeg, int16_t delta);


int QuickCheckASensor(struct lg_master *pLgMaster, int16_t centerX, int16_t centerY);
int DoSearch(int16_t startX, int16_t startY, int16_t *foundX, int16_t *foundY);

int QuickCheckASensor(struct lg_master *pLgMaster, int16_t centerX, int16_t centerY);

int QuickCheckOne(struct lg_master *pLgMaster, int16_t centerX, int16_t centerY,
		  int16_t *foundX, int16_t *foundY);
int DoRedundandSuperFineSearch(int16_t *foundX, int16_t *foundY);
int DoSuperFineSearch(int16_t *foundX, int16_t *foundY, int numberOfSuperCrosses);
int DoFineSearch(int16_t *foundX, int16_t *foundY);
void InitDrift(int16_t *Xarr, int16_t *Yarr);
void FindDrift(int16_t cX, int16_t cY, int16_t fX, int16_t fY);
void CorrectDrift(int16_t cX, int16_t cY, int16_t *fX, int16_t *fY );
void SetSuperFineFactor(uint32_t n);
void InitSensorSearch(void);
void CloseSensorSearch(void);
void SensorInitLog(void);
#if 0
int SearchForASensor (struct lg_master *pLgMaster, int16_t startX, int16_t startY,
		      int16_t *foundX, int16_t *foundY);
#endif
void ClearSensorBuffers(void);
uint32_t getSuperFineFactor(void);

extern
int SearchSingleSetOutXY( struct lg_master *pLgMaster
                               , int sweep
                               , int16_t inpX
                               , int16_t inpY
                               , int16_t delX
                               , int16_t delY
                               , int16_t *inpBuf
                               , int steps
                               , int32_t *countX
                               , int32_t *countY
                               , int32_t *sumX
                               , int32_t *sumY
                               )
;

extern
int SearchWithTwoSetsOutX( struct lg_master *pLgMaster
                                , int sweep
                                , int16_t inpX1
                                , int16_t inpY1
                                , int16_t delX1
                                , int16_t delY1
                                , int16_t inpX2
                                , int16_t inpY2
                                , int16_t delX2
                                , int16_t delY2
                                , int16_t *inpBuf1
                                , int16_t *inpBuf2
                                , int steps
                                , int32_t *countX
                                , int32_t *countY
                                , int32_t *sumX
                                , int32_t *sumY
                                )
;

extern
 int SearchWithTwoSetsOutY( struct lg_master *pLgMaster
                                , int sweep
                                , int16_t inpX1
                                , int16_t inpY1
                                , int16_t delX1
                                , int16_t delY1
                                , int16_t inpX2
                                , int16_t inpY2
                                , int16_t delX2
                                , int16_t delY2
                                , int16_t *inpBuf1
                                , int16_t *inpBuf2
                                , int steps
                                , int32_t *countX
                                , int32_t *countY
                                , int32_t *sumX
                                , int32_t *sumY
                                )
;

extern
int SearchWithTwoSetsOutXplus( struct lg_master *pLgMaster
                                , int sweep
                                , int16_t inpX1
                                , int16_t inpY1
                                , int16_t delX1
                                , int16_t delY1
                                , int16_t inpX2
                                , int16_t inpY2
                                , int16_t delX2
                                , int16_t delY2
                                , int16_t *inpBuf1
                                , int16_t *inpBuf2
                                , int steps
                                , int32_t *countX
                                , int32_t *countY
                                , int32_t *sumX
                                , int32_t *sumY
                                )
;

extern
int SearchWithTwoSetsOutYplus( struct lg_master *pLgMaster
                                , int sweep
                                , int16_t inpX1
                                , int16_t inpY1
                                , int16_t delX1
                                , int16_t delY1
                                , int16_t inpX2
                                , int16_t inpY2
                                , int16_t delX2
                                , int16_t delY2
                                , int16_t *inpBuf1
                                , int16_t *inpBuf2
                                , int steps
                                , int32_t *countX
                                , int32_t *countY
                                , int32_t *sumX
                                , int32_t *sumY
                                )
;

extern
int
findFirstLast(struct lg_master *pLgMaster, int16_t *firstX, int16_t *lastX,
              int16_t *firstY, int16_t *lastY, int16_t *foundX, int16_t *foundY,
              int16_t currentX, int16_t currentY, int16_t xStep, int16_t yStep,
              uint16_t nSteps);



void AddTargetReturnInfo(int16_t x, int16_t y, int16_t returnedValue, int sweep); 

extern int gTargetDrift;

extern uint32_t gCoarseSearchStep;
extern int  gSuperFineCount;
extern int  gSuperFineSkip;
extern int  gNoFine;
extern int  gMaxQuickSearches;
extern int  gNumberOfSensorSearchAttempts;

extern int32_t   gLoutCount;
extern int32_t   gLoutSize;
extern char * gLoutBase;
extern char * gLoutPtr;

extern int16_t * gLout;
extern int16_t * gLout1;
extern int16_t * gLout2;

extern uint32_t gSuperIndex;
extern int16_t  *gXsuperSave;
extern int16_t  *gYsuperSave;

extern int gLoutIndex;
extern double   *gSaveAvgX;
extern double   *gSaveAvgY;

extern uint32_t gSuperFineSearchStep;
extern uint32_t gSuperFineFactor;

//
// targetreturns file structures
//

struct targetreturns_file_header {
  uint32_t    format;
  uint32_t    memoryUsed;
  uint32_t    targetReturnsAllTargets;
  uint32_t    targetReturnsPerTarget[MAX_TARGETSFLEX];
} __attribute__ ((packed));

struct targetreturns_file_entry {
  uint32_t      x;
  uint32_t      y;
  uint16_t      returnValue;
  unsigned char sweep;
  unsigned char fill; 
} __attribute__ ((packed));

#endif
