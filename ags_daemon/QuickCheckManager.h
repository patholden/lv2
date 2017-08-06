#include <stdint.h>
/*  $Id$  */

#ifndef QUICKCHECKMANAGER_H
#define QUICKCHECKMANAGER_H

#include "L3DTransform.h"
#include "Protocol.h"

void PerformAndSendQuickCheck(struct lg_master *pLgMaster, unsigned char *pAngles, uint32_t nTargets);
extern void PerformThresholdQuickCheck(struct lg_master *pLgMaster, char * data,
                                       uint32_t nTargets, uint32_t nThresh);
extern uint32_t SaveAnglesForQuickCheck (struct displayData *data,
                              uint32_t respondToWhom );

extern int PerformPeriodicQuickCheck(struct lg_master *pLgMaster);

extern void PerformAndSendQuickies(struct lg_master *pLgMaster, int32_t StopOrGo,
				    char * data, uint32_t respondToWhom );
void DoQCcount(struct lg_master *pLgMaster, char * data, uint32_t respondToWhom);
void DoQCtimer (struct lg_master *pLgMaster, char * data, uint32_t respondToWhom);

extern int16_t gCurrentQCSensor;
extern int16_t  gCurrentQCSet;

extern int32_t gQuickCheck;

extern int32_t gQCtimer;

extern int gRealTimeTransform;

void DummyQuickCheck ( void );

int ShouldDoQuickCheck ( void );

extern doubleInputPoint gSavePt[11000];

extern char * gQCsaveData;

extern uint32_t gQClengthOfData;

extern uint32_t       gQuickCheckAngles[ kMaxNumberOfPlies * kNumberOfFlexPoints * 2 ];

extern int32_t       gQuickCheckTargetNumber[ kMaxNumberOfPlies ];

extern int32_t gQuickFailNumber;

#endif
