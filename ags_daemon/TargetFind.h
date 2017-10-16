#include <stdint.h>

#ifndef TARGETFIND_H
#define TARGETFIND_H


#define SENSE_BUF_SIZE          sizeof(int16_t) * 500000
#define DEFAULT_SENSE_DELAY     25

// Specific default values for CoarseScan()
#define COARSE_SCAN_STEP        10
#define COARSE_SCAN_NUM_POINTS   5
#define COARSE_SCAN_MIN_FOUND    3
#define COARSE_SCAN_MAX_LOOPS   50
#define CS_SENSE_THRESHOLD    0x30
#define CS_SENSE_HITS            3

// Specific default values for FindSuperScanCoords()
#define FSSC_SCAN_STEP           1
#define FSSC_SCAN_NUMPOINTS      0xFFFF             // Max steps in grid
// Specific default values for SuperScan()
#define SUPER_SCAN_STEP          1
#define SS_SENSE_HITS            5
#define SS_SENSE_THRESHOLD    0x30

int isOutOfBounds(int16_t point, uint16_t step, uint32_t count);
int CoarseScanFindMatch(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData, int16_t *foundX, int16_t *foundY);
int CoarseScan(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
	       int16_t *foundX, int16_t *foundY);
int FindSuperScanCoords(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
			int16_t *superScanX, int16_t *superScanY, uint32_t *numLines, uint32_t *numPoints);
int SuperScan(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
	      uint32_t numLines, uint32_t numPoints, int16_t *foundX, int16_t *foundY);
int FindTarget(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
	       int16_t *foundX, int16_t *foundY);

#endif
