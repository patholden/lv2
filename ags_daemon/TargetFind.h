#include <stdint.h>

#ifndef TARGETFIND_H
#define TARGETFIND_H


#define SENSE_BUF_SIZE          sizeof(int16_t) * 500000
#define DEFAULT_SENSE_DELAY     40

// Specific default values for CoarseScan()
#define COARSE_SCAN_STEP        10
#define FSSC_SCAN_STEP           1
#define COARSE_SCAN_NUM_POINTS   5
#define COARSE_SCAN_MIN_FOUND    5
#define COARSE_SCAN_MAX_LOOPS   70
#define COARSE_SCAN_THRESHOLD1  60
#define COARSE_SCAN_THRESHOLD2  30

// Specific default values for FindSuperScanCoords()
#define FSSC_STEP                1

// Specific default values for SuperScan()
#define SUPER_SCAN_STEP          1

int FindSuperScanCoords(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
			int16_t *superScanX, int16_t *superScanY, uint32_t *numLines, uint32_t *numPoints);
int CoarseScan_drv(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
		   int16_t *foundX, int16_t *foundY);
int CoarseScan(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
		      int16_t *foundX, int16_t *foundY);
int SuperScan(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
	      int16_t *foundX, int16_t *foundY);
int ScanEnd(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
	    int16_t *foundX, int16_t *foundY);
int FindTarget(struct lg_master *pLgMaster, int16_t startX, int16_t startY,
	       int16_t *foundX, int16_t *foundY);

#endif
