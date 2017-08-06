#ifndef DOCOARSESCAN_H
#define DOCOARSESCAN_H

int DoCoarseScan(struct lg_master *pLgMaster, int16_t dX, int16_t dY,
		 int16_t lsstep, uint16_t lscount,
		 int16_t *xfound, int16_t *yfound);
#endif
