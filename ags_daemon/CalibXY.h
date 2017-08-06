#include <stdint.h>
#ifndef CALIBXY_H
#define CALIBXY_H

void CalibXY(struct lg_master *pLgMaster, struct parse_calibxy_parms* parameters,
	     uint32_t respondToWhom);

#endif
