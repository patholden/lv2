#include <stdint.h>
#ifndef DOTAKEPICTURE_H
#define DOTAKEPICTURE_H

extern
void DoTakePicture ( struct lg_master *pLgMaster,
		     struct parse_takepic_parms* parameters,
		     uint32_t respondToWhom);

#endif
