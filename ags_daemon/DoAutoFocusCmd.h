#include <stdint.h>
#ifndef DOAUTOFOCUSCMD_H
#define DOAUTOFOCUSCMD_H

void DoAutoFocusCmd(struct lg_master *pLgMaster, unsigned char *buffer);
int  InitAutoFocus(void);
void FreeUpAFBuffs(void);
#endif
