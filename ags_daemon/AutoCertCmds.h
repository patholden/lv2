/*   $Id: AutoCertCmds.h,v 1.3 1997/05/11 23:20:53 ags-sw Exp $  */
#ifndef AUTOCERTCMD_H
#define AUTOCERTCMD_H

#define kNoResult				0x00000000UL
uint32_t CheckSearch(void);
uint32_t CheckStop(void);
uint32_t PostSearch(void);
uint32_t PostStop(void);
void InitAutoCertCmds(void);
void AutoCertResponse(char *theResponse);
#endif
