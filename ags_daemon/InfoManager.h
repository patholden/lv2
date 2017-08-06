/*   $Id: InfoManager.h,v 1.3 1997/05/11 23:19:56 ags-sw Exp $  */
#ifndef INFOMGR_H
#define INFOMGR_H

void InformCommand(uint32_t theCommand);
void InformParameters(uint32_t theCommand, char *theParamBuffer);
void InformSerialNumber(uint32_t theNumber);
#endif
