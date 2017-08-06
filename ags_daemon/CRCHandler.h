/*   $Id: CRCHandler.h,v 1.2 1996/12/25 18:38:04 ags-sw Exp $  */
#ifndef CRCHANDLE_H
#define CRCHANDLE_H

#define	kCRC_OK		0
#define	kCRC_Bad	1

void AppendCRC(unsigned char *theBuffer, uint32_t lengthOfBufferWithoutCRC);
short CheckCRC(unsigned char *theBuffer, uint32_t lengthOfBufferWithoutCRC);

/* These two functions are platform dependent */
void InitCRCHandler(void);
void CloseCRCHandler(void);
#endif
