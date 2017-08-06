/*   $Id: APTParser.h,v 1.3 1999/07/29 19:57:44 ags-sw Exp $  */
#ifndef APTPARSER_H
#define APTPARSER_H

void InitAPTParser(void);
void CloseAPTParser(void);
uint32_t ProcessPatternData(struct lg_master *pLgMaster, char *theData,	uint32_t lengthOfData);
#endif
