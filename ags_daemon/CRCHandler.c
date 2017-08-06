//static char rcsid[] = "$Id: CRCHandler.c,v 1.3 2001/01/03 17:49:18 ags-sw Exp pickle $";

	/********************************/
	/*	Platform independent code	*/
	/*	Use on both computers		*/
	/********************************/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include "CRCHandler.h"

static	unsigned short		*g256Lookup16bitWords = (unsigned short *)0; 
static	void				SetupLookupTable ( void );

#define kCCITTPolynomial	(unsigned short)0x1021

#define BAD_CRC 0

void AppendCRC(unsigned char *theBuffer, uint32_t lengthOfBufferWithoutCRC)
{
	/* msb == most significant byte */
	/* lsb == least significant byte */
	unsigned char	*theCurrentByte, *msbCRC, *lsbCRC;
	unsigned short	the16bitValueFromLookupTable;
	uint32_t        i;
	
	theCurrentByte	= (unsigned char *)theBuffer;
	msbCRC = (unsigned char *)&theBuffer[lengthOfBufferWithoutCRC];
	lsbCRC = (unsigned char *)&theBuffer[lengthOfBufferWithoutCRC + 1];
	*msbCRC = (unsigned char)'\0';
	*lsbCRC = (unsigned char)'\0';
	i = lengthOfBufferWithoutCRC;
	while ( i-- )
	{
		the16bitValueFromLookupTable =
			g256Lookup16bitWords [ *msbCRC ^ *(theCurrentByte++) ];
		*msbCRC = *lsbCRC ^
			(unsigned char)( the16bitValueFromLookupTable >> 8 );
		*lsbCRC =
			(unsigned char)( the16bitValueFromLookupTable & 0x00FF );
	}
	return;
}


short CheckCRC(unsigned char *theBuffer, uint32_t lengthOfBufferWithoutCRC)
{
	unsigned short	theCRC;
	unsigned char	*theCurrentByte;
	uint32_t        i;

	theCRC = 0;
	theCurrentByte = theBuffer;
	i = lengthOfBufferWithoutCRC + 2;
	
	while (i--)
	  theCRC = (theCRC << 8) ^
		g256Lookup16bitWords[(theCRC >> 8) ^ *(theCurrentByte++)];
	
	if (theCRC)
	  {
	    syslog(LOG_ERR,"Command CRC failure, result %x",theCRC);
	    return kCRC_Bad;
	}
	return(kCRC_OK);
}


void SetupLookupTable ( void )
{
	unsigned short	tableIndex, j, input;
	unsigned short	*theCurrent16bitWord;

	theCurrent16bitWord = g256Lookup16bitWords;
	tableIndex = 0;
	while ( tableIndex < 256 )
	{
		input = tableIndex++ << 8;
		*theCurrent16bitWord = 0;
		j = 8;
		while ( j-- )
		{
			if ( ( input ^ *theCurrent16bitWord ) & 0x8000 )
				*theCurrent16bitWord =
					( *theCurrent16bitWord << 1 ) ^ kCCITTPolynomial;
			else
				*theCurrent16bitWord <<= 1;

			input <<= 1;
		}
		theCurrent16bitWord++;
	}
	return;
}

	/********************************************/
	/* 		Macintosh specific code				*/
	/* 		Do not use this code on IBM			*/
	/********************************************/

#include "AppCommon.h"
#include "AppErrors.h"
#include "AppStrListIDs.h"

enum
{
	kInitCRCTableErr = 1,
	kInitializingCRCMsg
};


void CloseCRCHandler ( void )
{
    if ( g256Lookup16bitWords )
      free ( (void *)g256Lookup16bitWords );
    g256Lookup16bitWords = 0;
}

void InitCRCHandler ( void )
{
	
    g256Lookup16bitWords = (unsigned short *)calloc( (size_t)256, sizeof ( unsigned short ) );
	
    SetupLookupTable();
}
