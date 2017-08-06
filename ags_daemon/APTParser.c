//static char rcsid[] = "$Id: APTParser.c,v 1.11 1999/07/29 19:58:10 ags-sw Exp pickle $";

#define kNumberOfSymbolChars 20

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <syslog.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "parse_data.h"
#include "APTParser.h"
#include "AppErrors.h"
#include "LaserPattern.h"
#include "LaserInterface.h"
#include "AppStrListIDs.h"
#include "QuickCheckManager.h"

enum
{
	kInitStatementBufferErr = 1,
	kInitializingAPTParser
};

#define kMaxNumberOfElements	20

#define LineSize   256

typedef struct
{
	uint32_t	type;
	unsigned short	length;
	char			data[LineSize];
} elementRecord;

enum
{
	kSymbol,
	kNumber,
	kIllegal,
	kBlank,
	kDollar,
	kDoubleDollar,
	kSemicolon,
	kColon,
	kRightParenthesis,
	kLeftParenthesis,
	kApostroph,
	kAsterisk,
	kDoubleAsterisk,
	kPlus,
	kMinus,
	kSlant,
	kEqual,
	kComma,
	kQuote,
	kBar
};

static uint32_t	GetNextElement(char *theData, short dataLength, 
			       elementRecord *theElement, short *theResult );
static	int32_t	GetNextRecord(char *theData, int32_t dataLength);
static	uint32_t ProcessStatement(struct lg_master *pLgMaster, short numberOfElements);
static	uint32_t	FirstFourLetters ( elementRecord *theElement );
static	unsigned short	LastTwoLetters ( elementRecord *theElement );
					
static	elementRecord	*gElements = 0;

uint32_t ProcessPatternData (struct lg_master *pLgMaster, char *theData, uint32_t lengthOfData )
{
  int32_t	lengthOfRecord, remainingInput;
  short		processedLength, numberOfElements;
  char          *currentStartOfRecord;
  char          *currentStartOfData;
  uint32_t	theError;
  short			elementLength;
#ifndef __unix__
  int32_t                       theLastCheck, tempLong;
#endif
  if (gRealTimeTransform)
    {
      gQClengthOfData = lengthOfData;
      memcpy( gQCsaveData, theData, lengthOfData );
    }

  remainingInput = lengthOfData;
  currentStartOfData = theData;
  numberOfElements = 0;
  while (remainingInput > 0)
    {
      lengthOfRecord = GetNextRecord ( currentStartOfData, remainingInput );
      currentStartOfRecord = currentStartOfData;
      remainingInput -= lengthOfRecord;
      currentStartOfData += lengthOfRecord;

      if (lengthOfRecord > LineSize)
	return(kAPTError + 1);

      processedLength = 0;
#ifndef __unix__
      theLastCheck = 0;
#endif
      while (processedLength < lengthOfRecord)
	{
#ifndef __unix__
	  tempLong = TickCount();
	  if ((tempLong - theLastCheck) > 90L)
	    theLastCheck = tempLong;
#endif
	  theError = GetNextElement(currentStartOfRecord, 
				    ((int16_t)lengthOfRecord - processedLength),
				    &gElements[numberOfElements], &elementLength);
	  if (theError != 0)
	    syslog(LOG_ERR,"\nProcessPatternData : GetNextElement() : theError %d\n", theError);

	  if (theError)
	    return(theError);
	  processedLength += elementLength;
	  currentStartOfRecord += elementLength;
	  switch (gElements[numberOfElements++].type)
	    {
	    case kBlank:
	    case kIllegal:
	    case kDollar:
	    case kDoubleDollar:
	      numberOfElements--;
	      break;
	    case kNumber:
	      if ((numberOfElements > 1) && (gElements[numberOfElements - 2].type == kMinus))
		{
		  gElements[numberOfElements - 2].data[0] = '-';
		  memmove ((char *)&gElements[numberOfElements - 2].data[1],
			   (char *)gElements[numberOfElements - 1].data,
			   (size_t)gElements[numberOfElements - 1].length);
		  gElements[numberOfElements - 2].length =
		    1 + gElements[numberOfElements - 1].length;
		  gElements[numberOfElements - 2].data[gElements[numberOfElements - 2].length] = '\0';
		  gElements[numberOfElements - 2].type = kNumber;
		  numberOfElements--;
		}
	      else
		gElements[numberOfElements - 1].data[gElements[numberOfElements - 1].length] = '\0';
	      break;
	    }
	  if ( numberOfElements > kMaxNumberOfElements )
	    return(kAPTError + 1005);
	}
      theError = ProcessStatement (pLgMaster, numberOfElements );
      if (theError != 0)
	syslog(LOG_ERR,
		"\nProcessPatternData : Call ProcessStatement() theError %d\n",
		theError);
      if (theError == (kAPTError + 1133))
	return(0);
      if (theError)
	return(theError);

      numberOfElements = 0;
    }
  return(0);
}

static int32_t GetNextRecord (char *theData, int32_t dataLength )
{
	register int32_t i;
	register char *charIn;
	register char theChar;

	charIn = theData;
        i = 0L;
	while ( i < dataLength )
	{
		theChar = *charIn;
		if ( theChar == '\x0D' ) break;
		if ( theChar == '\x0A' ) break;
                charIn++;  i++;
	} 
	if ( ( i != dataLength ) &&
	     ( ( *charIn == '\x0D' ) || ( *charIn == '\x0A' ) ) ) i++;
	return i;
}


uint32_t GetNextElement (char *theData, short dataLength, 
	elementRecord *theElement, short *theResult )
{
	char *currentChar;
	short mantissaPoint, exponentED, exponentSign;
	short mantissaDigits, exponentDigits;
	unsigned char wasNonDollarAfterDollar;

	wasNonDollarAfterDollar = false;
	theElement->type = kBlank;
	theElement->length = 0;
	for  (
			*theResult = 0, currentChar = theData;
			*theResult < dataLength;
			++*theResult, ++currentChar
		)
	{
		if ( theElement->type == kDollar )
		{
			if ( wasNonDollarAfterDollar ) continue;
			wasNonDollarAfterDollar = true;
			if ( *currentChar == '$' ) theElement->type = kDoubleDollar;
			continue;
		}

		if ( theElement->type == kDoubleDollar ) continue;
		
		if ( ( theElement->type == kSymbol ) &&
			( theElement->length >= 6 ) &&
			( !strncmp ( theElement->data, "PPRINT", 6 ) ) ) 
			continue;

		if ( ( ' ' == *currentChar ) || ( '\t' == *currentChar ) )
		{
			if ( theElement->type == kNumber ) return 0;
			if ( theElement->type == kSymbol ) return 0;
			continue;
		}

		if ( ( '0' <= *currentChar ) && ( *currentChar <= '9' ) )
		{
			switch ( theElement->type )
			{
				case kBlank:
					mantissaPoint = 0;
					mantissaDigits = 1;
					exponentED = 0;
					exponentSign = 0;
					exponentDigits = 0;
					theElement->type = kNumber;
					theElement->length = 1;
					theElement->data[0] = *currentChar;
					continue;
				case kNumber:
					if ( exponentED )
					{ 
						if ( exponentDigits >= 2 )
							return ( kAPTError + 1004 );
						else exponentDigits++;
					}
					else mantissaDigits++;
					
					theElement->data[theElement->length++] =
						*currentChar;
					continue;
				case kSymbol:
					if ( theElement->length >= kNumberOfSymbolChars )
						return ( kAPTError + 1012 );
					theElement->data[theElement->length++] =
						*currentChar;
					continue;
				default:
					return 0;
			}
		}

		if ( ( 'A' <= *currentChar ) && ( *currentChar <= 'Z' ) )
		{
			switch ( theElement->type )
			{
				case kBlank:
					theElement->type = kSymbol;
					theElement->length = 1;
					theElement->data[0] = *currentChar;
					continue;
				case kSymbol:
					if ( theElement->length >= kNumberOfSymbolChars )
						return ( kAPTError + 1012 );
					theElement->data[theElement->length++] =
						*currentChar;
					continue;
				case kNumber:
					if ( exponentED ) return ( kAPTError + 1004 );
					if ( ( *currentChar == 'D' ) ||
						( *currentChar == 'E' ) )
					{
						exponentED = 1;
						theElement->data[theElement->length++] = 'e';
						continue;
					}
/*					if ( mantissaPoint ) return ( kAPTError + 1004 );*/
					theElement->type = kSymbol;
					if ( theElement->length >= kNumberOfSymbolChars )
						return ( kAPTError + 1012 );
					theElement->data[theElement->length++] =
						*currentChar;
					continue;
				default:
					return 0;
			}
		}
		
			
		if ( theElement->type == kNumber )
		{
			if ( ( ( *currentChar == '-' ) || ( *currentChar == '+' ) )
				&& exponentED && !exponentSign && !exponentDigits )
			{
				exponentSign = 1;
				theElement->data[theElement->length++] =
					*currentChar;
				continue;
			}
			if ( ( *currentChar == '.' ) && 
				!mantissaPoint && !exponentED )
			{
				mantissaPoint = 1;
				theElement->data[theElement->length++] =
					*currentChar;
				continue;
			}
			if ( mantissaPoint && !mantissaDigits &&
				!exponentED && !exponentSign && !exponentDigits )
			{
				/* Added to accomodate wild and lawless points */
				theElement->type = kIllegal;
				continue;
			}
			if ( !mantissaDigits ||
				( exponentED && !( mantissaPoint && exponentDigits ) ) ||
				( exponentDigits > 2 ) )
				return ( kAPTError + 1004 );
			else return 0;
		}

		if ( theElement->type == kSymbol )
		{
			if ( theElement->length > kNumberOfSymbolChars )
				return ( kAPTError + 1012 );
			else return 0;
		}
		
		if ( theElement->type != kBlank ) return 0;
		
		switch ( *currentChar )
		{
			case '$':
				theElement->type = kDollar;
				break;
			case '+':
				theElement->type = kPlus;
				break;
			case '-':
				theElement->type = kMinus;
				break;
			case '.':
				mantissaPoint = 1;
				mantissaDigits = 0;
				exponentED = 0;
				exponentSign = 0;
				exponentDigits = 0;
				theElement->type = kNumber;
				theElement->length = 1;
				theElement->data[0] = '.';
				break;
			case '/':
				theElement->type = kSlant;
				break;
			case ',':
				theElement->type = kComma;
				break;
			default:
				theElement->type = kIllegal;
				break;
		}
		continue;
	}
	return 0;
}


#define k__		0x0000
#define k____	0x00000000U
#define	kPEND	(	( (uint32_t)'P' << 24 ) + \
					( (uint32_t)'E' << 16 ) + \
					( (uint32_t)'N' << 8 ) + (uint32_t)'D' )
#define	kWN		( ( (unsigned short)'W' << 8 ) + (unsigned short)'N' )
#define	kPENU	(	( (uint32_t)'P' << 24 ) + \
					( (uint32_t)'E' << 16 ) + \
					( (uint32_t)'N' << 8 ) + (uint32_t)'U' )
#define	kP_		( (unsigned short)'P' << 8 )
#define	kZSUR	(	( (uint32_t)'Z' << 24 ) + \
					( (uint32_t)'S' << 16 ) + \
					( (uint32_t)'U' << 8 ) + (uint32_t)'R' )
#define	kF_		( (unsigned short)'F' << 8 )
#define	kGOTO	(	( (uint32_t)'G' << 24 ) + \
					( (uint32_t)'O' << 16 ) + \
					( (uint32_t)'T' << 8 ) + (uint32_t)'O' )

uint32_t ProcessStatement (struct lg_master *pLgMaster, short numberOfElements )
{
	elementRecord *startingElement;
	short numberOfTrueElements;
	double x, y, z;

	numberOfTrueElements = numberOfElements;
	startingElement = gElements;
	
	if ( numberOfTrueElements == 0 ) return 0;

	if ( startingElement[0].type != kSymbol )
		return ( kAPTError + 1028 );
	
	switch ( FirstFourLetters ( &startingElement[0] ) )
	  {
	  case kPEND:
	    if (LastTwoLetters(&startingElement[0]) == kWN)
	      {
		SetPenDown();
		return(0);
	      }
	    break;
	  case kPENU:
	    if (LastTwoLetters(&startingElement[0]) == kP_)
	      {
		SetPenUp();
		return(0);
	      }
	    break;
	  case kZSUR:
	    if (LastTwoLetters(&startingElement[0]) == kF_)
	      {
		if ((numberOfTrueElements < 3) ||
		    (startingElement[1].type != kSlant))
		  return(kAPTError + 1123);
		startingElement = &startingElement[2];
		switch(startingElement->type)
		  {
		  case kSymbol:
		    return 0;
		  case kNumber:
		    if (sscanf(startingElement->data, "%le", &z) != 1)
		      return(kAPTError + 1123);
		    break;
		  default:
		    return(kAPTError + 1123);
		  }
		FromAPTtoInch(&z);
		SetDefaultZ(z);
		return(0);
	      }
	    break;
	  case kGOTO:
	    if (LastTwoLetters(&startingElement[0]) == k__)
	      {
		if (numberOfTrueElements < 3)
		  return(kAPTError + 1133);
		startingElement = &startingElement[1];
		numberOfTrueElements--;
		if (startingElement->type == kSlant)
		  {
		    startingElement = &startingElement[1];
		    numberOfTrueElements--;
		  }
		switch(startingElement->type)
		  {
		  case kSymbol:
		    return 0;
		  case kNumber:
		    if (--numberOfTrueElements == 0)
		      return(kAPTError + 1133);
		    if (sscanf(startingElement->data, "%le", &x) != 1)
		      return(kAPTError + 1004);
		    startingElement++;
		    break;
		  default:
		    return(kAPTError + 1133);
		  }
		switch(startingElement->type)
		  {
		  case kSymbol:
		    return 0;
		  case kNumber:
		    if (sscanf(startingElement->data, "%le", &y) != 1)
		      return(kAPTError + 1004);
		    if (--numberOfTrueElements == 0)
		      {
			FromAPTtoInch ( &x );
			FromAPTtoInch ( &y );
			return(PutGoTo2D(pLgMaster, x, y));
		      }
		    startingElement++;
		    break;
		  case kComma:
		    if (--numberOfTrueElements == 0)
		      return(kAPTError + 1133);
		    startingElement++;
		    switch(startingElement->type)
		      {
		      case kSymbol:
			return 0;
		      case kNumber:
			if (sscanf(startingElement->data, "%le", &y) != 1)
			  return(kAPTError + 1004);
			if (--numberOfTrueElements == 0)
			  {
			    FromAPTtoInch(&x);
			    FromAPTtoInch(&y);
			    return(PutGoTo2D(pLgMaster, x, y));
			  }
			startingElement++;
			break;
		      default:
			return(kAPTError + 1133);
		      }
		    break;
		  default:
		    return(kAPTError + 1133);
		  }
		switch(startingElement->type)
		  {
		  case kSymbol:
		    return 0;
		  case kNumber:
		    if (sscanf(startingElement->data, "%le", &z) != 1)
		      return(kAPTError + 1004);
		    FromAPTtoInch(&x);
		    FromAPTtoInch(&y);
		    FromAPTtoInch(&z);
		    return(PutGoTo3D(pLgMaster, x, y, z));
		  case kComma:
		    if (--numberOfTrueElements == 0)
		      return( kAPTError + 1133);
		    startingElement++;
		    switch(startingElement->type)
		      {
		      case kSymbol:
			return 0;
		      case kNumber:
			if (sscanf(startingElement->data, "%le", &z) != 1)
			  return(kAPTError + 1004);
			FromAPTtoInch(&x);
			FromAPTtoInch(&y);
			FromAPTtoInch(&z);
			return(PutGoTo3D(pLgMaster, x, y, z));
		      default:
			return(kAPTError + 1133);
		      }
		    break;
		  default:
		    return(kAPTError + 1133);
		  }
	      }
	    break;
	  default:
	    break;
	  }
	return 0;
}


uint32_t FirstFourLetters ( elementRecord *theElement )
{
	if ( ( theElement->type != kSymbol ) || ( theElement->length == 0 ) )
		return k____;
	switch ( theElement->length )
	{
		case 1:
			return ( (uint32_t)theElement->data[0] << 24 );
		case 2:
			return (
				( (uint32_t)theElement->data[0] << 24 ) +
				( (uint32_t)theElement->data[1] << 16 ) );
		case 3:
			return (
				( (uint32_t)theElement->data[0] << 24 ) +
				( (uint32_t)theElement->data[1] << 16 ) + 
				( (uint32_t)theElement->data[2] << 8 ) );
		default:
			return (
				( (uint32_t)theElement->data[0] << 24 ) +
				( (uint32_t)theElement->data[1] << 16 ) + 
				( (uint32_t)theElement->data[2] << 8 ) + 
				(uint32_t)theElement->data[3] );
	}
}

unsigned short LastTwoLetters ( elementRecord *theElement )
{
	if ( ( theElement->type != kSymbol ) || ( theElement->length < 5 ) )
		return k__;
	if ( theElement->length == 5)
		return ( (unsigned short)theElement->data[4] << 8 );
	return ( ( (unsigned short)theElement->data[4] << 8 ) + 
		(unsigned short)theElement->data[5] );
}

void CloseAPTParser ( void )
{
	if ( gElements ) free((void *)gElements );
	gElements = 0;
}

void InitAPTParser ( void )
{
	gElements = (elementRecord *)malloc( (size_t)
		( sizeof ( elementRecord ) * kMaxNumberOfElements ) );
}
