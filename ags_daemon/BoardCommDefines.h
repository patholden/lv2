/*   $Id: BoardCommDefines.h,v 1.2 1996/12/25 18:38:04 ags-sw Exp $  */
#ifndef BOARDCOMMDEF_H
#define BOARDCOMMDEF_H

#define _NEW_PATH_			1

#define	kPackAChannelAControl	0x00700009U
#define	kPackBChannelAControl	0x00780009U
#define	kLatchBit				'\x04'

#define	Digital48Type	0x24

#define kOnBoardMessageBase		0x00000500U

#define	kGSstatus				kOnBoardMessageBase

#define kMacToGSMessageFlag		( kOnBoardMessageBase + 4 )
#define kMacToGSMessage			( kOnBoardMessageBase + 8 )

#define kGStoMacMessageFlag		( kOnBoardMessageBase + 12 )
#define kGStoMacMessage			( kOnBoardMessageBase + 16 )

#define	kGSAnglesIn				( kOnBoardMessageBase + 20 )
#define	kGSAnglesOut			( kOnBoardMessageBase + 24 )

#define	kGSPattern				( kOnBoardMessageBase + 28 )

#define kRefreshRateMicroSec	( kOnBoardMessageBase + 32 )

#define kPlyIndex				( kOnBoardMessageBase + 40 )
#define kSensorIndex			( kOnBoardMessageBase + 42 )

#define kPatternBuffSize		( kOnBoardMessageBase + 44 )

#define	kDoSegmentPattern		( kOnBoardMessageBase + 48 )
#define	kSegmentStart			( kOnBoardMessageBase + 50 )
#define	kSegmentLength			( kOnBoardMessageBase + 54 )

#define kDebug					( kOnBoardMessageBase + 58 )

#define kNextPointFlag			0x00000600U
/* These two bytes are sacred. Do not redefine them.	*/
/* They are used in SpringBoardInterrupt.a,				*/
/* which does not refer to this file					*/

#define kPostingMessage		0x00000001U << 0
#define kMessagePosted		0x00000001U << 1
#define kReadingMessage		0x00000001U << 2
#define kMessageRead		0x00000001U << 3

#define	kGSOffLine			0x00000000U
#define	kGSInitializing		0x00000001U
#define	kGSOnLine			0x00000002U
#define	kGSError			0x00000003U


#define kMaxNumberOfPatternPoints           160000
#define kGSReadTimeout				60

enum
{
	kBadOpenIn = 1,
	kBadOpenOut,
	kNoDigital48Type,
	kAnglesOutMemInit,
	kAnglesInMemInit,
	kPatternMemInit,
	kNoDualSerialType,
	kNoSerialAndDigital48Type
};

enum
{
	kNoSegment = 0,
	kNewSegment,
	kDoSegment
};
#endif
