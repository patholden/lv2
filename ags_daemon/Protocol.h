/*   $Id: Protocol.h,v 1.25 2003/04/25 10:32:11 ags-sw Exp ags-sw $ */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#define kSizeOldLongDouble 12

#define	kMessageMask				0xFF000000U
#define	kSerialNumberMask			0x00FFFFU
/* Used internally to check for kQuickCheckSensor
and to create (kQuickCheckSensor + kQuickCheckSensorUnit * sensorToCheck) */
#define	kQuickCheckSensorMask		0xF0000000U
#define	kQuickCheckSensorUnit		0x01000000U

#define	kNoCommand				0x01
#define	kStop					0x02
#define	kGoAngle				0x03
#define	kDoFullReg				0x04
#define	kDisplay				0x05
#define	kQuickCheck				0x06
#define	kDisplayNoQuickCheck			0x07
#define kGetDacs				0x08
#define	kDisplayChunksStart			0x09
#define	kDisplayChunksData			0x0A
#define	kDisplayChunksDo			0x0B
#define	kDisplaySeveral				0x0C
#define	kGimme3D				0x0D
#define	kDisplayVideoCheck			0x0E
#define	kSegmentDisplay				0x0F
#define	kDisplayRawChunksDo			0x10
#define kDisplayNoResponse                      0x15
#define	kTakePicture				0x30
#define	kCheckForEdge				0x31
#define	kMultipleFOD				0x32
#define	kDarkAngle				0x34
#define	kROI					0x35
#define	kAutoCert				0x36
#define	kFileTransfer				0x37
#define	kTFS					0x38
#define	kCheckOrient 				0x3A 
#define	kFOM					0x40
#define	kRightON				0x41
#define	ksuperFOM				0x42
#define	kDoubleToTransform			0x43
#define	kQuickies         			0x44
#define	kReInit           			0x45
#define	kQuickStop        			0x46
#define	kFracCert				0x50
#define	kRightOnCert				0x51
#define	kRightOnReg 				0x52
#define kShowTargets                     	0x53
#define	kDisplayNewVideo			0x54
#define	kGetNewVideo    			0x55
#define	kFindOneTarget  			0x56
#define	kCalculateTransform			0x57
#define	kDimAngle				0x58
#define	kFileGetStart				0x5A
#define	kFileGetData				0x5B
#define	kFilePutStart				0x5C
#define	kFilePutData				0x5D
#define	kFilePutDone				0x5E
#define kGeoQuickies				0x60
#define kDisplayKitVideo			0x61
#define kRegWithFeedback                        0x62
#define kDisplayChunksXYZ                       0x63
#define kAutoFocusCmd				0x64
#define kQuickCheckCount			0x66
#define kQuickCheckTimer			0x67
#define kHobbsGet        			0x68
#define kHobbsSet        			0x69
#define kSetBit          			0x71
#define kEtherAngle                             0x7A
#define kSuperLevelScan        			0x7B
#define kFlexDisplay                            0x82
#define kFlexQuickCheck                         0x83
#define kFlexDisplayChunksDo                    0x84
#define kFlexCalculateTransform                 0x86
#define kFlexRegWithFeedback                    0x87
#define kFlexShowTargets                        0x88
#define kCheckFOD                               0x8A
#define kRefreshRate                            0x8B
#define kThresholdQuickCheck                    0x8E
#define kFlexCalWithFeedback                    0x92
#define kChangeDisplayPeriod                    0x94
#define kChangeTransformTolerance               0x95
#define kGetTargetsUsed                         0x96
#define kCalibrateXY                            0x97
#define kMessageLength                          0xA4


/* The following three messages are used only internally by Mac
for diagnostic purposes. Currently they would not be understood if sent to Mac
from outside. However, the ability to receive these messages from IBM and
properly interpret them can easily be added */
#define	kSearchForASensor			0x80
#define	kQuickCheckSensor			0xD0

/* The following message is handled in a VERY non-standard way */
/* It is used to support HAL CELL communications */
#define	k_HAL_CELL_Message			0x40000000U


#define	kUnrecognizedCommandMsg			0xF0
#define	kParameterTimeoutMsg			0xF1
#define	kDataTimeoutMsg				0xF2
#define	kBadDataLengthMsg			0xF3
#define	kCRC16NoMatchMsg			0xF4
#define	kLocalModeMsg				0xF5
#define	kNetworkModeMsg				0xF6
#define	kOtherErrorMsg				0xFF

#define kNetworkModeNotSetMsg			0xA1000000U
#define kAEWrongLengthMsg			0xA2000000U
#define	kAEMissingCommandMsg			0xA3000000U
#define kHandlingAnotherAEMsg			0xA4000000U
#define	kBadSourseAEMsg				0xA5000000U
#define	kOK					0xE0000000U
#define	kQuickCheckOK				0xE1000000U
#define	kStopOK					0xE2000000U
#define	kQuickCheckPlyOK			0xE3000000U
#define	kItsGone				0xE4000000U
#define kItsHere                                0xE5000000U
#define kFlexFail                               0xE7000000U
#define	kFail					0xE8000000U
#define	kQuickCheckFail				0xE9000000U
#define	kStopFail				0xEA000000U
#define	kQuickCheckPlyFail			0xEB000000U
#define	kItsGoneNOT				0xEC000000U
#define	kSensorNotFound			        0x00010000U
#define	kInputAngleOutOfRange		        0x00020000U
#define	kPatternAngleOutOfRange		        0x00030000U
#define	kFirstSensor				( 0x00000001U << 0 )
#define	kSecondSensor				( 0x00000001U << 1 )
#define	kThirdSensor				( 0x00000001U << 2 )
#define	kFourthSensor				( 0x00000001U << 3 )
#define	kFifthSensor				( 0x00000001U << 4 )
#define	kSixthSensor				( 0x00000001U << 5 )
#define	kXTooSmall				0x1
#define	kXTooLarge				0x2
#define	kYTooSmall				0x4
#define	kYTooLarge				0x8
#define	kAPTError				0x00040000U
#define	kBoardError				0x00050000U
#define	kBoardInitializing			0x00000001U
#define	kBoardTimeout				0x00000002U
#define	kUnacceptableCommand		 	0x00000003U
#define	kTooManyPatternPoints			0x00000004U
#define	kLocalMode				0x00000005U
#define	kDataTooLarge				0x00000006U
#define	kTooManyAPTVectors			0x00000007U
#define	kTooManyPlies				0x00000008U
#define kRemoteMode				0x00000009U
#define kAEMode					0x0000000AU
#define kNetworkModeNotSet			0x0000000BU
#define	kNoIntersection				0x0000000CU
#define	kNoVideo				0x0000000DU
#define	kOtherError				0x0000FFFFU
#define	kMaxDataLength				2000000
#define	kMaxNumberOfAPTVectors			32000
#define kMaxTargets				50
#define	kParameterTimeoutLength			300 /* In ticks */
#define	kDataTimeoutLength			300 /* In ticks */
#define	kWriteTimeoutLength			300 /* In ticks */
#define	kNumberOfRegPoints			6
#define kNumberOfFlexPoints                     24
#define kFeedbackNumber                         8
#define	kCRCSize				2
#define	kMaxNumberOfPlies			12

/*	k###Sensors are added in any combination to kFail or kQuickCheckFail 
	with either kSensorNotFound or kInputAngleOutOfRange added.
	If a k###Sensor is set -- it means something is not OK with it.
	
	kXTooSmallLarge and kYTooSmallLarge are added in any combination
	to kFail with either kInputAngleOutOfRange or
	kPatternAngleOutOfRange added. In case of kInputAngleOutOfRange
	kXYTooSmallLarge can coexist with k###Sensors.
	
	kMaxDataLength is in bytes.
	
	kAPTError is followed by a 2 byte APT error code as
	specified in the standard.
	
	Some stuff would never be seen by IBM, for example kNoCommand,
	kLocalMode, or kQuickCheckOK, they are used by Mac internally.
*/

#ifdef __unix__
#define kLongDoublePadding 0
#else
#define kLongDoublePadding 0
#endif


#define kSizeOfCommand				sizeof ( uint32_t )

#define	kSizeOfGoAngleParameters		2 * sizeof ( double )

#define kSizeOfEtherAngleParameters             2 * sizeof ( double )

#define	kSizeOfDarkAngleParameters		2 * sizeof ( double )

#define	kSizeOfDimAngleParameters					\
        ( 2 * sizeof ( double )  +  2 * sizeof ( int32_t ) )

#define	kSizeOfSegmentDisplayParameters		2 * sizeof ( double )

#define	kSizeOfFindOneTargetParameters		2 * sizeof ( double )

#define	kSizeOfDoFullRegParameters					\
	( kNumberOfRegPoints * 3 * sizeof ( double ) +			\
	kNumberOfRegPoints * 2 * sizeof ( double ) )

#define	kSizeOfCalculateTransformParameters				\
	( kNumberOfRegPoints * 3 * sizeof ( double ) +			\
	kNumberOfRegPoints * 2 * sizeof ( double ) )

#define	kSizeOfRightOnRegParameters					\
	(  sizeof(int32_t) +                                               \
        kNumberOfRegPoints * 3 * sizeof ( double ) +			\
	kNumberOfRegPoints * 2 * sizeof ( double ) +                    \
        kNumberOfRegPoints * 2 * sizeof ( uint32_t )               \
        )

#define kSizeOfRegWithFeedbackParameters                           \
        (  sizeof(int32_t)                                            \
        +  sizeof(int32_t)                                            \
        +  kFeedbackNumber * 3 * sizeof ( double )                 \
        +  kFeedbackNumber * 2 * sizeof ( double )                 \
        +  kFeedbackNumber * 2 * sizeof ( uint32_t )          \
        )

#define kSizeOfTakePictureParameters                                    \
        (    12 * ( sizeof ( double ) )                                 \
        +     3 * sizeof ( double )                                     \
        )

#define	kSizeOfMultipleFODParameters				   \
        (         sizeof ( int32_t ) +                                \
           12 * ( kSizeOldLongDouble + kLongDoublePadding )  + \
              2 * sizeof ( double )    +                           \
              4 * sizeof ( double )                                \
        )

#define	kSizeOf1FODdata					\
	( 4 * 3 * sizeof ( double )  +			\
	          sizeof ( double )  			\
        )


#define	kSizeOfCheckForEdgeParameters					\
	( 4 * 3 * sizeof ( double )  +					\
	   12 * ( kSizeOldLongDouble + kLongDoublePadding )  + 	\
              2 * sizeof ( double )    + 				\
              4 * sizeof ( double )      				\
        )

#define	kSizeOfCheckOrientParameters					\
	( 2 * 3 * sizeof ( double )    +				\
                  sizeof ( double )    + 				\
	   12 * ( kSizeOldLongDouble + kLongDoublePadding )  + 	\
              2 * sizeof ( double )    + 				\
              4 * sizeof ( double )      				\
        )

#define       kSizeOfShowTargetsParameters                              \
        (         sizeof ( int32_t )                                  	\
        )

#define       kSizeOf1TargetData                                        \
        (     2 * sizeof ( double )   +                            	\
                  sizeof ( int32_t )                                  	\
        )

#define	kSizeOfROIParameters			\
	( sizeof ( int32_t )  + 			\
              3 * sizeof ( double )    		\
        )

#define	kSizeOfAutoCertParameters					\
	( 12 * ( kSizeOldLongDouble + kLongDoublePadding )  +	\
              3 * sizeof ( double )      				\
        )

#define	kSizeOfRightOnCertParameters					\
	(         sizeof(int32_t) +                                        \
             12 * ( kSizeOldLongDouble + kLongDoublePadding )  +	\
              3 * sizeof ( double )  +   				\
              2 * sizeof ( uint32_t ) + 				\
              2 * sizeof ( double )      				\
        )

#define	kSizeOfFracCertParameters					\
	(         sizeof(int32_t) +                                        \
             12 * ( kSizeOldLongDouble + kLongDoublePadding )  +	\
              3 * sizeof ( double )  +   				\
              2 * sizeof ( double ) + 					\
              2 * sizeof ( double )      				\
        )

#define	kSizeOfFileTransferParameters					\
	( sizeof ( uint32_t ) + 					\
	64 * sizeof( char ) )

#define	kSizeOfFileGetStartParameters					\
	( 			 					\
	     32 * sizeof( char ) 					\
        )

#define	kSizeOfFileGetDataParameters					\
	( 			 					\
	     32 * sizeof( char ) +					\
	          sizeof( int32_t ) + 					\
	          sizeof( int32_t ) 					\
        )

#define	kSizeOfFilePutStartParameters					\
	( 			 					\
	     32 * sizeof( char ) +					\
	          sizeof( int32_t ) 					\
        )

#define	kSizeOfFilePutDataParameters					\
	( 			 					\
	     32 * sizeof( char ) +					\
	          sizeof( int32_t ) + 					\
	          sizeof( int32_t ) + 					\
	          sizeof( int32_t ) 					\
        )

#define	kSizeOfFilePutDoneParameters					\
	( 			 					\
	     32 * sizeof( char )  					\
        )

#define	kSizeOfDisplayParameters					\
	( sizeof ( uint32_t ) + 					\
	kNumberOfRegPoints * 3 * sizeof ( double ) + 			\
	12 * ( kSizeOldLongDouble + kLongDoublePadding ) )

#define	kSizeOfDisplayChunksStartParameters				\
	( sizeof ( uint32_t ) )

#define	kSizeOfDisplayChunksDataParameters				\
	( 2 * sizeof ( uint32_t ) )

#define	kSizeOfDisplayChunksDoParameters				\
	( kNumberOfRegPoints * 2 * sizeof ( uint32_t ) + 		\
	12 * ( kSizeOldLongDouble + kLongDoublePadding ) )

#define kSizeOfDisplayChunksXYZParameters                               \
        ( kNumberOfRegPoints * 2 * sizeof ( double ) +                  \
        ( kNumberOfRegPoints * 3 * sizeof ( double ) +                  \
        12 * ( sizeof ( double ) ) )

#define	kSizeOfQuickCheckParameters					\
	( kNumberOfRegPoints * 2 * sizeof ( uint32_t ) )

#define	kSizeOfQuickiesParameters	( 2 * sizeof ( uint32_t ) )

#define	kSizeOf1Quickie    ( 2 * sizeof ( uint32_t ) )

#define	kSizeOfGeoQuickiesParameters	( 2 * sizeof ( uint32_t ) )

#define	kSizeOf1GeoQuickie    ( 2 * sizeof ( double ) )

#define	kSizeOfDisplaySeveralParameters					\
	( sizeof ( uint32_t ) )

#define	kSizeOfSearchForASensorParameters				\
	2 * sizeof ( double )

#define	kSizeOfGimme3DParameters					\
	( 2 * sizeof ( double ) + 					\
	12 * ( kSizeOldLongDouble + kLongDoublePadding ) )

#define	kSizeOfGetNewVideoParameters					\
	( sizeof ( int32_t ) + 						\
	2 * sizeof ( double ) + 					\
	3 * sizeof ( double ) + 					\
	12 * ( kSizeOldLongDouble + kLongDoublePadding ) )

#define	kSizeOfDisplayNewVideoParameters				\
	( sizeof ( int32_t ) + 						\
	12 * ( kSizeOldLongDouble + kLongDoublePadding ) +		\
        4 * 3 * sizeof ( double ) + 					\
	    sizeof ( int32_t )       +					\
        2 * sizeof ( double )     + 					\
        4 * sizeof ( double )     +	  				\
            sizeof ( double )     + 					\
            sizeof ( double )     + 					\
	    sizeof ( int32_t )       + 					\
	    sizeof ( int32_t )       + 					\
	4 * sizeof ( char ) )

#define	kSizeOfDisplayKitVideoParameters				\
	    ( sizeof ( int32_t )   					\
	+   12 * ( kSizeOldLongDouble + kLongDoublePadding ) 	\
	+   sizeof ( int32_t )        					\
	+   sizeof ( int32_t )        					\
        + 3 * sizeof ( double )   					\
	+   sizeof ( char )        					\
	+   sizeof ( char )                                             \
	    )


#define	kSizeOfDisplayVideoCheckParameters			\
	( sizeof ( uint32_t ) + 3 * sizeof ( double ) + 	\
	12 * ( kSizeOldLongDouble + kLongDoublePadding ) )

#define	kSizeOfDisplayNoQuickCheckParameters				\
	( sizeof ( uint32_t ) + 					\
	12 * ( kSizeOldLongDouble + kLongDoublePadding ) )

#define	kSizeOfDoubleToTransformParameters				\
	( 12 * ( sizeof ( double ) ) )

#define	kSizeOfDisplayRawParameters					\
	( sizeof ( uint32_t ) )
	
#define	kSizeOfHobbsSetParameters					\
	( 2 * sizeof ( uint32_t ) )
	
#define	kSizeOfHobbsGetParameters					\
	( 1 * sizeof ( uint32_t ) )

#define	kSizeOfSetBitParameters						\
	( 2 * sizeof ( uint32_t ) )
	
#define kMaxParametersLength			kSizeOfDisplayParameters

#define kSizeOfAutoFocusCmdParameters  8 * sizeof( int32_t )    	

#define kSizeOfQuickCheckCountParameters  sizeof( int32_t )    	

#define kSizeOfQuickCheckTimerParameters  sizeof( int32_t )    	

#define	kSizeOfLevelScanParameters		( 2 * sizeof ( double ) )

#define kSizeOfFlexDisplayParameters                                    \
        ( sizeof ( uint32_t ) +                                    \
        sizeof ( uint32_t ) +                                      \
        kNumberOfFlexPoints * 3 * sizeof ( double ) +                   \
        12 * ( kSizeOldLongDouble + kLongDoublePadding ) )

#define kSizeOfFlexQuickCheckParameters                                 \
        ( sizeof ( uint32_t ) +                                    \
        kNumberOfFlexPoints * 2 * sizeof ( uint32_t ) )

#define kSizeOfThresholdQuickCheckParameters                            \
        ( sizeof ( uint32_t ) +                                    \
          sizeof ( uint32_t ) +                                    \
        kNumberOfFlexPoints * 2 * sizeof ( uint32_t ) )


#define kSizeOfFlexDisplayChunksDoParameters                            \
        ( sizeof ( int32_t )                                               \
        + kNumberOfFlexPoints * 2 * sizeof ( uint32_t )            \
        + 12 * ( kSizeOldLongDouble                                 \
        + kLongDoublePadding ) )

#define kSizeOfFlexCalculateTransformParameters                         \
        ( sizeof ( int32_t )                                               \
        + kNumberOfFlexPoints * 3 * sizeof ( double )                   \
        + kNumberOfFlexPoints * 2 * sizeof ( double ) )

#define kSizeOfFlexRegWithFeedbackParameters                           \
        (  sizeof(int32_t)                                                \
        +  sizeof(int32_t)                                                \
        +  kNumberOfFlexPoints * 3 * sizeof ( double )                 \
        +  kNumberOfFlexPoints * 2 * sizeof ( double )                 \
        +  kNumberOfFlexPoints * 2 * sizeof ( uint32_t )          \
        +  2 * sizeof(unsigned char )                                  \
        )

#define kSizeOfFlexCalWithFeedbackParameters                           \
        (  sizeof(int32_t)                                                \
        +  kNumberOfFlexPoints * 3 * sizeof ( double )                 \
        +  kNumberOfFlexPoints * 2 * sizeof ( double )                 \
        )


#define kSizeOfRefreshRateParameters                                    \
        ( sizeof ( int32_t )                                               \
        + sizeof ( int32_t ) )

#define kSizeOfChangeDisplayPeriodParameters    ( sizeof ( int32_t )  )

#define kSizeOfChangeTransformToleranceParameters    ( sizeof ( double )  )

#define kSizeOfGetTargetsUsed                                            \
        ( sizeof(int32_t) +  64 * sizeof ( unsigned char )   )                              

#define kSizeOfCalibrateXYParameters                                      \
        ( 3 * sizeof( double ) )

#endif


