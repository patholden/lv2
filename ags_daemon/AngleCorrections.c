#include <stdint.h>
//static char rcsid[] = "$Id: AngleCorrections.c,v 1.11 2001/01/03 17:46:35 ags-sw Exp pickle $";

#include <float.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <math.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <unistd.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "AppErrors.h"
#include "AngleCorrections.h"
#include "LaserInterface.h"
#include "3DTransform.h"
#include "AutoCert.h"
#include "Files.h"

#include "readnewcal.h"
#include "polyfunc.h"

#if GENERATINGCFM
#define setround(x)
#define rint(x) floor(x)
#else
#endif

#include <stdlib.h>
#include <string.h>


enum
{
	kNone,
	kX,
	kY
};


static	unsigned char		gHalfWayThru = false;

static	double	AbsCos ( double x1, double y1, double x2, double y2 );

static	char **		gFileBuffer = 0;
static	int32_t		gFileLength = 0;

static	double	**gXTheoryHdl = 0;
static	double	**gYTheoryHdl = 0;

static	double	**gXActualHdl = 0;
static	double	**gYActualHdl = 0;

static	short		gNumberOfPoints = 0;

static	double	(*gXTheoryCoeffs)[kGridPointsY][6] = 0;
static	double	(*gYTheoryCoeffs)[kGridPointsY][6] = 0;

static	double	(*gXActualCoeffs)[kGridPointsY][6] = 0;
static	double	(*gYActualCoeffs)[kGridPointsY][6] = 0;

static	short	*gSortedPoints = 0;

static	double	gCurrentX;
static	double gCurrentY;

static	double	*gCurrentXTable;
static	double *gCurrentYTable;

						
static	unsigned char		BuildTwoTables ( double *x, double *y,
						double *zX, double *zY,
						double coeffsX[][kGridPointsY][6],
						double coeffsY[][kGridPointsY][6] );

static	int			DistanceFromCurrentPoint
						( const void *elem1, const void *elem2 );
						
static	unsigned char		GetPlaneCoefficients ( double *z,
						double *coefficients,
						double *x, double *y,
						short *indices, short type );
						
#if	kFOUR
static	unsigned char		GetBetterPlaneCoefficients ( double *z,
						double *coefficients,
						double *x, double *y,
						short *indices, short type );
#endif

#if kSIX
static	unsigned char		GetParabolicCoefficients ( double *z,
						double *coefficients,
						double *x, double *y,
						short *indices, short type );
#endif

static void DoCorrection(struct lg_master *pLgMaster, double *x, double *y,
 			 double	xCoeffs[][kGridPointsY][6],
 			 double	yCoeffs[][kGridPointsY][6]);
						
static	void		CleanupInit ( void );

static	unsigned char		AllocateAngleBuffers ( void );
static	unsigned char		AllocateCorrectionTables ( void );
static	unsigned char		ProcessFile (struct lg_master *pLgMaster);
						
enum
{
	kInitializingAngluarCorrectionsMsg = 1,
	kCannotAllocateAngleBuffer,
	kCalibrationFileNameNotFoundMsg,
	kCalibrationFileNotFound,
	kCalibrationFileCannotBeOpened,
	kCalibrationFileCannotBeSized,
	kCannotFitFileInMemory,
	kCannotReadCalibrationFile,
	kCalibrationFileCannotBeClosed,
	kLineIsTooLong,
	kDeltaMirrorCorrupted,
	kBadLine,
	kTooManyLines,
	kNoDeltaMirror,
	kProblemsSettingUpTheory,
	kProblemsSettingUpActual,
	kCannotAllocateTableBuffer,
	kCannotAllocateSortBuffer,
	kBuildingCorrectionTable,
	kBuildingCorruptionTable,
	kToleranceCorrupted,
	kSerialNumberCorrupted,
	kNoTolerance,
	kNoSerialNumber,
	kAutoCertFile,
	kCmdPeriodToStop
};

enum
{
	kCalibrationFileName = 1
};


void ApplyPoly (struct lg_master *pLgMaster, double *x, double *y )
{
        double xin, yin, xout, yout;

        xin = *x;
        yin = *y;
        polyfunc( pLgMaster, xin, yin, &xout, &yout );
        *x  = xout;
        *y  = yout;

#ifdef ZDEBUG
syslog(LOG_NOTICE, "ApplyPoly xyin %lf %lf xyout %lf %lf", xin, yin, xout, yout );
#endif

	return;
}

void ApplyCorrection (struct lg_master *pLgMaster, double *x, double *y )
{
#ifdef ZDEBUG
        double xin, yin, xout, yout;
#endif

	if (!pLgMaster->gCALIBFileOK) {
	    // next two lines fixes gcc complaints
	    *x = *x;
	    *y = *y;
#ifdef ZDEBUG
            xin = *x;
            yin = *y;
            xout = *x;
            yout = *y;
#endif
	} else {
#ifdef ZDEBUG
            xin = *x;
            yin = *y;
#endif
	    DoCorrection(pLgMaster, x, y, gXActualCoeffs, gYActualCoeffs );
#ifdef ZDEBUG
            xout = *x;
            yout = *y;
#endif
        }
#ifdef ZDEBUG
syslog(LOG_NOTICE, "ApplyCorrection xyin %lf %lf xyout %lf %lf", xin, yin, xout, yout );
#endif

	return;
}

void RemovePoly(struct lg_master *pLgMaster, double *x, double *y )
{
        double xin, yin, xout, yout;

        xin = *x;
        yin = *y;
        invpolyfunc( pLgMaster, xin, yin, &xout, &yout );
        *x  = xout;
        *y  = yout;

#ifdef ZDEBUG
syslog(LOG_NOTICE, "RemovePoly xyin %lf %lf xyout %lf %lf", xin, yin, xout, yout );
#endif
}

void RemoveCorrection(struct lg_master *pLgMaster, double *x, double *y )
{
#ifdef ZDEBUG
        double xin, yin, xout, yout;
#endif

	if (!pLgMaster->gCALIBFileOK) {
	    // next two lines fixes gcc complaints
	    *x = *x;
	    *y = *y;
#ifdef ZDEBUG
            xin = *x;
            yin = *y;
            xout = *x;
            yout = *y;
#endif
	} else {
#ifdef ZDEBUG
            xin = *x;
            yin = *y;
#endif
	    DoCorrection(pLgMaster, x, y, gXTheoryCoeffs, gYTheoryCoeffs );
#ifdef ZDEBUG
            xout = *x;
            yout = *y;
#endif
        }

#ifdef ZDEBUG
syslog(LOG_NOTICE, "RemoveCorrection xyin %lf %lf xyout %lf %lf", xin, yin, xout, yout );
#endif
	return;
}

static void DoCorrection(struct lg_master *pLgMaster, double *x, double *y,
	double	xCoeffs[][kGridPointsY][6],
	double	yCoeffs[][kGridPointsY][6] )
{
	double temp, indexLD, pointsLD;
	double weightRight, weightLeft, weightUp, weightDown;
	double wDwL, wDwR, wUwL, wUwR;
	short xIndex, yIndex, xIndex1, yIndex1; 

	if (!pLgMaster->gCALIBFileOK)
	  return;

	if ( *x < kGridOriginX )
	{
		xIndex = 0;
		temp = kGridOriginX - *x;
		weightRight = temp / ( temp + temp + kGridStepX );
		weightLeft = 1.0 - weightRight;
	}
	else
	{

		temp = ( *x - kGridOriginX ) / kGridStepX;
		indexLD = (double)floor ( 0.5 + (double)temp );
		pointsLD = (double)( kGridPointsX - 1 );
		if ( indexLD >= pointsLD )
		{
			xIndex = kGridPointsX - 2;
			temp = *x - kGridOriginX - kGridStepX * pointsLD;
			weightLeft = temp / ( temp + temp + kGridStepX );
			weightRight = 1.0 - weightLeft;
		}
		else
		{
			xIndex = (short)indexLD;
			weightRight = temp - indexLD;
			weightLeft = 1.0 - weightRight;
		}
	} 
	
	if ( *y < kGridOriginY )
	{
		yIndex = 0;
		temp = kGridOriginY - *y;
		weightUp = temp / ( temp + temp + kGridStepY );
		weightDown = 1.0 - weightUp;
	}
	else
	{

		temp = ( *y - kGridOriginY ) / kGridStepY;
		indexLD = (double)floor( 0.5 + (double)temp );
		pointsLD = (double)( kGridPointsY - 1 );
		if ( indexLD >= pointsLD )
		{
			yIndex = kGridPointsY - 2;
			temp = *y - kGridOriginY - kGridStepY * pointsLD;
			weightDown = temp / ( temp + temp + kGridStepY );
			weightUp = 1.0 - weightDown;
		}
		else
		{
			yIndex = (short)indexLD;
			weightUp = temp - indexLD;
			weightDown = 1.0 - weightUp;
		}
	} 

	
	wDwL = weightDown * weightLeft;
	wDwR = weightDown * weightRight;
	wUwL = weightUp * weightLeft;
	wUwR = weightUp * weightRight;
	xIndex1 = xIndex + 1;
	yIndex1 = yIndex + 1;

	temp =
		wDwL * xCoeffs[xIndex ][yIndex ][0] +
		wDwR * xCoeffs[xIndex1][yIndex ][0] +
		wUwL * xCoeffs[xIndex ][yIndex1][0] +
		wUwR * xCoeffs[xIndex1][yIndex1][0]
		+ *x * (
		wDwL * xCoeffs[xIndex ][yIndex ][1] +
		wDwR * xCoeffs[xIndex1][yIndex ][1] +
		wUwL * xCoeffs[xIndex ][yIndex1][1] +
		wUwR * xCoeffs[xIndex1][yIndex1][1] )
		+ *y * (
		wDwL * xCoeffs[xIndex ][yIndex ][2] +
		wDwR * xCoeffs[xIndex1][yIndex ][2] +
		wUwL * xCoeffs[xIndex ][yIndex1][2] +
		wUwR * xCoeffs[xIndex1][yIndex1][2] )
		;
	
	*y =
		wDwL * yCoeffs[xIndex ][yIndex ][0] +
		wDwR * yCoeffs[xIndex1][yIndex ][0] +
		wUwL * yCoeffs[xIndex ][yIndex1][0] +
		wUwR * yCoeffs[xIndex1][yIndex1][0]
		+ *x * (
		wDwL * yCoeffs[xIndex ][yIndex ][1] +
		wDwR * yCoeffs[xIndex1][yIndex ][1] +
		wUwL * yCoeffs[xIndex ][yIndex1][1] +
		wUwR * yCoeffs[xIndex1][yIndex1][1] )
	  + *y * (wDwL * yCoeffs[xIndex][yIndex][2]
			+ wDwR * yCoeffs[xIndex1][yIndex][2]
			+ wUwL * yCoeffs[xIndex][yIndex1][2]
			+ wUwR * yCoeffs[xIndex1][yIndex1][2]);
	
	*x = temp;	
	return;
}

void CloseAngleCorrections ( void )
{
	if ( gXTheoryCoeffs ) free( (void *)gXTheoryCoeffs );
	gXTheoryCoeffs = 0;
	if ( gYTheoryCoeffs ) free( (void *)gYTheoryCoeffs );
	gYTheoryCoeffs = 0;
	if ( gXActualCoeffs ) free( (void *)gXActualCoeffs );
	gXActualCoeffs = 0;
	if ( gYActualCoeffs ) free( (void *)gYActualCoeffs );
	gYActualCoeffs = 0;
}

#if kSIX
#include "NumRec.c"
#endif

unsigned char InitPoly (struct lg_master *pLgMaster)
{
	// uint16_t i;
        // int32_t j;
        // int length;
	// unsigned char fTableBuilt;
        // char * gTempBuff;
        // char * ptr;
        // int err;
        // char calibname[] = "/laser/data/calib";


        syslog(LOG_NOTICE   , "about to read polynomial mirror corrections" );
        readnewcal( pLgMaster );

        syslog(LOG_NOTICE   , "gDeltaMirror %lf", gDeltaMirror);
        syslog(LOG_NOTICE   , "gMirrorThickness %lf", gMirrorThickness);
        syslog(LOG_NOTICE   , "gHalfMirror %lf", pLgMaster->gHalfMirror);

	return true;
}

unsigned char InitAngleCorrections (struct lg_master *pLgMaster)
{
	uint16_t i;
        int32_t j;
        int length;
	unsigned char fTableBuilt;
        char * gTempBuff;
        char * ptr;
        int err;
        char calibname[] = "/laser/data/calib";
        
        gTempBuff = (char *)malloc( (size_t)CALIB_SIZE );
        if (gTempBuff <=0)
         {
           syslog(LOG_NOTICE, "Unable to malloc gTempBuff" );
           return(-3);
         }


        err = ReadFromFS( gTempBuff, calibname, 0, CALIB_SIZE, &length );
        if ( err ) {
           syslog(LOG_NOTICE,"unable to read %s", calibname); 
           return(-1);
        }

        syslog(LOG_NOTICE   , "calibration file size %d", length);
        
        j = 0L;
        ptr = gTempBuff;
        while( (*ptr) && (j < (int32_t)CALIB_SIZE) ) {
          if ( *ptr == '\r' ) *ptr = '\n';
          if ( isupper( *ptr ) ) {
             *ptr = tolower( *ptr );
          }
          ptr++; j++;
        }
        gFileLength = j;

        gFileBuffer = (char **)malloc( sizeof(void *) + (size_t)gFileLength );
        if (gFileBuffer)
	  *(void **)gFileBuffer = (void *)gFileBuffer + sizeof (void *);
        else
	  {
	    gFileBuffer = 0;
	    return false;
	  }
	
        memmove( *gFileBuffer, gTempBuff, (size_t)gFileLength );
	if ( !AllocateAngleBuffers (  ) ) return false;
	if ( !ProcessFile (pLgMaster) ) return false;

        syslog(LOG_NOTICE   , "gDeltaMirror %lf", gDeltaMirror);
        syslog(LOG_NOTICE   , "gMirrorThickness %lf", gMirrorThickness);
        syslog(LOG_NOTICE   , "gHalfMirror %lf", pLgMaster->gHalfMirror);
	
	if ( gFileBuffer ) free((void *)gFileBuffer );
	gFileBuffer = 0;
	
	if ( !AllocateCorrectionTables (  ) ) return false;
	
	gSortedPoints = (short *)
		malloc ( gNumberOfPoints * sizeof ( short ) );
	if (!gSortedPoints)
	  return false;
	
	i = gNumberOfPoints;
	while ( i-- ) gSortedPoints[i] = i;
	
	gHalfWayThru = false;
	
	fTableBuilt = BuildTwoTables ( *gXTheoryHdl, *gYTheoryHdl,
		*gXActualHdl, *gYActualHdl, gXActualCoeffs, gYActualCoeffs );
	if ( !fTableBuilt )
	  return false;

	gHalfWayThru = true;
	fTableBuilt = BuildTwoTables ( *gXActualHdl, *gYActualHdl,
		*gXTheoryHdl, *gYTheoryHdl, gXTheoryCoeffs, gYTheoryCoeffs );
	if (gSortedPoints)
	  {
	    free(gSortedPoints );
	    gSortedPoints = 0;
	  }
	if (!fTableBuilt )
	  return false;
	
	CleanupInit (  );
	pLgMaster->gCALIBFileOK = true;
	syslog(LOG_NOTICE   , "gCALIBFileOK = true");
	return true;
}

unsigned char ProcessFile (struct lg_master *pLgMaster)
{
	char * currentFilePtr;
	char * newFilePtr;
	unsigned char fWasPound, fWasDeltaMirror;
	unsigned char fWasTolerance, fWasSerialNumber;
	short lineNumber, numberOfDollars;
        int32_t  lineLength;
	char theLinePtr[1024];
	double tX, tY, tZ, aX, aY, dX, dY;
        double dtemp1;
	double currentTolerance, currentDeltaMirror;
	int32_t processedLength, currentSerialNumber;
	
	gNumberOfPoints = 0;
	currentFilePtr = *gFileBuffer;
	fWasPound = false;
	fWasDeltaMirror = false;
	lineNumber = 0;
	numberOfDollars = 0;
	processedLength = 0L;
	
	while ( gFileLength )
	{
		lineNumber++;

		lineLength = 0L;
		currentFilePtr = &(*gFileBuffer)[processedLength];
		newFilePtr = currentFilePtr;

                while ( ( lineLength < gFileLength ) &&
                        ( *newFilePtr != 0x0A )      &&
                        ( *newFilePtr != 0x0D ) ) {
                  lineLength++;
                  newFilePtr++;
                }
		if ( ( lineLength != gFileLength ) &&
			((*newFilePtr == 0x0A) || (*newFilePtr == 0x0D)) )
		{
			lineLength++;
			newFilePtr++;
		}

		if ( lineLength > 1023 )
		{
			return false;
		}
		
		gFileLength -= lineLength;
		processedLength += lineLength;
		if ( lineLength == 0 ) continue;

		
		memmove( (void *)theLinePtr, (void *)currentFilePtr,
                         (size_t)lineLength );
		currentFilePtr = newFilePtr;
		
		if (  ( theLinePtr[0] == 0x0D ) || ( theLinePtr[0] == 0x0A ) )
			continue;

		if (  ( theLinePtr[lineLength - 1] ==  0x0D )
		   || ( theLinePtr[lineLength - 1] ==  0x0A ) )
			theLinePtr[lineLength - 1] = '\0';
				
		if ( memcmp ( theLinePtr, "$AutoCert", 9 ) == 0 )
		{
			return false;
		}

		if ( theLinePtr[0] == '$' )
		{
			numberOfDollars++;
			switch ( numberOfDollars )
			{
				case 8:
					if ( sscanf (theLinePtr
                                                    , "$%lf"
                                                    , &dtemp1
                                                    ) == 1 )
					{
					    gMirrorThickness =       dtemp1;
                                            pLgMaster->gHalfMirror = 0.5 * dtemp1;
					}
                                        break;
				case 7:
					currentDeltaMirror = gDeltaMirror;
					if ( sscanf ( theLinePtr, "$%lf ",
						&gDeltaMirror ) != 1 )
					{
					    gDeltaMirror = currentDeltaMirror;
					    return false;
					}
					fWasDeltaMirror = true;
					break;
				case 6:
					currentTolerance = pLgMaster->gTolerance;
					if ( sscanf ( theLinePtr, "$%lf ",
						&pLgMaster->gTolerance ) != 1 )
					{
						pLgMaster->gTolerance = currentTolerance;
						return false;
					}
					fWasTolerance = true;
					break;
				case 5:
					currentSerialNumber = pLgMaster->gProjectorSerialNumber;
					if ( sscanf ( theLinePtr, "$%ld ",
						&pLgMaster->gProjectorSerialNumber ) != 1 )
					{
						pLgMaster->gProjectorSerialNumber = currentSerialNumber;
						return false;
					}
					fWasSerialNumber = true;
					break;
				default:
					break;
			}
			continue;
		}
		
		fWasPound = false;
		if ( theLinePtr[0] == '#' ) fWasPound = true;
		if ( fWasPound ) continue;

		if ( sscanf ( theLinePtr,
				" %lf %lf %lf %lf %lf %lf %lf ", &tX, &tY, &tZ,
				&aX, &aY, &dX, &dY  ) != 7 )
		{
			return false;
		}

		if ( gNumberOfPoints >= kMaxNumberOfCALIBAngles )
		{
			return false;
		}
		
		if ( !fWasDeltaMirror )
		{
			return false;
		}
		GeometricAnglesFrom3D(pLgMaster, tX, tY, tZ,
			&(*gXTheoryHdl)[gNumberOfPoints],
			&(*gYTheoryHdl)[gNumberOfPoints] );
			
		ConvertGeometricAnglesToMirror (
			&(*gXTheoryHdl)[gNumberOfPoints],
			&(*gYTheoryHdl)[gNumberOfPoints] );
		
		GeometricAnglesFrom3D(pLgMaster, aX, aY, tZ,
			&(*gXActualHdl)[gNumberOfPoints],
			&(*gYActualHdl)[gNumberOfPoints] );
			
		ConvertGeometricAnglesToMirror (
			&(*gXActualHdl)[gNumberOfPoints],
			&(*gYActualHdl)[gNumberOfPoints] );

#ifdef ZDEBUG
syslog( LOG_NOTICE, "%d ", gNumberOfPoints );
syslog( LOG_NOTICE, "%lf %lf ",(*gXTheoryHdl)[gNumberOfPoints],(*gYTheoryHdl)[gNumberOfPoints] );
syslog( LOG_NOTICE, "%lf %lf ",(*gXActualHdl)[gNumberOfPoints],(*gYActualHdl)[gNumberOfPoints] );
syslog( LOG_NOTICE, "caldata" );
#endif

		gNumberOfPoints++;
	}
	
	if ( !fWasTolerance )
	{
		return false;
	}
	
	if ( !fWasSerialNumber )
	{
		return false;
	}
	
	syslog(LOG_NOTICE   , "Points %d    ", gNumberOfPoints);
	syslog(LOG_NOTICE   , "Mirror %f   ", gDeltaMirror);
	syslog(LOG_NOTICE   , "SerialNumber %ld", pLgMaster->gProjectorSerialNumber);
	return true;
}

unsigned char AllocateAngleBuffers ( void )
{
		
    gXTheoryHdl = malloc( sizeof (void *) +
			  (size_t) ( kMaxNumberOfCALIBAngles * sizeof ( double ) ) );
    if (gXTheoryHdl)
      *(void **)gXTheoryHdl = (void *)gXTheoryHdl + sizeof (void *);
    else
      {
	gXTheoryHdl = 0;
	return false;
      }

    gYTheoryHdl = malloc( sizeof (void *) +
			  (size_t) ( kMaxNumberOfCALIBAngles * sizeof ( double ) ) );
    if (gYTheoryHdl)
      *(void **)gYTheoryHdl = (void *)gYTheoryHdl + sizeof (void *);
    else
      {
	gYTheoryHdl = 0;
	return false;
      }

    gXActualHdl = malloc( sizeof (void *) +
			  (size_t) ( kMaxNumberOfCALIBAngles * sizeof ( double ) ) );
    if (gXActualHdl)
      *(void **)gXActualHdl = (void *)gXActualHdl + sizeof (void *);
    else
      {
	gXActualHdl = 0;
	return false;
      }

    gYActualHdl = malloc( sizeof (void *) +
			  (size_t) ( kMaxNumberOfCALIBAngles * sizeof ( double ) ) );
    if (gYActualHdl)
      *(void **)gYActualHdl = (void *)gYActualHdl + sizeof (void *);
    else
      {
	gYActualHdl = 0;
	return false;
      }
    return true;
}


unsigned char AllocateCorrectionTables ( void )
{
    gXTheoryCoeffs = (double (*)[kGridPointsY][6])malloc
      ( kGridPointsX * kGridPointsY * 6 * sizeof ( double ) );
    if (!gXTheoryCoeffs)
      return false;
    gYTheoryCoeffs = (double (*)[kGridPointsY][6])malloc
      ( kGridPointsX * kGridPointsY * 6 * sizeof ( double ) );
    if (!gYTheoryCoeffs)
      return false;
    gXActualCoeffs = (double (*)[kGridPointsY][6])malloc
      ( kGridPointsX * kGridPointsY * 6 * sizeof ( double ) );
    if (!gXActualCoeffs)
      return false;
    gYActualCoeffs = (double (*)[kGridPointsY][6])malloc
      ( kGridPointsX * kGridPointsY * 6 * sizeof ( double ) );
    if (!gYActualCoeffs)
      return false;
    return true;
}

void CleanupInit ( void )
{
    if (gXTheoryHdl)
      {
	free(gXTheoryHdl);
	gXTheoryHdl = 0;
      }
    if (gYTheoryHdl)
      {
	free(gYTheoryHdl);
	gYTheoryHdl = 0;
      }
    if (gXActualHdl)
      {
	free ( (void *)gXActualHdl );
	gXTheoryHdl = 0;
      }
    if (gYActualHdl)
      {
	free(gYActualHdl);
	gYActualHdl = 0;
      }
    if (gFileBuffer)
      {
	free(gFileBuffer);
	gFileBuffer = 0;
      }
    if (gSortedPoints)
      {
	free(gSortedPoints);
	gSortedPoints = 0;
      }
    return;
}


unsigned char BuildTwoTables ( double *x, double *y,
	double *zX, double *zY,
	double coeffsX[][kGridPointsY][6],
	double coeffsY[][kGridPointsY][6] )
{
	short index[6], nX, nY, i, j, k;
	short indexCacheX[6], indexCacheY[6], n;
	unsigned char fPlaneFoundX, fPlaneFoundY;
	double coeffsXcache[6], coeffsYcache[6];

	
	i = 6;
	while ( i-- )
	{
		indexCacheX[i] = gNumberOfPoints;
		indexCacheY[i] = gNumberOfPoints;
	}
	
	gCurrentXTable = x;
	gCurrentYTable = y;
	
	for ( nX = 0; nX < kGridPointsX; nX++ )
	{
		for ( nY = 0; nY < kGridPointsY; nY++ )
		{
			gCurrentX = kGridOriginX + nX * kGridStepX;
			gCurrentY = kGridOriginY + nY * kGridStepY;
			qsort ( (void *)gSortedPoints, (size_t)gNumberOfPoints,
				sizeof ( short ), &DistanceFromCurrentPoint );

			fPlaneFoundX = false;
			fPlaneFoundY = false;
#if		kSIX
			for ( l = 3; l < ( gNumberOfPoints - 2 ); l++ )
			{
				for ( k = 2; k < l; k++ )
				{
					for ( j = 1; j < k; j++ )
					{
						for ( i = 0; i < j; i++ )
						{
							for ( q = l + 2; q < gNumberOfPoints; q++ )
							{
								for ( p = l + 1; p < q; p++ )
								{
#elif	kFOUR
			for ( l = 3; l < gNumberOfPoints; l++ )
			{
				for ( k = 2; k < l; k++ )
				{
					
					for ( j = 1; j < k; j++ )
					{
						
						for ( i = 0; i < j; i++ )
						{
#else
			for ( k = 2; k < gNumberOfPoints; k++ )
			{
				
				for ( j = 1; j < k; j++ )
				{
					
					for ( i = 0; i < j; i++ )
					{
#endif
						index[0] = gSortedPoints[i];
						index[1] = gSortedPoints[j];
						index[2] = gSortedPoints[k];
#if		kSIX
						index[3] = gSortedPoints[l];
						index[4] = gSortedPoints[p];
						index[5] = gSortedPoints[q];
#elif	kFOUR
						index[3] = gSortedPoints[l];
#endif
						if ( !fPlaneFoundX )
						{
							if ( ( index[0] == indexCacheX[0] )
								&& ( index[1] == indexCacheX[1] )
								&& ( index[2] == indexCacheX[2] )
#if		kSIX
								&& ( index[3] == indexCacheX[3] )
								&& ( index[4] == indexCacheX[4] )
								&& ( index[5] == indexCacheX[5] )
#elif	kFOUR
								&& ( index[3] == indexCacheX[3] )
#endif
								)
							{
								n = 6;
								while ( n-- ) {
                                                                    coeffsX[nX][nY][n] = coeffsXcache[n];
                                                                }
								fPlaneFoundX = true;
							}
							else
							{
								fPlaneFoundX =
#if		kSIX
									GetParabolicCoefficients
#elif	kFOUR
									GetBetterPlaneCoefficients
#else
									GetPlaneCoefficients
#endif
										( zX, coeffsX[nX][nY],
										x, y, index, kX );
								if ( fPlaneFoundX )
								{
									n = 6;
									while ( n-- )
									{
										indexCacheX[n] = index[n];
										coeffsXcache[n] = 
											coeffsX[nX][nY][n];
									}
								}
							}
						}
						if ( !fPlaneFoundY )
						{
							if ( ( index[0] == indexCacheY[0] )
								&& ( index[1] == indexCacheY[1] )
								&& ( index[2] == indexCacheY[2] )
#if		kSIX
								&& ( index[3] == indexCacheY[3] )
								&& ( index[4] == indexCacheY[4] )
								&& ( index[5] == indexCacheY[5] )
#elif	kFOUR
								&& ( index[3] == indexCacheY[3] )
#endif
								)
							{
								n = 6;
								while ( n-- ) {
                                                                   coeffsY[nX][nY][n] = coeffsYcache[n];
                                                                }
								fPlaneFoundY = true;
							}
							else
							{
								fPlaneFoundY =
#if		kSIX
									GetParabolicCoefficients
#elif	kFOUR
									GetBetterPlaneCoefficients
#else
									GetPlaneCoefficients
#endif
										( zY, coeffsY[nX][nY],
										x, y, index, kY );
								if ( fPlaneFoundY )
								{
									n = 6;
									while ( n-- )
									{
										indexCacheY[n] = index[n];
										coeffsYcache[n] = 
											coeffsY[nX][nY][n];
									}
								}
							}
						}
#if		kSIX
									if ( fPlaneFoundX && fPlaneFoundY )
										break;
								}
								if ( fPlaneFoundX && fPlaneFoundY )
									break;
							}
							if ( fPlaneFoundX && fPlaneFoundY )
								break;
						}
						if ( fPlaneFoundX && fPlaneFoundY ) break;
					}
					if ( fPlaneFoundX && fPlaneFoundY ) break;
				}
				if ( fPlaneFoundX && fPlaneFoundY ) break;
			}
#elif	kFOUR
							if ( fPlaneFoundX && fPlaneFoundY ) break;
						}
						if ( fPlaneFoundX && fPlaneFoundY ) break;
					}
					if ( fPlaneFoundX && fPlaneFoundY ) break;
				}
				if ( fPlaneFoundX && fPlaneFoundY ) break;
			}
#else
						if ( fPlaneFoundX && fPlaneFoundY ) break;
					}
					if ( fPlaneFoundX && fPlaneFoundY ) break;
				}
				if ( fPlaneFoundX && fPlaneFoundY ) break;
			}
#endif
			if ( fPlaneFoundX && fPlaneFoundY ) continue;
			else return false;
		}
	}
	return true;
}

int DistanceFromCurrentPoint ( const void *elem1, const void *elem2 )
{
	register double dist, temp;
	temp = gCurrentXTable[*(const short *)elem1] - gCurrentX;
	dist = temp * temp;
	temp = gCurrentYTable[*(const short *)elem1] - gCurrentY;
	dist += temp * temp;
	temp = gCurrentXTable[*(const short *)elem2] - gCurrentX;
	dist -= temp * temp;
	temp = gCurrentYTable[*(const short *)elem2] - gCurrentY;
	dist -= temp * temp;
	if ( dist < 0.0 ) return -1;
	if ( dist > 0.0 ) return 1;
	return 0;
}

	
#define kDeterminantMin		0.000001	
#define kMaxCos				0.850	

#if	kFOUR
unsigned char GetBetterPlaneCoefficients ( double *z,
		double *coefficients, double *x, double *y,
		short *indices, short type )
{
	double m[3][3], mi[3][3], zv[3], tempCoeffs[3];
	short i, nOfPlanes;
	short *indexPtr;

	// FIXME--PAH--fix compile error: Get rid of parameter??
	type = type;
	
	indexPtr = indices;
	nOfPlanes = 0;
	i = 3;
	while ( i-- ) coefficients[i] = 0.0;
	for ( i = 0; i < 3; i++, indexPtr++ )
	{
		m[i][0] = 1.0;
		m[i][1] = x[*indexPtr];
		m[i][2] = y[*indexPtr];
		zv[i] = z[*indexPtr];
	}
	if ( ( fabs ( Determinant3D ( m ) ) > kDeterminantMin ) &&
		( AbsCos ( ( m[1][1] - m[0][1] ), ( m[1][2] - m[0][2] ), 
		( m[2][1] - m[0][1] ), ( m[2][2] - m[0][2] ) ) < kMaxCos ) )
	{
		Invert3DMatrix ( m, mi );
		Multiply3DMatrixByVector ( mi, zv, tempCoeffs );
		i = 3;
		while ( i-- ) coefficients[i] += tempCoeffs[i];
		nOfPlanes++;
	}
	m[2][1] = x[3];
	m[2][2] = y[3];
	zv[2] = z[3];
	if ( ( fabs ( Determinant3D ( m ) ) > kDeterminantMin ) &&
		( AbsCos ( ( m[1][1] - m[0][1] ), ( m[1][2] - m[0][2] ), 
		( m[2][1] - m[0][1] ), ( m[2][2] - m[0][2] ) ) < kMaxCos ) )
	{
		Invert3DMatrix ( m, mi );
		Multiply3DMatrixByVector ( mi, zv, tempCoeffs );
		i = 3;
		while ( i-- ) coefficients[i] += tempCoeffs[i];
		nOfPlanes++;
	}
	m[1][1] = x[2];
	m[1][2] = y[2];
	zv[1] = z[2];
	if ( ( fabs ( Determinant3D ( m ) ) > kDeterminantMin ) &&
		( AbsCos ( ( m[1][1] - m[0][1] ), ( m[1][2] - m[0][2] ), 
		( m[2][1] - m[0][1] ), ( m[2][2] - m[0][2] ) ) < kMaxCos ) )
	{
		Invert3DMatrix ( m, mi );
		Multiply3DMatrixByVector ( mi, zv, tempCoeffs );
		i = 3;
		while ( i-- ) coefficients[i] += tempCoeffs[i];
		nOfPlanes++;
	}
	if ( nOfPlanes == 0 ) return false;
	i = 3;
	while ( i-- ) coefficients[i] /= (double)nOfPlanes;
	return true;
}
#endif

unsigned char GetPlaneCoefficients ( double *z,
		double *coefficients, double *x, double *y,
		short *indices, short type )
{
	double m[3][3], mi[3][3], zv[3], temp;
	short i;

	type = type;
	
	for ( i = 0; i < 3; i++, indices++ )
	{
		m[i][0] = 1.0;
		m[i][1] = x[*indices];
		m[i][2] = y[*indices];
		zv[i] = z[*indices];
	}
	if ( fabs ( Determinant3D ( m ) ) <= kDeterminantMin ) return false;
	temp = AbsCos ( ( m[1][1] - m[0][1] ), ( m[1][2] - m[0][2] ), 
		( m[2][1] - m[0][1] ), ( m[2][2] - m[0][2] ) );
	if ( temp >= kMaxCos ) return false;
	Invert3DMatrix ( m, mi );
	Multiply3DMatrixByVector ( mi, zv, coefficients );
	return true;
}

double AbsCos ( double x1, double y1,
	double x2, double y2 )
{
	return ( fabs ( x1 * x2 + y1 * y2 ) /
		sqrt ( ( x1 * x1 + y1 * y1 ) * ( x2 * x2 + y2 * y2 ) ) );
}


#if kSIX
unsigned char GetParabolicCoefficients ( double *z,
		double *coefficients, double *x, double *y,
		short *indices, short type )
{
	double m[6][6], zv[6];
	short i, indx[6];
	i = 0;
	while ( i < 6 )
	{
		m[i][0] = 1.0;
		m[i][1] = x[*indices];
		m[i][2] = y[*indices];
		m[i][3] = x[*indices] * x[*indices];
		m[i][4] = x[*indices] * y[*indices];
		m[i][5] = y[*indices] * y[*indices];
		switch ( type )
		{
			case kX:
				zv[i] = z[*indices] - x[*indices];
				break;
			case kY:
				zv[i] = z[*indices] - y[*indices];
				break;
			default:
				zv[i] = z[*indices];
				break;
		}
		i++;
		indices++;
	}
	if ( LUDCMP ( m, indx ) == 0.0 ) return false;
	if ( fabs (
		m[0][0] * m[1][1] * m[2][2] * m[3][3] * m[4][4] * m[5][5]
		) <= kDeterminantMin ) return false;
	LUBKSB ( m, indx, zv );
	memmove( (void *)coefficients, (void *)zv, sizeof ( zv ) );
	switch ( type )
	{
		case kX:
			coefficients[1] += 1.0;
			break;
		case kY:
			coefficients[2] += 1.0;
			break;
		default:
			break;
	}
	return true;
}
#endif
