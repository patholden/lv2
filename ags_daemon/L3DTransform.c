//static char rcsid[] = "$Id: L3DTransform.c,v 1.2 1999/10/22 15:39:02 ags-sw Exp $";
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <stddef.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <assert.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/ioctl.h>
#include <linux/laser_api.h>
#include <float.h>
#include <math.h>
#include "BoardComm.h"
#include "L3DTransform.h"
/********************************************************************/
/*								*/
/*	By Ilya R. Lapshin at Assembly Guidance Systems, Inc.	*/
/*								*/
/********************************************************************/
int permuted[24][4] = { { 0, 1, 2, 3 }
                      , { 0, 1, 3, 2 }
                      , { 0, 2, 1, 3 }
                      , { 0, 2, 3, 1 }
                      , { 0, 3, 1, 2 }
                      , { 0, 3, 2, 1 }
                      , { 1, 0, 2, 3 }
                      , { 1, 0, 3, 2 }
                      , { 1, 2, 0, 3 }
                      , { 1, 2, 3, 0 }
                      , { 1, 3, 0, 2 }
                      , { 1, 3, 2, 0 }
                      , { 2, 0, 1, 3 }
                      , { 2, 0, 3, 1 }
                      , { 2, 1, 0, 3 }
                      , { 2, 1, 3, 0 }
                      , { 2, 3, 0, 1 }
                      , { 2, 3, 1, 0 }
                      , { 3, 0, 1, 2 }
                      , { 3, 0, 2, 1 }
                      , { 3, 1, 0, 2 }
                      , { 3, 1, 2, 0 }
                      , { 3, 2, 0, 1 }
                      , { 3, 2, 1, 0 }
                      };

enum {
	false,
	true
};

typedef unsigned char Boolean;

#define pi()	3.14159265358979323846L

static	long double 	gQuarterPi_;
static	long double	gSqrtOfTwo_;

typedef struct { long double oldLoc[3], xRad, yRad; }
	inputPoint;

typedef struct { long double rotMatrix[3][3], transVector[3]; }
	transform;

static	void MakeARotMatrix ( long double m[3][3] );

static	void FindRotMatrix ( long double iV1[3], long double iV2[3], 
	long double oV1[3], long double oV2[3], long double rotMat[3][3] );

static	long double CubicRoot ( long double x );

extern
double
pDist ( inputPoint *iPt0, inputPoint *iPt1, inputPoint *iPt2 );

extern
double
aDist ( inputPoint *iPt0, inputPoint *iPt1, inputPoint *iPt2 );

static	void		TransformPoint
						( transform *tr, long double oldLoc[3],
						long double newLoc[3] );
	
static	long double	Square ( long double x );


static	void		Multiply3DMatrices
						( long double m1[3][3], long double m2[3][3],
						long double mp[3][3] );
	
static	void		Multiply3DMatrixByVector
						( long double m[3][3],
						long double v[3], long double vp[3] );
	
static	long double	Determinant3D ( long double m[3][3] );

static	short		SolveTheSystem ( long double a01, long double sqd01,
						long double a02, long double sqd02,
						long double a12, long double sqd12,
						long double rs[16][3] );
	
static	short		Solve4thDegree (
						long double a, long double b,
						long double c, long double d,
						long double r[4] );
	
static	short		Solve3rdDegree (
						long double a, long double b,
						long double c, long double r[3] );
	
static	short		Solve2ndDegree
						( long double a, long double b,
						long double r[2] );

#define kMaxIter			50

#define kReallySmall		1.E-4 /* Good value: 1.E-7L */

static	long double	MirrorFactor_ ( long double x );

static	long double	MirrorOffset_ ( long double x );
static void GetAngles(struct lg_master *pLgMaster, long double *x, long double *xa, long double *ya,
		      long double deltaXHeight);
static long double ZfromR(struct lg_master *pLgMaster, long double R,long double xa,
			  long double ya, long double deltaXHeight );
static long double CoeffX(struct lg_master *pLgMaster, long double z, long double xa, long double ya,
			  long double deltaXHeight);
static long double CoeffY(struct lg_master *pLgMaster, long double z, long double ya);
static short From3PtsToTrans(struct lg_master *pLgMaster, inputPoint *iPt0, inputPoint *iPt1,
			     inputPoint *iPt2, long double deltaXHeight, transform *tr);

double minDistance = 1.0;
double minL3Distance = 1.0;
double minAngDistance = 0.02;


#define kIterations 7
#define	kHalfXMirror	0.031L
#define	kHalfYMirror	0.031L

/* might be changed to improve the performance */



void TransformPoint
	( transform *tr, long double oldLoc[3], long double newLoc[3] )
{
	Multiply3DMatrixByVector
		( tr->rotMatrix, oldLoc, newLoc );
	newLoc[0] += tr->transVector[0];
	newLoc[1] += tr->transVector[1];
	newLoc[2] += tr->transVector[2];
}

long double MirrorFactor_ ( long double x )
{
	return ( 1.L / cos ( x * .5L - gQuarterPi_ ) - gSqrtOfTwo_ );
}

long double MirrorOffset_ ( long double x )
{
	return ( 1.L / cos ( x * .5L - gQuarterPi_ ) );
}

void GetAngles(struct lg_master *pLgMaster, long double *x, long double *xa, long double *ya,
	long double deltaXHeight )
{
	long double temp;
	short i;
	if (fabs(x[2]) > LDBL_MIN)
	  {
	    *ya = atan(x[1] / x[2]);
	    if (x[2] < 0.L)
	      *ya += pi();
	    i = kIterations;
	    while (i--)
	      {
		*ya =
		  atan((x[1] - (pLgMaster->gHalfMirror * MirrorFactor_(*ya))) / x[2]);
		if (x[2] < 0.L)
		  *ya += pi();
	      }
	    temp = (1.L / fabs(cos(*ya)))
	      + fabs((deltaXHeight - pLgMaster->gHalfMirror * MirrorOffset_(*ya))
		     / x[2]);
	    *xa = atan((x[0] / x[2]) / temp);
	    if ( x[2] < 0.L )
	      *xa += pi();
	    i = kIterations;
	    while (i--)
	      {
		*xa = atan(((x[0] - pLgMaster->gHalfMirror * MirrorFactor_(*xa)) / x[2]) / temp);
		if (x[2] < 0.L)
		  *xa += pi();
	      }
	  }
	else
	{
	  temp = fabs(deltaXHeight - (pLgMaster->gHalfMirror * MirrorOffset_(0.L)));
	  *xa = atan(x[0] / temp);
	  if (x[2] < 0)
	    *xa += pi();
	  i = kIterations;
	  while (i--)
	    {
	      *xa = atan((x[0] - pLgMaster->gHalfMirror * MirrorFactor_(*xa)) / temp);
	      if (x[2] < 0)
		*xa += pi();
	    }
	  if (fabs(x[1]) > LDBL_MIN)
	    {
	      if (x[1] < 0)
		*ya = -0.5L * pi();
	      else
		*ya = 0.5L * pi();
	    }
	  else
	    *ya = 0.L;
	}
	return;	
}


/************************************************************************/
/*	For the four inputPoints FindTransform looks for the best       */
/*	fitting transform. If not able to find one satisfying the       */
/*	error allowance errDist, returns false, otherwise returns       */
/*	true and the transform in tr.				        */
/************************************************************************/
Boolean FindBestTransform(struct lg_master *pLgMaster, doubleInputPoint *DiPt,
			  doubleTransform *Dtr, double DdeltaXHeight, double Dtolerance,
			  double * bestCosine)
{
	short i, j, numberOfTrans;
        int a, b, c, d;
        int index;	
	long double newPoint[3], tan4thPtX, tan4thPtY, sec4thPtSq;
	long double theBestCos, tempBestCos, xa, ya;
	inputPoint iPt[4];
        transform *tr;
        transform Ttr;
	double deltaXHeight;
        double tolerance;
        double perpend;
        double angdist;
        int goodPermutes;

	transform trBestForEach3, trFor3Pts[16];

        *bestCosine = -1.0;

        tr = &Ttr;

             // reject quartet if two targets are the same
        for (i=0; i<4; i++)
	  {
            for (j=i; j<4; j++)
	      {
                if (j != i)
		  {
                    if ((DiPt[i].oldLoc[0] == DiPt[j].oldLoc[0])
			&& (DiPt[i].oldLoc[1] == DiPt[j].oldLoc[1])
			&& (DiPt[i].oldLoc[2] == DiPt[j].oldLoc[2]))
		      return(false);
		  }
	      }
	  }
        deltaXHeight = DdeltaXHeight;
        tolerance = Dtolerance;
        for ( i=0; i<4; i++)
	  {
            iPt[i].oldLoc[0] = (DiPt[i].oldLoc[0]);
            iPt[i].oldLoc[1] = (DiPt[i].oldLoc[1]);
            iPt[i].oldLoc[2] = (DiPt[i].oldLoc[2]);
            iPt[i].xRad = (DiPt[i].xRad);
            iPt[i].yRad = (DiPt[i].yRad);
	  }
	gQuarterPi_ = pi() * .25L;
	gSqrtOfTwo_ = sqrt(2.L);
	theBestCos = -1.L;
        numberOfTrans = 0;
        goodPermutes = 0;
        for (index = 0; index < 24; index++)
	  {
	    a = permuted[index][0];
	    b = permuted[index][1];
	    c = permuted[index][2];
	    d = permuted[index][3];
	    angdist = aDist( &iPt[a], &iPt[b], &iPt[c] );
	    if (angdist < minAngDistance)
	      continue;
	    perpend = pDist( &iPt[a], &iPt[b], &iPt[c] );
	    if (perpend < minDistance)
	      continue;
	    numberOfTrans = From3PtsToTrans(pLgMaster,&iPt[a], &iPt[b], &iPt[c],
					    deltaXHeight, (transform *)&trFor3Pts);
	    if (numberOfTrans == 0)
	      continue;
	    tan4thPtX = tan ( iPt[d].xRad );
	    tan4thPtY = tan ( iPt[d].yRad );
	    sec4thPtSq = Square(tan4thPtX) + Square(tan4thPtY) + 1.L;
	    theBestCos = -1.L;
	    while (numberOfTrans--)
	      {
		TransformPoint ( &trFor3Pts[numberOfTrans],
				 iPt[d].oldLoc, newPoint );
		GetAngles (pLgMaster, (long double *)&newPoint[0], &xa, &ya, deltaXHeight );
		newPoint[0] = tan(xa);
		newPoint[1] = tan(ya);
		tempBestCos = ((newPoint[0]*tan4thPtX) + (newPoint[1]*tan4thPtY) + 1.L)
		  / (sqrt(sec4thPtSq * (Square(newPoint[0]) + Square(newPoint[1]) + 1.L)));
		if (tempBestCos > theBestCos)
		  {
		    theBestCos = tempBestCos;
		    // copy over best transform
		    // trBestForEach3 = trFor3Pts[numberOfTrans];
		    i = 3;
		    while (i--)
		      {
			trBestForEach3.transVector[i] = trFor3Pts[numberOfTrans].transVector[i];
			j = 3;
			while (j--)
			  trBestForEach3.rotMatrix[i][j] = trFor3Pts[numberOfTrans].rotMatrix[i][j];
		      }
		  }
	      }
	    if (fabs(1.L - theBestCos) < tolerance)
	      goodPermutes++;
	    if ((double)theBestCos > *bestCosine)
	      {
		*bestCosine = (double)theBestCos;
		i = 3;
		while (i--)
		  {
		    tr->transVector[i] = trBestForEach3.transVector[i];
		    j = 3;
		    while (j--)
		      tr->rotMatrix[i][j] = trBestForEach3.rotMatrix[i][j];
		  }
	      }
	  }
        numberOfTrans = goodPermutes;
        if (*bestCosine <= 0.L)
	  return(false);
	/* At this point we found the best transform for each			*/
	/* three point combination of four input points					*/
	/* Now we are averaging them (don't worry, it's not all			*/
	/* we'll do). */
        if ((double)theBestCos > *bestCosine)
	  *bestCosine = (double)(theBestCos);
        if (numberOfTrans == 0)
	  return(false);
	/* The problem is that the tr->rotMatrix generally is not	*/
	/* of the type that describe rotation.  What follows should	*/
	/* alter it so it is a rotating matrix.				*/
	if (Determinant3D(tr->rotMatrix) <= 0.L)
	  return(false);
	MakeARotMatrix(tr->rotMatrix);
	i = 4;
	while (i--)
	  {
	    tan4thPtX = tan(iPt[i].xRad);
	    tan4thPtY = tan(iPt[i].yRad);
	    sec4thPtSq = Square(tan4thPtX) + Square(tan4thPtY) + 1.L;
	    TransformPoint(tr, iPt[i].oldLoc, newPoint);
	    GetAngles(pLgMaster, (long double *)&newPoint[0], &xa, &ya, deltaXHeight);
	    newPoint[0] = tan(xa);
	    newPoint[1] = tan(ya);
	    tempBestCos =
	      ( (newPoint[0]*tan4thPtX) + (newPoint[1]*tan4thPtY) + 1.L )
	      /
              sqrt(
                     sec4thPtSq *
                       (  Square( newPoint[0] )
                        + Square( newPoint[1] )
                        + 1.L
                       )
                   );

		if (fabs(1.L - tempBestCos) > tolerance)
		  return(false);
 	  }
        Dtr->rotMatrix[0][0] = (double)(tr->rotMatrix[0][0]);
        Dtr->rotMatrix[0][1] = (double)(tr->rotMatrix[0][1]);
        Dtr->rotMatrix[0][2] = (double)(tr->rotMatrix[0][2]);
        Dtr->rotMatrix[1][0] = (double)(tr->rotMatrix[1][0]);
        Dtr->rotMatrix[1][1] = (double)(tr->rotMatrix[1][1]);
        Dtr->rotMatrix[1][2] = (double)(tr->rotMatrix[1][2]);
        Dtr->rotMatrix[2][0] = (double)(tr->rotMatrix[2][0]);
        Dtr->rotMatrix[2][1] = (double)(tr->rotMatrix[2][1]);
        Dtr->rotMatrix[2][2] = (double)(tr->rotMatrix[2][2]);
        Dtr->transVector[0] = (double)(tr->transVector[0]);
        Dtr->transVector[1] = (double)(tr->transVector[1]);
        Dtr->transVector[2] = (double)(tr->transVector[2]);


	return(true);
}

/* Works only for matrices with det > 0 */
/* Makes a rotational matrix from a near-rotational one*/
/* Modified to consider the rows, not columns, to be the vectors */
/* being transformed */
void MakeARotMatrix ( long double m[3][3] )
{
	long double rotAxis[3], rotAxisAbs, rotAxisSq[3], averAxis[3];
	long double rotMatrix1[3][3], rotMatrix2[3][3], tempVect[3];
	long double temp, sinD;
	
	rotAxis[0] = ( m[0][0] + m[1][0] + m[2][0] ) / 3.L;
	rotAxis[1] = ( m[0][1] + m[1][1] + m[2][1] ) / 3.L;
	rotAxis[2] = ( m[0][2] + m[1][2] + m[2][2] ) / 3.L;

	rotAxisSq[0] = Square ( rotAxis[0] );
	rotAxisSq[1] = Square ( rotAxis[1] );
	rotAxisSq[2] = Square ( rotAxis[2] );
	
	rotAxisAbs = sqrt ( rotAxisSq[0] + rotAxisSq[1] + rotAxisSq[2] );
	
	rotAxis[0] /= rotAxisAbs;
	rotAxis[1] /= rotAxisAbs;
	rotAxis[2] /= rotAxisAbs;

	/* Now rotAxis is the average of the three suggested axes */

	sinD = -sqrt ( 0.75L ); /* D = -120 degrees */
	
	tempVect[0] = sinD * rotAxis[0];
	tempVect[1] = sinD * rotAxis[1];
	tempVect[2] = sinD * rotAxis[2];
	
	rotMatrix1[0][0] = 1.5L * rotAxis[0] * rotAxis[0] - 0.5L;
	rotMatrix1[0][1] = 1.5L * rotAxis[0] * rotAxis[1] - tempVect[2];
	rotMatrix1[0][2] = 1.5L * rotAxis[0] * rotAxis[2] + tempVect[1];
	
	rotMatrix1[1][0] = 1.5L * rotAxis[1] * rotAxis[0] + tempVect[2];
	rotMatrix1[1][1] = 1.5L * rotAxis[1] * rotAxis[1] - 0.5L;
	rotMatrix1[1][2] = 1.5L * rotAxis[1] * rotAxis[2] - tempVect[0];
	
	rotMatrix1[2][0] = 1.5L * rotAxis[2] * rotAxis[0] - tempVect[1];
	rotMatrix1[2][1] = 1.5L * rotAxis[2] * rotAxis[1] + tempVect[0];
	rotMatrix1[2][2] = 1.5L * rotAxis[2] * rotAxis[2] - 0.5L;
	
	rotMatrix2[0][0] = rotMatrix1[0][0];
	rotMatrix2[0][1] = rotMatrix1[1][0];
	rotMatrix2[0][2] = rotMatrix1[2][0];
	
	rotMatrix2[1][0] = rotMatrix1[0][1];
	rotMatrix2[1][1] = rotMatrix1[1][1];
	rotMatrix2[1][2] = rotMatrix1[2][1];
	
	rotMatrix2[2][0] = rotMatrix1[0][2];
	rotMatrix2[2][1] = rotMatrix1[1][2];
	rotMatrix2[2][2] = rotMatrix1[2][2];

	/*now rotMatrix1 = 1/rotMatrix2*/

	/* yAxis -> xAxis */

	Multiply3DMatrixByVector ( rotMatrix1, m[1], tempVect );
	
	averAxis[0] = m[0][0] + tempVect[0];
	averAxis[1] = m[0][1] + tempVect[1];
	averAxis[2] = m[0][2] + tempVect[2];

	/* zAxis -> xAxis */

	Multiply3DMatrixByVector ( rotMatrix2, m[2], tempVect );
	
	averAxis[0] += tempVect[0];
	averAxis[1] += tempVect[1];
	averAxis[2] += tempVect[2];
	
	temp = averAxis[0]*rotAxis[0] + averAxis[1]*rotAxis[1] +
		averAxis[2]*rotAxis[2];
	tempVect[0] = averAxis[0] - rotAxis[0] * temp;
	tempVect[1] = averAxis[1] - rotAxis[1] * temp;
	tempVect[2] = averAxis[2] - rotAxis[2] * temp;
	
	sinD = sqrt ( 3.L );
	temp = sqrt ( 2.L / ( Square ( tempVect[0] ) +
		Square ( tempVect[1] ) + Square ( tempVect[2] ) ) );
	averAxis[0] = ( rotAxis[0] + temp * tempVect[0] ) / sinD;
	averAxis[1] = ( rotAxis[1] + temp * tempVect[1] ) / sinD;
	averAxis[2] = ( rotAxis[2] + temp * tempVect[2] ) / sinD;
	
	m[0][0] = averAxis[0];
	m[0][1] = averAxis[1];
	m[0][2] = averAxis[2];

	/* xAxis -> yAxis */
	Multiply3DMatrixByVector ( rotMatrix2, averAxis, tempVect );

	m[1][0] = tempVect[0];
	m[1][1] = tempVect[1];
	m[1][2] = tempVect[2];

	/* xAxis -> zAxis */
	Multiply3DMatrixByVector ( rotMatrix1, averAxis, tempVect );

	m[2][0] = tempVect[0];
	m[2][1] = tempVect[1];
	m[2][2] = tempVect[2];
}


/********************************************************************/
/*	From three inputPoints From3PtsToTrans derives several			*/
/*	transforms. We consider only those for which the newZ is		*/
/*	less or equal zero (so that it is looked from above). If the	*/
/*	other four trans are needed, just mutiply rotMatrix and			*/
/*	transVector by -1. The short output says how many transforms	*/
/*	is actually found. 												*/
/********************************************************************/

long double ZfromR(struct lg_master *pLgMaster, long double R, long double xa, long double ya,
		long double deltaXHeight )
{
	long double cosX, cosY, sinX, sinY, fX, fY, d;
	cosX = cos ( xa );
	cosY = cos ( ya );
	sinX = sin ( xa );
	sinY = sin ( ya );
	fX = pLgMaster->gHalfMirror * MirrorFactor_(xa);
	fY = pLgMaster->gHalfMirror * MirrorFactor_(ya);
	d = deltaXHeight - (pLgMaster->gHalfMirror * MirrorOffset_(ya));
	return ((d * Square(sinX) * fabs(cosY))
		- (fabs(cosX * cosY) * sqrt(Square(R) - Square(d * sinX))
		   - (Square(fY) * (1.L - Square(cosX * sinY)))
		   - Square(fX * cosX)
		   + (2.L * d * fX * sinX * cosX)
		   - (2.L * d * fY * Square(sinX) * sinY * ((cosY < 0.L) ? -1.L : 1.L))
		   + (2.L * fX * fY * cosX * sinX * sinY * ((cosY < 0.L) ? -1.L : 1.L)))
		- (fY * Square(cosX) * cosY * sinY)
		- (fX * fabs(cosY) * sinX * cosX));
}

long double CoeffX(struct lg_master *pLgMaster, long double z, long double xa, long double ya,
		long double deltaXHeight )
{
    if (fabs(z) > LDBL_MIN)
      return(tan(xa) * (1.L / fabs(cos(ya))
			+ fabs((deltaXHeight - pLgMaster->gHalfMirror * MirrorOffset_(ya)) / z))
	     + pLgMaster->gHalfMirror * MirrorFactor_(xa) / z);
    return(tan(xa) / fabs(cos(ya)));
}

long double CoeffY(struct lg_master *pLgMaster, long double z, long double ya )
{
    if (fabs(z) > LDBL_MIN)
      return(tan(ya) + pLgMaster->gHalfMirror * MirrorFactor_(ya) / z);
    return (tan(ya));
}

static short From3PtsToTrans(struct lg_master *pLgMaster, inputPoint *iPt0, inputPoint *iPt1,
			     inputPoint *iPt2, long double deltaXHeight, transform *tr)
{	
	long double secSq0, secSq1, secSq2;
	long double a01, a02, a12;
	long double ax0, ax1, ax2;
	long double ay0, ay1, ay2;
	long double tempZ0, tempZ1, tempZ2;
	
	long double minDr, tempDr, maxR[3];
	
	short i, j, k, bestJ, iterN, outputNumber;
	Boolean fBetterRisFound;
	
	long double newRs[16][3], tempRs[16][3];
	/* newR = sqrt ( newX**2 + newY**2+ newZ**2 ) */

	long double oPt0[3], oPt1[3], oPt2[3];
	/* output points */
	/* (newX0,newY0,newZ0), (newX1,newY1,newZ1), (newX2,newY2,newZ2) */

	long double oPtD01[3], oPtD02[3]; /* output point differences */
	/* oPtD01: (newX1,newY1,newZ1) - (newX0,newY0,newZ0)*/
	/* oPtD02: (newX2,newY2,newZ2) - (newX0,newY0,newZ0)*/

	long double iPtD01[3], iPtD02[3]; /* input point differences */
	/* iPtD01: (oldX1,oldY1,oldZ1) - (oldX0,oldY0,oldZ0)*/
	/* iPtD02: (oldX2,oldY2,oldZ2) - (oldX0,oldY0,oldZ0)*/

	long double sqd01, sqd02, sqd12;
	/* squares of distances between points */
	
	
	iPtD01[0] = ( iPt1->oldLoc[0] - iPt0->oldLoc[0] );
	iPtD01[1] = ( iPt1->oldLoc[1] - iPt0->oldLoc[1] );
	iPtD01[2] = ( iPt1->oldLoc[2] - iPt0->oldLoc[2] );
	
	iPtD02[0] = ( iPt2->oldLoc[0] - iPt0->oldLoc[0] );
	iPtD02[1] = ( iPt2->oldLoc[1] - iPt0->oldLoc[1] );
	iPtD02[2] = ( iPt2->oldLoc[2] - iPt0->oldLoc[2] );

	sqd01 = Square ( iPtD01[0] ) + Square ( iPtD01[1] ) +
		Square ( iPtD01[2] );
	if ( fabs ( sqd01 ) <= LDBL_MIN ) return 0;
	sqd02 = Square ( iPtD02[0] ) + Square ( iPtD02[1] ) +
		Square ( iPtD02[2] );
	if ( fabs ( sqd02 ) <= LDBL_MIN ) return 0;
	
	if ( ( (
		Square ( iPtD01[1] * iPtD02[2] - iPtD01[2] * iPtD02[1] ) +
		Square ( iPtD01[2] * iPtD02[0] - iPtD01[0] * iPtD02[2] ) +
		Square ( iPtD01[0] * iPtD02[1] - iPtD01[1] * iPtD02[0] ) ) /
		sqrt ( sqd01 * sqd02 ) ) <= LDBL_MIN ) return 0;
	
	sqd12 =
		Square ( iPt2->oldLoc[0] - iPt1->oldLoc[0] ) +
		Square ( iPt2->oldLoc[1] - iPt1->oldLoc[1] ) +
		Square ( iPt2->oldLoc[2] - iPt1->oldLoc[2] );

	ax0 = tan ( iPt0->xRad ) / fabs ( cos ( iPt0->yRad ) );
	ay0 = tan ( iPt0->yRad );

	ax1 = tan ( iPt1->xRad ) / fabs ( cos ( iPt1->yRad ) );
	ay1 = tan ( iPt1->yRad );

	ax2 = tan ( iPt2->xRad ) / fabs ( cos ( iPt2->yRad ) );
	ay2 = tan ( iPt2->yRad );

	secSq0 = Square ( ax0 ) + Square ( ay0 ) + 1.L;
	secSq1 = Square ( ax1 ) + Square ( ay1 ) + 1.L;
	secSq2 = Square ( ax2 ) + Square ( ay2 ) + 1.L;
	
	a01 = ( ax0 * ax1 + ay0 * ay1 + 1.L ) / sqrt ( secSq0 * secSq1 );
	a02 = ( ax0 * ax2 + ay0 * ay2 + 1.L ) / sqrt ( secSq0 * secSq2 );
	a12 = ( ax1 * ax2 + ay1 * ay2 + 1.L ) / sqrt ( secSq1 * secSq2 );
	
	outputNumber = SolveTheSystem
		( a01, sqd01, a02, sqd02, a12, sqd12, newRs );

	k = 3;
	while ( k-- )
	{
		if ( outputNumber == 0 ) return 0;
		
		j = 3;
		while ( j-- )
		{
			maxR[j] = 0.L;
			i = outputNumber;
			while ( i-- ) maxR[j] += newRs[i][j];
			maxR[j] /= (long double)outputNumber;
		}
		
		tempZ0 = ZfromR(pLgMaster, maxR[0], iPt0->xRad, iPt0->yRad, deltaXHeight);
		ax0 = CoeffX(pLgMaster, tempZ0, iPt0->xRad, iPt0->yRad, deltaXHeight);
		ay0 = CoeffY(pLgMaster, tempZ0, iPt0->yRad);

		tempZ1 = ZfromR(pLgMaster, maxR[1], iPt1->xRad, iPt1->yRad, deltaXHeight);
		ax1 = CoeffX(pLgMaster, tempZ1, iPt1->xRad, iPt1->yRad, deltaXHeight);
		ay1 = CoeffY(pLgMaster, tempZ1, iPt1->yRad);

		tempZ2 = ZfromR(pLgMaster, maxR[2], iPt2->xRad, iPt2->yRad, deltaXHeight);
		ax2 = CoeffX(pLgMaster, tempZ2, iPt2->xRad, iPt2->yRad, deltaXHeight);
		ay2 = CoeffY(pLgMaster, tempZ2, iPt2->yRad);
		
		secSq0 = Square(ax0) + Square(ay0) + 1.L;
		secSq1 = Square(ax1) + Square(ay1) + 1.L;
		secSq2 = Square(ax2) + Square(ay2) + 1.L;
		
		a01 = ( ax0 * ax1 + ay0 * ay1 + 1.L ) / sqrt ( secSq0 * secSq1 );
		a02 = ( ax0 * ax2 + ay0 * ay2 + 1.L ) / sqrt ( secSq0 * secSq2 );
		a12 = ( ax1 * ax2 + ay1 * ay2 + 1.L ) / sqrt ( secSq1 * secSq2 );
		
		outputNumber = SolveTheSystem
			( a01, sqd01, a02, sqd02, a12, sqd12, tempRs );
	}

	i = outputNumber;
	while (i--)
	  {
	    iterN = 0;
	    do
	      {
		tempZ0 = ZfromR(pLgMaster, newRs[i][0], iPt0->xRad, iPt0->yRad, deltaXHeight);
		ax0 = CoeffX(pLgMaster, tempZ0, iPt0->xRad, iPt0->yRad, deltaXHeight);
		ay0 = CoeffY(pLgMaster, tempZ0, iPt0->yRad);

		tempZ1 = ZfromR(pLgMaster, newRs[i][1], iPt1->xRad, iPt1->yRad, deltaXHeight);
		ax1 = CoeffX(pLgMaster, tempZ1, iPt1->xRad, iPt1->yRad, deltaXHeight);
		ay1 = CoeffY(pLgMaster, tempZ1, iPt1->yRad);

		tempZ2 = ZfromR(pLgMaster, newRs[i][2], iPt2->xRad, iPt2->yRad, deltaXHeight);
		ax2 = CoeffX(pLgMaster, tempZ2, iPt2->xRad, iPt2->yRad, deltaXHeight);
		ay2 = CoeffY(pLgMaster, tempZ2, iPt2->yRad);

		secSq0 = Square ( ax0 ) + Square ( ay0 ) + 1.L;
		secSq1 = Square ( ax1 ) + Square ( ay1 ) + 1.L;
		secSq2 = Square ( ax2 ) + Square ( ay2 ) + 1.L;
			
		a01 = ((ax0 * ax1) + (ay0 * ay1) + 1.L) / sqrt(secSq0 * secSq1);
		a02 = ((ax0 * ax2) + (ay0 * ay2) + 1.L) / sqrt(secSq0 * secSq2);
		a12 = ((ax1 * ax2) + (ay1 * ay2) + 1.L) / sqrt(secSq1 * secSq2);
			
		j = SolveTheSystem(a01, sqd01, a02, sqd02, a12, sqd12, tempRs);
			
		minDr = 3.L;
		fBetterRisFound = false;
		while (j--)
		  {	
		    tempDr = 0.L;
		    k = 3;
		    while (k--)
		      {
			if (tempRs[j][k] > newRs[i][k])
			  {
			    if (tempRs[j][k] > LDBL_MIN ) tempDr +=
							    fabs ( tempRs[j][k] - newRs[i][k] ) /
							    tempRs[j][k];
			    else tempDr += fabs ( tempRs[j][k] - newRs[i][k] );
			  }
			else
			  {
			    if (newRs[i][k] > LDBL_MIN) tempDr +=
							  fabs ( tempRs[j][k] - newRs[i][k] ) /
							  newRs[i][k];
			    else tempDr += fabs ( tempRs[j][k] - newRs[i][k] );
			  }
		      }
		    if (tempDr < minDr)
		      {
			fBetterRisFound = true;
			bestJ = j;
			minDr = tempDr;
		      }
		  }
		if (fBetterRisFound)
		  {
		    newRs[i][0] = tempRs[bestJ][0];
		    newRs[i][1] = tempRs[bestJ][1];
		    newRs[i][2] = tempRs[bestJ][2];
		  }
	      }
	    while ((minDr > kReallySmall) && (iterN++ < kMaxIter));
		
	    if ( minDr > kReallySmall )
	      {
		j = i;
		while ( ++j < outputNumber );
		{
		  newRs[j-1][0] = newRs[j][0];
		  newRs[j-1][1] = newRs[j][1];
		  newRs[j-1][2] = newRs[j][2];
		}
		outputNumber--;
	      }
	  }
	
	sqd01 = sqrt ( sqd01 );
	iPtD01[0] /= sqd01;
	iPtD01[1] /= sqd01;
	iPtD01[2] /= sqd01;
	
	sqd02 = sqrt ( sqd02 );
	iPtD02[0] /= sqd02;
	iPtD02[1] /= sqd02;
	iPtD02[2] /= sqd02;
		
	i = outputNumber;
	while ( i-- )
	  {
	    oPt0[2] = ZfromR(pLgMaster, newRs[i][0], iPt0->xRad, iPt0->yRad, deltaXHeight);
	    oPt1[2] = ZfromR(pLgMaster, newRs[i][1], iPt1->xRad, iPt1->yRad, deltaXHeight);
	    oPt2[2] = ZfromR(pLgMaster, newRs[i][2], iPt2->xRad, iPt2->yRad, deltaXHeight );
	    oPt0[0] = oPt0[2] * tan(iPt0->xRad) *
	      (1.L / fabs(cos(iPt0->yRad)) + fabs((deltaXHeight -
						   pLgMaster->gHalfMirror * MirrorOffset_(iPt0->yRad)) /
						  oPt0[2]))
	      + pLgMaster->gHalfMirror * MirrorFactor_(iPt0->xRad);
	    oPt0[1] = oPt0[2] * tan(iPt0->yRad) +
	      pLgMaster->gHalfMirror * MirrorFactor_(iPt0->yRad);
									
	    oPt1[0] = oPt1[2] * tan(iPt1->xRad)
	      * (1.L / fabs(cos(iPt1->yRad))
		 + fabs((deltaXHeight - pLgMaster->gHalfMirror * MirrorOffset_(iPt1->yRad)) / oPt1[2]))
	      + pLgMaster->gHalfMirror * MirrorFactor_(iPt1->xRad);
	    oPt1[1] = oPt1[2] * tan(iPt1->yRad) +
	      pLgMaster->gHalfMirror * MirrorFactor_(iPt1->yRad);
	    oPt2[0] = oPt2[2] * tan(iPt2->xRad) *
	      (1.L / fabs(cos(iPt2->yRad)) +
	       fabs((deltaXHeight -
		     pLgMaster->gHalfMirror * MirrorOffset_(iPt2->yRad)) / oPt2[2]))
	      + pLgMaster->gHalfMirror * MirrorFactor_(iPt2->xRad);
	    oPt2[1] = oPt2[2] * tan(iPt2->yRad) +
	      pLgMaster->gHalfMirror * MirrorFactor_(iPt2->yRad);
		
		oPtD01[0] = ( oPt1[0] - oPt0[0] );
		oPtD01[1] = ( oPt1[1] - oPt0[1] );
		oPtD01[2] = ( oPt1[2] - oPt0[2] );
		
		oPtD02[0] = ( oPt2[0] - oPt0[0] );
		oPtD02[1] = ( oPt2[1] - oPt0[1] );
		oPtD02[2] = ( oPt2[2] - oPt0[2] );
		
		sqd01 = sqrt ( Square ( oPtD01[0] ) + Square ( oPtD01[1] ) + 
			Square ( oPtD01[2] ) );

		oPtD01[0] /= sqd01;
		oPtD01[1] /= sqd01;
		oPtD01[2] /= sqd01;
		
		sqd02 = sqrt ( Square ( oPtD02[0] ) + Square ( oPtD02[1] ) + 
			Square ( oPtD02[2] ) );
		
		oPtD02[0] /= sqd02;
		oPtD02[1] /= sqd02;
		oPtD02[2] /= sqd02;
		
		FindRotMatrix
			( iPtD01, iPtD02, oPtD01, oPtD02, tr[i].rotMatrix );
		
		Multiply3DMatrixByVector
			( tr[i].rotMatrix, iPt0->oldLoc, tr[i].transVector );
		tr[i].transVector[0] = oPt0[0] - tr[i].transVector[0];
		tr[i].transVector[1] = oPt0[1] - tr[i].transVector[1];
		tr[i].transVector[2] = oPt0[2] - tr[i].transVector[2];
	}
	return outputNumber;
}


/********************************************************************/
/*	This function solves the following system						*/
/*	r[0] >= 0;										*/
/*	r[0]**2 - 2*a01*r[0]*r[1] + r[1]**2 = sqd01						*/
/*	r[0]**2 - 2*a02*r[0]*r[2] + r[2]**2 = sqd02						*/
/*	r[1]**2 - 2*a12*r[1]*r[2] + r[2]**2 = sqd12						*/
/*	and returns the number of real solutions.						*/
/*	Actually there are four of them.							*/
/*	But some pathological cases might produce up to					*/
/*	16(!!!), some of which are dupli-, or quadruplated.				*/
/*	Let's keep them all for simplicity.							*/
/********************************************************************/
short SolveTheSystem ( long double a01, long double sqd01,
	long double a02, long double sqd02,
	long double a12, long double sqd12,
	long double rs[16][3] )
{
	/* The resulting equation is */
	/* t*r[0]**4 + a*r[0]**3 + b*r[0]**2 + c*r[0] + d = 0 */
	
	long double a, b, c, d, t, r[4];
	long double a2, b2, c2, d2, t2;
	long double a01p2, a02p2, a12p2, sqd01p2, sqd02p2, sqd12p2;
	long double a01p3, a02p3, a12p3, sqd01p3, sqd02p3, sqd12p3;
	long double a01p4, a02p4, a12p4, sqd01p4, sqd02p4, sqd12p4;
	
	long double det01, det02, r0Temp, r1Temp1, r1Temp2, r2Temp;

	long double rootCoefficient, rootCoefficientTemp;
	
	short i, numberOfSolutions, numberOfRoots;
	
	a01p2 = a01 * a01;
	a02p2 = a02 * a02;
	a12p2 = a12 * a12;

	a01p3 = a01p2 * a01;
	a02p3 = a02p2 * a02;
	a12p3 = a12p2 * a12;

	a01p4 = a01p2 * a01p2;
	a02p4 = a02p2 * a02p2;
	a12p4 = a12p2 * a12p2;
	
	sqd01p2 = sqd01 * sqd01;
	sqd02p2 = sqd02 * sqd02;
	sqd12p2 = sqd12 * sqd12;

	sqd01p3 = sqd01p2 * sqd01;
	sqd02p3 = sqd02p2 * sqd02;
	sqd12p3 = sqd12p2 * sqd12;

	sqd01p4 = sqd01p2 * sqd01p2;
	sqd02p4 = sqd02p2 * sqd02p2;
	sqd12p4 = sqd12p2 * sqd12p2;

	/* Thank you Mathematica 2.2 */

	t = 
		16*(1 - 2*a01p2 + a01p4 - 2*a02p2 + 2*a01p2*a02p2 + a02p4 +
		4*a01*a02*a12 - 4*a01p3*a02*a12 - 4*a01*a02p3*a12 - 2*a12p2
		+ 2*a01p2*a12p2 + 2*a02p2*a12p2 + 4*a01p2*a02p2*a12p2 -
		4*a01*a02*a12p3 + a12p4);
	
	a = 
		32*(-sqd01 + a01p2*sqd01 + 2*a02p2*sqd01 - a01p2*a02p2*sqd01
		- a02p4*sqd01 - 3*a01*a02*a12*sqd01 + a01p3*a02*a12*sqd01 +
		3*a01*a02p3*a12*sqd01 + 2*a12p2*sqd01 - a01p2*a12p2*sqd01 -
		2*a02p2*a12p2*sqd01 - 2*a01p2*a02p2*a12p2*sqd01 +
		3*a01*a02*a12p3*sqd01 - a12p4*sqd01 - sqd02 + 2*a01p2*sqd02
		- a01p4*sqd02 + a02p2*sqd02 - a01p2*a02p2*sqd02 -
		3*a01*a02*a12*sqd02 + 3*a01p3*a02*a12*sqd02 +
		a01*a02p3*a12*sqd02 + 2*a12p2*sqd02 - 2*a01p2*a12p2*sqd02 -
		a02p2*a12p2*sqd02 - 2*a01p2*a02p2*a12p2*sqd02 +
		3*a01*a02*a12p3*sqd02 - a12p4*sqd02 + sqd12 - 2*a01p2*sqd12
		+ a01p4*sqd12 - 2*a02p2*sqd12 + 4*a01p2*a02p2*sqd12 -
		2*a01p4*a02p2*sqd12 + a02p4*sqd12 - 2*a01p2*a02p4*sqd12 +
		a01*a02*a12*sqd12 - a01p3*a02*a12*sqd12 - a01*a02p3*a12*sqd12 +
		4*a01p3*a02p3*a12*sqd12 - a12p2*sqd12 + a01p2*a12p2*sqd12 +
		a02p2*a12p2*sqd12 - 4*a01p2*a02p2*a12p2*sqd12 +
		a01*a02*a12p3*sqd12);
		
	b = 
		8*(3*sqd01p2 - a01p2*sqd01p2 - 5*a02p2*sqd01p2 +
		2*a01p2*a02p2*sqd01p2 + 2*a02p4*sqd01p2 + 2*a01*a02*a12*sqd01p2
		- 4*a01*a02p3*a12*sqd01p2 - 5*a12p2*sqd01p2 +
		2*a01p2*a12p2*sqd01p2 + 6*a02p2*a12p2*sqd01p2 -
		4*a01*a02*a12p3*sqd01p2 + 2*a12p4*sqd01p2 + 6*sqd01*sqd02 -
		6*a01p2*sqd01*sqd02 - 6*a02p2*sqd01*sqd02 +
		20*a01*a02*a12*sqd01*sqd02 - 4*a01p3*a02*a12*sqd01*sqd02 -
		4*a01*a02p3*a12*sqd01*sqd02 - 14*a12p2*sqd01*sqd02 +
		4*a01p2*a12p2*sqd01*sqd02 + 4*a02p2*a12p2*sqd01*sqd02 +
		8*a01p2*a02p2*a12p2*sqd01*sqd02 - 16*a01*a02*a12p3*sqd01*sqd02
		+ 8*a12p4*sqd01*sqd02 + 3*sqd02p2 - 5*a01p2*sqd02p2 +
		2*a01p4*sqd02p2 - a02p2*sqd02p2 + 2*a01p2*a02p2*sqd02p2 +
		2*a01*a02*a12*sqd02p2 - 4*a01p3*a02*a12*sqd02p2 -
		5*a12p2*sqd02p2 + 6*a01p2*a12p2*sqd02p2 + 2*a02p2*a12p2*sqd02p2
		- 4*a01*a02*a12p3*sqd02p2 + 2*a12p4*sqd02p2 - 6*sqd01*sqd12
		+ 6*a01p2*sqd01*sqd12 + 10*a02p2*sqd01*sqd12 -
		8*a01p2*a02p2*sqd01*sqd12 - 4*a02p4*sqd01*sqd12 -
		4*a01*a02*a12*sqd01*sqd12 - 4*a01p3*a02*a12*sqd01*sqd12 +
		8*a01*a02p3*a12*sqd01*sqd12 + 6*a12p2*sqd01*sqd12 -
		8*a02p2*a12p2*sqd01*sqd12 + 8*a01p2*a02p2*a12p2*sqd01*sqd12
		- 4*a01*a02*a12p3*sqd01*sqd12 - 6*sqd02*sqd12 +
		10*a01p2*sqd02*sqd12 - 4*a01p4*sqd02*sqd12 +
		6*a02p2*sqd02*sqd12 - 8*a01p2*a02p2*sqd02*sqd12 -
		4*a01*a02*a12*sqd02*sqd12 + 8*a01p3*a02*a12*sqd02*sqd12 -
		4*a01*a02p3*a12*sqd02*sqd12 + 6*a12p2*sqd02*sqd12 -
		8*a01p2*a12p2*sqd02*sqd12 + 8*a01p2*a02p2*a12p2*sqd02*sqd12
		- 4*a01*a02*a12p3*sqd02*sqd12 + 3*sqd12p2 - 5*a01p2*sqd12p2
		+ 2*a01p4*sqd12p2 - 5*a02p2*sqd12p2 + 6*a01p2*a02p2*sqd12p2
		+ 2*a02p4*sqd12p2 + 2*a01*a02*a12*sqd12p2 -
		4*a01p3*a02*a12*sqd12p2 - 4*a01*a02p3*a12*sqd12p2 -
		a12p2*sqd12p2 + 2*a01p2*a12p2*sqd12p2 + 2*a02p2*a12p2*sqd12p2);
	
	c = 
		8*(-sqd01p3 + a02p2*sqd01p3 + a01*a02*a12*sqd01p3 +
		a12p2*sqd01p3 - 2*a02p2*a12p2*sqd01p3 - 3*sqd01p2*sqd02 +
		a01p2*sqd01p2*sqd02 + 2*a02p2*sqd01p2*sqd02 -
		5*a01*a02*a12*sqd01p2*sqd02 + 7*a12p2*sqd01p2*sqd02 -
		2*a01p2*a12p2*sqd01p2*sqd02 + 4*a01*a02*a12p3*sqd01p2*sqd02
		- 4*a12p4*sqd01p2*sqd02 - 3*sqd01*sqd02p2 +
		2*a01p2*sqd01*sqd02p2 + a02p2*sqd01*sqd02p2 -
		5*a01*a02*a12*sqd01*sqd02p2 + 7*a12p2*sqd01*sqd02p2 -
		2*a02p2*a12p2*sqd01*sqd02p2 + 4*a01*a02*a12p3*sqd01*sqd02p2
		- 4*a12p4*sqd01*sqd02p2 - sqd02p3 + a01p2*sqd02p3 +
		a01*a02*a12*sqd02p3 + a12p2*sqd02p3 - 2*a01p2*a12p2*sqd02p3
		+ 3*sqd01p2*sqd12 - a01p2*sqd01p2*sqd12 - 3*a02p2*sqd01p2*sqd12
		- a01*a02*a12*sqd01p2*sqd12 - 2*a12p2*sqd01p2*sqd12 +
		4*a02p2*a12p2*sqd01p2*sqd12 + 6*sqd01*sqd02*sqd12 -
		4*a01p2*sqd01*sqd02*sqd12 - 4*a02p2*sqd01*sqd02*sqd12 +
		6*a01*a02*a12*sqd01*sqd02*sqd12 - 8*a12p2*sqd01*sqd02*sqd12
		+ 4*a01*a02*a12p3*sqd01*sqd02*sqd12 + 3*sqd02p2*sqd12 -
		3*a01p2*sqd02p2*sqd12 - a02p2*sqd02p2*sqd12 -
		a01*a02*a12*sqd02p2*sqd12 - 2*a12p2*sqd02p2*sqd12 +
		4*a01p2*a12p2*sqd02p2*sqd12 - 3*sqd01*sqd12p2 +
		2*a01p2*sqd01*sqd12p2 + 3*a02p2*sqd01*sqd12p2 -
		a01*a02*a12*sqd01*sqd12p2 + a12p2*sqd01*sqd12p2 -
		2*a02p2*a12p2*sqd01*sqd12p2 - 3*sqd02*sqd12p2 +
		3*a01p2*sqd02*sqd12p2 + 2*a02p2*sqd02*sqd12p2 -
		a01*a02*a12*sqd02*sqd12p2 + a12p2*sqd02*sqd12p2 -
		2*a01p2*a12p2*sqd02*sqd12p2 + sqd12p3 - a01p2*sqd12p3 -
		a02p2*sqd12p3 + a01*a02*a12*sqd12p3);
		 		
	d =
		sqd01p4 + 4*sqd01p3*sqd02 - 8*a12p2*sqd01p3*sqd02 +
		6*sqd01p2*sqd02p2 - 16*a12p2*sqd01p2*sqd02p2 +
		16*a12p4*sqd01p2*sqd02p2 + 4*sqd01*sqd02p3 -
		8*a12p2*sqd01*sqd02p3 + sqd02p4 - 4*sqd01p3*sqd12 -
		12*sqd01p2*sqd02*sqd12 + 16*a12p2*sqd01p2*sqd02*sqd12 -
		12*sqd01*sqd02p2*sqd12 + 16*a12p2*sqd01*sqd02p2*sqd12 -
		4*sqd02p3*sqd12 + 6*sqd01p2*sqd12p2 + 12*sqd01*sqd02*sqd12p2 -
		8*a12p2*sqd01*sqd02*sqd12p2 + 6*sqd02p2*sqd12p2 -
		4*sqd01*sqd12p3 - 4*sqd02*sqd12p3 + sqd12p4;
			
	t2 = fabs ( t );
	a2 = fabs ( a );
	b2 = fabs ( b );
	c2 = fabs ( c );
	d2 = fabs ( d );
	
	if ( ( ( t2/( a2 + t2 ) ) <= LDBL_MIN ) ||
		( ( t2/( b2 + t2 ) ) <= LDBL_MIN ) ||
		( ( t2/( c2 + t2 ) ) <= LDBL_MIN ) ||
		( ( t2/( d2 + t2 ) ) <= LDBL_MIN ) )
	{
		if ( ( ( a2/( b2 + a2 ) ) <= LDBL_MIN ) ||
			( ( a2/( c2 + a2 ) ) <= LDBL_MIN ) ||
			( ( a2/( d2 + a2 ) ) <= LDBL_MIN ) )
		{
			if ( ( ( b2/( c2 + b2 ) ) <= LDBL_MIN ) ||
				( ( b2/( d2 + b2 ) ) <= LDBL_MIN ) )
			{
				if ( ( c2/( c2 + d2 ) ) <= LDBL_MIN ) return 0;
				else
				{
					numberOfRoots = 1;
					rootCoefficient = 1.L;
					r[0] = -d/c;
				}
			}
			else
			{
				t = b;
				a = c/t;
				b = d/t;
				rootCoefficient = 1.L;
				numberOfRoots = Solve2ndDegree ( a, b, r );
			}
		}
		else
		{
			t = a;
			a = b/t;
			b = c/t;
			c = d/t;
			rootCoefficient = 1.L;
			numberOfRoots = Solve3rdDegree ( a, b, c, r );
		}
	}
	else
	{
		rootCoefficient = sqrt ( sqrt ( fabs ( d / t ) ) );
		if ( rootCoefficient <= LDBL_MIN )
		{
			a = a/t;
			b = b/t;
			c = c/t;
			d = d/t;
			rootCoefficient = 1.L;
		}
		else
		{
			rootCoefficientTemp = sqrt ( fabs ( d * t ) );
			if ( t > 0.L )
			{
				a = a / sqrt ( fabs ( t ) * rootCoefficientTemp );
				b = b / rootCoefficientTemp;
				c = c / sqrt ( fabs ( d ) * rootCoefficientTemp );
				if ( d > 0.L ) d = 1.L;
				else d = -1.L;
			}
			else
			{
				a = -a / sqrt ( fabs ( t ) * rootCoefficientTemp );
				b = -b / rootCoefficientTemp;
				c = -c / sqrt ( fabs ( d ) * rootCoefficientTemp );
				if ( d > 0.L ) d = -1.L;
				else d = 1.L;
			}
		}
		numberOfRoots = Solve4thDegree ( a, b, c, d, r );
	}
	
	i = numberOfRoots;
	numberOfSolutions = 0;
	while ( i-- )
	{
		r[i] *= rootCoefficient;
		if ( ( ( det01 = sqd01 - ( 1.L - a01p2 )*r[i] ) >= 0.L ) &&
			( ( det02 = sqd02 - ( 1.L - a02p2 )*r[i] ) >= 0.L ) )
		{
			/* Catching det01==0 and det02==0 is not very
			   useful, but it's OK */
			det01 = sqrt ( det01 );
			det02 = sqrt ( det02 );
			r0Temp = sqrt ( r[i] );
			r1Temp1 = a01*r0Temp + det01;
			r1Temp2 = a01*r0Temp - det01;
			r2Temp = a02*r0Temp + det02;
			if ( fabs ( ( r1Temp1*r1Temp1 - 2.L*a12*r1Temp1*r2Temp +
				r2Temp*r2Temp )/sqd12 - 1.L ) < kReallySmall )
			{
				rs[numberOfSolutions][0] = r0Temp;
				rs[numberOfSolutions][1] = r1Temp1;
				rs[numberOfSolutions][2] = r2Temp;
				numberOfSolutions++;
			}
			if ( ( fabs ( ( r1Temp2*r1Temp2 - 2.L*a12*r1Temp2*r2Temp +
				r2Temp*r2Temp )/sqd12 - 1.L ) < kReallySmall ) &&
				( det01 != 0.L ) )
				/* The last != almost never helps */
			{
				rs[numberOfSolutions][0] = r0Temp;
				rs[numberOfSolutions][1] = r1Temp2;
				rs[numberOfSolutions][2] = r2Temp;
				numberOfSolutions++;
			}
			if ( det02 != 0.L )
			{
				r2Temp = a02*r0Temp - det02;
				if ( fabs ( ( r1Temp1*r1Temp1 - 2.L*a12*r1Temp1*r2Temp +
					r2Temp*r2Temp )/sqd12 - 1.L ) < kReallySmall )
				{
					rs[numberOfSolutions][0] = r0Temp;
					rs[numberOfSolutions][1] = r1Temp1;
					rs[numberOfSolutions][2] = r2Temp;
					numberOfSolutions++;
				}
				if ( ( fabs ( ( r1Temp2*r1Temp2 - 2.L*a12*r1Temp2*r2Temp +
					r2Temp*r2Temp )/sqd12 - 1.L ) < kReallySmall ) &&
					( det01 != 0.L ) )
				{
					rs[numberOfSolutions][0] = r0Temp;
					rs[numberOfSolutions][1] = r1Temp2;
					rs[numberOfSolutions][2] = r2Temp;
					numberOfSolutions++;
				}
			}
		}
	}
	return numberOfSolutions;
}


/* Solving r**4 + a*r**3 + b*r**2 + c*r + d = 0 */
short Solve4thDegree (
	long double a, long double b, long double c, long double d,
	long double r[4] )
{
	long double r3[3], p, q, rr, a3, b3, c3, a4, a2, b2;
	short numberOf3rdDegreeSolutions;
	
	/* Just some intermediate numbers*/
	p = a * a;
	rr = -0.01171875L * p * p + 0.0625L * p * b - 0.25L * a * c + d;
	q = 0.125L * p * a - 0.5L * a * b + c;
	p = -0.375L * p + b;
	a4 = a * 0.25L;
	
	/* Coefficients for 3rd degree equation to be solved */
	a3 = 2.L * p;
	b3 = p * p - 4.L * rr;
	c3 = -q * q;
		
	numberOf3rdDegreeSolutions = Solve3rdDegree ( a3, b3, c3, r3 );
	if ( numberOf3rdDegreeSolutions == 3)
	{
		if ( ( r3[0] > 0.L ) && ( r3[1] > 0.L ) && ( r3[2] > 0.L ) )
		{
			r3[0] = sqrt ( r3[0] );
			if ( q >= 0.L ) r3[0] = -r3[0];
			r3[1] = sqrt ( r3[1] );
			r3[2] = sqrt ( r3[2] );
			r[0] = 0.5L*( r3[0] + r3[1] + r3[2] ) - a4;
			r[1] = 0.5L*( r3[0] - r3[1] - r3[2] ) - a4;
			r[2] = 0.5L*( r3[1] - r3[0] - r3[2] ) - a4;
			r[3] = 0.5L*( r3[2] - r3[0] - r3[1] ) - a4;
			return 4;
		}

		if ( ( r3[0] < 0.L ) || ( r3[1] < 0.L ) || ( r3[2] < 0.L ) )
			return 0;


/* The probability of an exotic case in the real life is nil */
/* The probability that something in the following code */
/* is not very smart is not nil. Start exotic cases */
		
		if (
			( ( r3[0] == 0.L ) && ( r3[1] > 0.L ) && ( r3[2] > 0.L ) ) ||
			( ( r3[0] > 0.L ) && ( r3[1] == 0.L ) && ( r3[2] > 0.L ) ) ||
			( ( r3[0] > 0.L ) && ( r3[1] > 0.L ) && ( r3[2] == 0.L ) ) )
		{
			if ( r3[0] == 0.L )
			{
				r3[0] = sqrt ( r3[2] );
				r3[1] = sqrt ( r3[1] );
			}
			else 
			{
				r3[0] = sqrt ( r3[0] );
				if ( r3[1] == 0.L ) r3[1] = sqrt ( r3[2] );
				else r3[1] = sqrt ( r3[1] );
			}

			r[0] = 0.5L*( r3[0] + r3[1] ) - a4;
			r[1] = 0.5L*( -r3[0] - r3[1] ) - a4;
			r[2] = r3[0] - r3[1];
			if ( r[2] == 0.L )
			{
				r[2] = -a4;
				return 3;
			}
			else
			{
				r[3] = -r[2] - a4;
				r[2] -= a4;
				return 4;
			}
		}
		
		
		if (
			( ( r3[0] == 0.L ) && ( r3[1] == 0.L ) && ( r3[2] > 0.L ) ) ||
			( ( r3[0] == 0.L ) && ( r3[1] > 0.L ) && ( r3[2] == 0.L ) ) ||
			( ( r3[0] > 0.L ) && ( r3[1] == 0.L ) && ( r3[2] == 0.L ) ) )
		{
			if ( r3[2] > 0.L ) r[0] = 0.5L * sqrt ( r3[2] );
			else
			{
				if ( r3[1] > 0.L ) r[0] = 0.5L*sqrt ( r3[1] );
				else r[0] = 0.5L*sqrt ( r3[0] );
			}
			r[1] = -r[0] - a4;
			r[0] -= a4;
			return 2;
		}
		
		r[0] = -a4;
		return 1;
	}
	
	if ( numberOf3rdDegreeSolutions == 2 )
	{
		if ( ( r3[0] > 0.L ) && ( r3[1] > 0.L ) )
		{
			if ( fabs ( 2.L*r3[0] + r3[1] + a3 ) <
					fabs ( 2.L*r3[1] + r3[0] + a3 ) )
			{
				r3[1] = sqrt ( r3[1] );
				if ( q >= 0.L ) r3[1] = -r3[1];
				r3[0] = 2.L*sqrt ( r3[0] );
				r[0] = 0.5L*( r3[1] + r3[0] ) - a4;
				r[1] = 0.5L*( r3[1] - r3[0] ) - a4;
				r[2] = -0.5L*r3[1] - a4;
				return 3;
			}
			else
			{
				r3[0] = sqrt ( r3[0] );
				if ( q >= 0.L ) r3[0] = -r3[0];
				r3[1] = 2.L*sqrt ( r3[1] );
				r[0] = 0.5L*( r3[0] + r3[1] ) - a4;
				r[1] = 0.5L*( r3[0] - r3[1] ) - a4;
				r[2] = -0.5L*r3[0] - a4;
				return 3;
			}
		}
		
		if ( ( r3[0] < 0.L ) )
		{
			if ( r3[1] > 0.L )
			{
				r[0] = 0.5L*sqrt ( r3[1] );
				r[1] = -r[0] - a4;
				r[0] -= a4;
				return 2;
			}
			if ( r3[1] == 0.L )
			{
				if ( fabs ( 2.L*r3[0] + a3 ) < fabs ( r3[0] + a3 ) )
				{
					r[0] = -a4;
					return 1;
				}
				else return 0;
			}
			else return 0;
		}

		if ( ( r3[1] < 0.L ) )
		{
			if ( r3[0] == 0.L )
			{
				if ( fabs ( 2.L*r3[1] + a3 ) < fabs ( r3[1] + a3 ) )
				{
					r[0] = -a4;
					return 1;
				}
				else return 0;
			}
			else
			{
				r[0] = 0.5L*sqrt ( r3[0] );
				r[1] = -r[0] - a4;
				r[0] -= a4;
				return 2;
			}
		}
		
		if ( ( r3[0] == 0.L ) && ( r3[1] == 0.L ) )
		{
			r[0] = -a4;
			return 1;
		}
		
		if ( r3[0] == 0.L )
		{
			if ( fabs ( 2.L*r3[1] + a3 ) < fabs ( r3[1] + a3 ) )
			{
				r[0] = sqrt ( r3[1] );
				r[1] = -r[0] - a4;
				r[0] -= a4;
				r[2] = -a4;
				return 3;
			}
			else
			{
				r[0] = 0.5L*sqrt ( r3[1] );
				r[1] = -r[0] - a4;
				r[0] -= a4;
				return 2;
			}
		}
		
		if ( fabs ( 2.L*r3[0] + a3 ) < fabs ( r3[0] + a3 ) )
		{
			r[0] = sqrt ( r3[0] );
			r[1] = -r[0] - a4;
			r[0] -= a4;
			r[2] = -a4;
			return 3;
		}
		else
		{
			r[0] = 0.5L*sqrt ( r3[0] );
			r[1] = -r[0] - a4;
			r[0] -= a4;
			return 2;
		}
	}
/* End exotic cases */

	a2 = a3 + r3[0];
	b2 = b3 + a3*r3[0] + Square ( r3[0] );
	r3[0] = sqrt ( r3[0] );
	if ( q >= 0.L ) r3[0] = -r3[0];

	if ( ( 4.L*b2 - a2*a2 ) > 0.L )
	{
		r3[1] = sqrt ( 2.L*sqrt ( b2 ) - a2 );
		r[0] = 0.5L*( r3[0] + r3[1] ) - a4;
		r[1] = 0.5L*( r3[0] - r3[1] ) - a4;
		return 2;
	}
	else
	{
		/* Another exotic case */
		r[0] = 0.5L*r3[0] - a4;
		r[1] = -0.5L*r3[0] - a4;
		r[2] = 1.5L*r3[0] - a4;
		return 3;
	}
}


/* Solving r**3 + a*r**2 + b*r + c = 0 */
short Solve3rdDegree (
	long double a, long double b, long double c, long double r[3] )
{
	long double p, q, Q, temp;
	
	p = -a*a/3.L + b;
	q = 2.L*a*a*a/27.L - a*b/3.L + c;
	Q = p*p*p/27.L + q*q/4.L;
	
	if ( Q == 0.L )
	{
		if ( ( p == 0.L ) && ( q == 0.L ) )
		{
			r[0] = -a/3.L;
			return 1;
		}
		else
		{
			r[0] = CubicRoot ( -4.L * q );
			r[1] = -0.5L * r[0];
			temp = a / 3.L;
			r[0] -= temp;
			r[1] -= temp;
			return 2;
		}
	}
	if ( Q > 0.L )
	{
		Q = sqrt ( Q );
		r[0] = CubicRoot ( -0.5L * q + Q ) +
			CubicRoot ( -0.5L * q - Q ) - a / 3.L;
		return 1;
	}
	p = sqrt ( -4.L * p / 3.L );
	q = acos ( -4.L * q / ( p*p*p ) ) / 3.L;
	temp = a / 3.L;
	r[0] = p * cos ( q ) - temp;
	Q = pi (  ) / 3.L;
	r[1] = -p * cos ( q - Q ) - temp;
	r[2] = -p * cos ( q + Q ) - temp;
	return 3;
}


/* Solving r**2 + a*r + b = 0 */
short Solve2ndDegree ( long double a, long double b, long double r[2] )
{
	long double d = a*a - 4.L*b;
	if ( d < 0.L ) return 0;
	if ( d == 0.L )
	{
		r[0] = -0.5L * a;
		return 1;
	}
	d = sqrt ( d );
	r[0] = -0.5L * ( a - d );
	r[1] = -0.5L * ( a + d );
	return 2;
}


long double CubicRoot ( long double x )
{
	if ( x < 0.L ) return -pow ( -x, ( 1.L/3.L ) );
	return pow ( x, ( 1.L/3.L ) );
}


long double Square ( long double x )
{
	return x * x;
}


/* Matrix algebra */
#if 0
static	void		Invert3DMatrix
						( long double m[3][3], long double im[3][3] );
void Invert3DMatrix ( long double m[3][3], long double im[3][3] )
{
	long double d = Determinant3D ( m );
	short i, j;
	
	if ( fabs ( d ) <= LDBL_MIN )
	{
		i = 3;
		while ( i-- )
		{
			j = 3;
			while ( j-- ) im[i][j] = 0.L;
		}
		return;
	}
	im[0][0] = ( m[1][1]*m[2][2] - m[1][2]*m[2][1] )/d;
	im[1][0] = ( m[1][2]*m[2][0] - m[1][0]*m[2][2] )/d;
	im[2][0] = ( m[1][0]*m[2][1] - m[1][1]*m[2][0] )/d;
	im[0][1] = ( m[0][2]*m[2][1] - m[0][1]*m[2][2] )/d;
	im[1][1] = ( m[0][0]*m[2][2] - m[0][2]*m[2][0] )/d;
	im[2][1] = ( m[0][1]*m[2][0] - m[0][0]*m[2][1] )/d;
	im[0][2] = ( m[0][1]*m[1][2] - m[0][2]*m[1][1] )/d;
	im[1][2] = ( m[0][2]*m[1][0] - m[0][0]*m[1][2] )/d;
	im[2][2] = ( m[0][0]*m[1][1] - m[0][1]*m[1][0] )/d;
}
#endif

void Multiply3DMatrices
	( long double m1[3][3], long double m2[3][3], long double mp[3][3] )
{
	mp[0][0] = m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0] + m1[0][2]*m2[2][0];
	mp[0][1] = m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1] + m1[0][2]*m2[2][1];
	mp[0][2] = m1[0][0]*m2[0][2] + m1[0][1]*m2[1][2] + m1[0][2]*m2[2][2];
	mp[1][0] = m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0] + m1[1][2]*m2[2][0];
	mp[1][1] = m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1] + m1[1][2]*m2[2][1];
	mp[1][2] = m1[1][0]*m2[0][2] + m1[1][1]*m2[1][2] + m1[1][2]*m2[2][2];
	mp[2][0] = m1[2][0]*m2[0][0] + m1[2][1]*m2[1][0] + m1[2][2]*m2[2][0];
	mp[2][1] = m1[2][0]*m2[0][1] + m1[2][1]*m2[1][1] + m1[2][2]*m2[2][1];
	mp[2][2] = m1[2][0]*m2[0][2] + m1[2][1]*m2[1][2] + m1[2][2]*m2[2][2];
}


void Multiply3DMatrixByVector
	( long double m[3][3], long double v[3], long double vp[3] )
{
	vp[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
	vp[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
	vp[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
}


long double Determinant3D ( long double m[3][3] )
{
	return (
		m[0][0]*m[1][1]*m[2][2] + m[0][1]*m[1][2]*m[2][0] +
		m[0][2]*m[1][0]*m[2][1] - m[0][2]*m[1][1]*m[2][0] -
		m[0][1]*m[1][0]*m[2][2] - m[0][0]*m[1][2]*m[2][1] );
}


/* Normalized vectors iV1 and iV2 transform under some
	rotation into oV1 and oV2. iV1.iV2 must be = oV1.oV2
	and != 1.
	Find the matrix of this rotation */
void FindRotMatrix ( long double iV1[3], long double iV2[3], 
	long double oV1[3], long double oV2[3], long double rotMat[3][3] )
{
	long double cosD, sinD, cosD1;
	long double iV2temp[3], oV2temp[3], tempVect[3];
	long double rotMatrix1[3][3], rotMatrix2[3][3];
	
	/* First we rotate around rotAxis = iV1 x oV1
	 then we rotate around oV1 */
	
	cosD = oV1[0]*iV1[0] + oV1[1]*iV1[1] + oV1[2]*iV1[2];

	if ( fabs ( 1.L - cosD ) < LDBL_MIN )
	{
		rotMatrix1[0][0] = 1.L;
		rotMatrix1[0][1] = 0.L;
		rotMatrix1[0][2] = 0.L;
		rotMatrix1[1][0] = 0.L;
		rotMatrix1[1][1] = 1.L;
		rotMatrix1[1][2] = 0.L;
		rotMatrix1[2][0] = 0.L;
		rotMatrix1[2][1] = 0.L;
		rotMatrix1[2][2] = 1.L;

		iV2temp[0] = iV2[0];
		iV2temp[1] = iV2[1];
		iV2temp[2] = iV2[2];
	}
	else
	{
		cosD1 = 1.L + cosD;
		
		if ( fabs ( cosD1 ) < LDBL_MIN )
		{
			if ( fabs ( iV1[0] ) > LDBL_MIN )
			{
				tempVect[0] = 0.L;
				tempVect[1] = 1.L;
				tempVect[2] = 1.L;
			}
			else
			{
				if ( fabs ( iV1[1] ) > LDBL_MIN )
				{
					tempVect[0] = 1.L;
					tempVect[1] = 0.L;
					tempVect[2] = 1.L;
				}
				else
				{
					tempVect[0] = 1.L;
					tempVect[1] = 1.L;
					tempVect[2] = 0.L;
				}
			}
			cosD = iV1[0]*tempVect[0] + iV1[1]*tempVect[1] + iV1[2]*tempVect[2];
			tempVect[0] -= cosD*iV1[0];
			tempVect[1] -= cosD*iV1[1];
			tempVect[2] -= cosD*iV1[2];
			cosD1 = 2.L/ ( Square ( tempVect[0] ) +
				Square ( tempVect[1] ) + Square ( tempVect[2] ) );
			
			rotMatrix1[0][0] = tempVect[0]*tempVect[0]*cosD1 - 1.L;
			rotMatrix1[0][1] = tempVect[0]*tempVect[1]*cosD1;
			rotMatrix1[0][2] = tempVect[0]*tempVect[2]*cosD1;
	
			rotMatrix1[1][0] = tempVect[1]*tempVect[0]*cosD1;
			rotMatrix1[1][1] = tempVect[1]*tempVect[1]*cosD1 - 1.L;
			rotMatrix1[1][2] = tempVect[1]*tempVect[2]*cosD1;
	
			rotMatrix1[2][0] = tempVect[2]*tempVect[0]*cosD1;
			rotMatrix1[2][1] = tempVect[2]*tempVect[1]*cosD1;
			rotMatrix1[2][2] = tempVect[2]*tempVect[2]*cosD1 - 1.L;
		}
		else
		{
			tempVect[0] = iV1[1]*oV1[2] - iV1[2]*oV1[1];
			tempVect[1] = iV1[2]*oV1[0] - iV1[0]*oV1[2];
			tempVect[2] = iV1[0]*oV1[1] - iV1[1]*oV1[0];
	
			rotMatrix1[0][0] = tempVect[0]*tempVect[0]/cosD1 + cosD;
			rotMatrix1[0][1] = tempVect[0]*tempVect[1]/cosD1 - tempVect[2];
			rotMatrix1[0][2] = tempVect[0]*tempVect[2]/cosD1 + tempVect[1];
	
			rotMatrix1[1][0] = tempVect[1]*tempVect[0]/cosD1 + tempVect[2];
			rotMatrix1[1][1] = tempVect[1]*tempVect[1]/cosD1 + cosD;
			rotMatrix1[1][2] = tempVect[1]*tempVect[2]/cosD1 - tempVect[0];
	
			rotMatrix1[2][0] = tempVect[2]*tempVect[0]/cosD1 - tempVect[1];
			rotMatrix1[2][1] = tempVect[2]*tempVect[1]/cosD1 + tempVect[0];
			rotMatrix1[2][2] = tempVect[2]*tempVect[2]/cosD1 + cosD;
		}
		Multiply3DMatrixByVector ( rotMatrix1, iV2, iV2temp );
	}
	
	cosD = oV2[0]*oV1[0] + oV2[1]*oV1[1] + oV2[2]*oV1[2];
	
	oV2temp[0] = oV2[0] - cosD*oV1[0];
	oV2temp[1] = oV2[1] - cosD*oV1[1];
	oV2temp[2] = oV2[2] - cosD*oV1[2];
	
	cosD = iV2temp[0]*oV1[0] + iV2temp[1]*oV1[1] + iV2temp[2]*oV1[2];

	iV2temp[0] -= cosD*oV1[0];
	iV2temp[1] -= cosD*oV1[1];
	iV2temp[2] -= cosD*oV1[2];
	
	cosD = sqrt (
		( Square ( oV2temp[0] ) + Square ( oV2temp[1] ) + Square ( oV2temp[2] ) ) *
		( Square ( iV2temp[0] ) + Square ( iV2temp[1] ) + Square ( iV2temp[2] ) ) );
	
	if ( cosD < LDBL_MIN ) cosD = 1.L;
	else cosD = ( oV2temp[0]*iV2temp[0] + oV2temp[1]*iV2temp[1] +
		oV2temp[2]*iV2temp[2] )/cosD;
		
	if ( fabs ( 1.L - cosD ) < LDBL_MIN )
	{
		rotMat[0][0] = rotMatrix1[0][0];
		rotMat[0][1] = rotMatrix1[0][1];
		rotMat[0][2] = rotMatrix1[0][2];
		rotMat[1][0] = rotMatrix1[1][0];
		rotMat[1][1] = rotMatrix1[1][1];
		rotMat[1][2] = rotMatrix1[1][2];
		rotMat[2][0] = rotMatrix1[2][0];
		rotMat[2][1] = rotMatrix1[2][1];
		rotMat[2][2] = rotMatrix1[2][2];
	}
	else
	{
		if ( (
			oV1[0]*iV2temp[1]*oV2temp[2] +
			oV1[1]*iV2temp[2]*oV2temp[0] +
			oV1[2]*iV2temp[0]*oV2temp[1] -
			oV1[2]*iV2temp[1]*oV2temp[0] -
			oV1[1]*iV2temp[0]*oV2temp[2] -
			oV1[0]*iV2temp[2]*oV2temp[1]
			) > 0.L ) sinD = sqrt ( 1.L - cosD*cosD );
		else sinD = -sqrt ( 1.L - cosD*cosD );

		cosD1 = 1.L - cosD;

		tempVect[0] = sinD*oV1[0];
		tempVect[1] = sinD*oV1[1];
		tempVect[2] = sinD*oV1[2];

		rotMatrix2[0][0] = cosD1*oV1[0]*oV1[0] + cosD;
		rotMatrix2[0][1] = cosD1*oV1[0]*oV1[1] - tempVect[2];
		rotMatrix2[0][2] = cosD1*oV1[0]*oV1[2] + tempVect[1];

		rotMatrix2[1][0] = cosD1*oV1[1]*oV1[0] + tempVect[2];
		rotMatrix2[1][1] = cosD1*oV1[1]*oV1[1] + cosD;
		rotMatrix2[1][2] = cosD1*oV1[1]*oV1[2] - tempVect[0];

		rotMatrix2[2][0] = cosD1*oV1[2]*oV1[0] - tempVect[1];
		rotMatrix2[2][1] = cosD1*oV1[2]*oV1[1] + tempVect[0];
		rotMatrix2[2][2] = cosD1*oV1[2]*oV1[2] + cosD;
		
		Multiply3DMatrices
			( rotMatrix2, rotMatrix1, rotMat );
	}
}


double
pDist ( inputPoint *iPt0, inputPoint *iPt1, inputPoint *iPt2 )
{
        double ax1, ax2, ax3;
        double ay1, ay2, ay3;
        double az1, az2, az3;
        double delx, dely, delz;
        double sqmag21, u;
        double xp, yp, zp;
        double mag3p;

        ax1 = iPt0->oldLoc[0];
        ay1 = iPt0->oldLoc[1];
        az1 = iPt0->oldLoc[2];
        ax2 = iPt1->oldLoc[0];
        ay2 = iPt1->oldLoc[1];
        az2 = iPt1->oldLoc[2];
        ax3 = iPt2->oldLoc[0];
        ay3 = iPt2->oldLoc[1];
        az3 = iPt2->oldLoc[2];

        delx = ax2 - ax1;
        dely = ay2 - ay1;
        delz = az2 - az1;
        sqmag21 = ( delx*delx + dely*dely + delz*delz );

        if ( sqmag21 < 0.00000001 ) return( 0.0 );

        u = ((ax3-ax1)*(ax2-ax1)+(ay3-ay1)*(ay2-ay1)+(az3-az1)*(az2-az1)) / sqmag21;
        xp = ax1 + u * (ax2 - ax1 );
        yp = ay1 + u * (ay2 - ay1 );
        zp = az1 + u * (az2 - az1 );

        delx = xp - ax3;
        dely = yp - ay3;
        delz = zp - az3;
        mag3p = sqrt(delx*delx + dely*dely + delz*delz);
        return(mag3p);
}


double
aDist ( inputPoint *iPt0, inputPoint *iPt1, inputPoint *iPt2 )
{
        double ax1, ax2, ax3;
        double ay1, ay2, ay3;
        double az1, az2, az3;
        double delx, dely, delz;
        double sqmag21, u;
        double xp, yp, zp;
        double amag3p;

        ax1 = iPt0->xRad;
        ay1 = iPt0->yRad;
        az1 = 0;
        ax2 = iPt1->xRad;
        ay2 = iPt1->yRad;
        az2 = 0;
        ax3 = iPt2->xRad;
        ay3 = iPt2->yRad;
        az3 = 0;

        delx = ax2 - ax1;
        dely = ay2 - ay1;
        delz = az2 - az1;
        sqmag21 = ( delx*delx + dely*dely + delz*delz );

        if ( sqmag21 < 0.00000001 ) return( 0.0 );

        u = ((ax3-ax1)*(ax2-ax1)+(ay3-ay1)*(ay2-ay1)+(az3-az1)*(az2-az1)) / sqmag21;
        xp = ax1 + u * (ax2 - ax1 );
        yp = ay1 + u * (ay2 - ay1 );
        zp = az1 + u * (az2 - az1 );

        delx = xp - ax3;
        dely = yp - ay3;
        delz = zp - az3;
        amag3p = sqrt((delx*delx) + (dely*dely) + (delz*delz));
        return(amag3p);
}

