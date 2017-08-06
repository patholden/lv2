#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <math.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"

/*
static char rcsid[] = "$Id: 3DTransform.c,v 1.8 2007/04/02 08:37:11 pickle Exp pickle $";
*/

/****************************************************************/
/*								*/
/*	By Ilya R. Lapshin at Assembly Guidance Systems, Inc.	*/
/*								*/
/****************************************************************/


enum {
	false,
	true
};

typedef unsigned char Boolean;

#include <float.h>
#include <math.h>
#include <stdio.h>
/*
 * #define pi()	3.14159265358979323846
 */


int min3Distance = 1.0;

#include "L3DTransform.h"
#include "3DTransform.h"

#define kMaxIter			50

#define kReallySmall		1.E-4 /* Good value: 1.E-7L */

#define kIterations 7
#define	kHalfXMirror	0.031
#define	kHalfYMirror	0.031

/* might be changed to improve the performance */



void TransformPoint
	( transform *tr, double oldLoc[3], double newLoc[3] )
{
	Multiply3DMatrixByVector
		( tr->rotMatrix, oldLoc, newLoc );
	newLoc[0] += tr->transVector[0];
	newLoc[1] += tr->transVector[1];
	newLoc[2] += tr->transVector[2];
}

/* Works only for matrices with det > 0 */
/* Makes a rotational matrix from a near-rotational one*/
/* Modified to consider the rows, not columns, to be the vectors */
/* being transformed */
void DblMakeARotMatrix ( double m[3][3] )
{
	double rotAxis[3], rotAxisAbs, rotAxisSq[3], averAxis[3];
	double rotMatrix1[3][3], rotMatrix2[3][3], tempVect[3];
	double temp, sinD;
	
	rotAxis[0] = ( m[0][0] + m[1][0] + m[2][0] ) / 3.0;
	rotAxis[1] = ( m[0][1] + m[1][1] + m[2][1] ) / 3.0;
	rotAxis[2] = ( m[0][2] + m[1][2] + m[2][2] ) / 3.0;

	rotAxisSq[0] = Square ( rotAxis[0] );
	rotAxisSq[1] = Square ( rotAxis[1] );
	rotAxisSq[2] = Square ( rotAxis[2] );
	
	rotAxisAbs = sqrt ( rotAxisSq[0] + rotAxisSq[1] + rotAxisSq[2] );
	
	rotAxis[0] /= rotAxisAbs;
	rotAxis[1] /= rotAxisAbs;
	rotAxis[2] /= rotAxisAbs;

	/* Now rotAxis is the average of the three suggested axes */

	sinD = -sqrt ( 0.750 ); /* D = -120 degrees */
	
	tempVect[0] = sinD * rotAxis[0];
	tempVect[1] = sinD * rotAxis[1];
	tempVect[2] = sinD * rotAxis[2];
	
	rotMatrix1[0][0] = 1.50 * rotAxis[0] * rotAxis[0] - 0.50;
	rotMatrix1[0][1] = 1.50 * rotAxis[0] * rotAxis[1] - tempVect[2];
	rotMatrix1[0][2] = 1.50 * rotAxis[0] * rotAxis[2] + tempVect[1];
	
	rotMatrix1[1][0] = 1.50 * rotAxis[1] * rotAxis[0] + tempVect[2];
	rotMatrix1[1][1] = 1.50 * rotAxis[1] * rotAxis[1] - 0.50;
	rotMatrix1[1][2] = 1.50 * rotAxis[1] * rotAxis[2] - tempVect[0];
	
	rotMatrix1[2][0] = 1.50 * rotAxis[2] * rotAxis[0] - tempVect[1];
	rotMatrix1[2][1] = 1.50 * rotAxis[2] * rotAxis[1] + tempVect[0];
	rotMatrix1[2][2] = 1.50 * rotAxis[2] * rotAxis[2] - 0.50;
	
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
	
	sinD = sqrt ( 3.0 );
	temp = sqrt ( 2.0 / ( Square ( tempVect[0] ) +
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
/*	From three inputPoints From3PtsToTrans derives several		*/
/*	transforms. We consider only those for which the newZ is	*/
/*	less or equal zero (so that it is looked from above). If the	*/
/*	other four trans are needed, just mutiply rotMatrix and		*/
/*	transVector by -1. The short output says how many transforms	*/
/*	is actually found. 						*/
/********************************************************************/

double
p3Dist ( doubleInputPoint *iPt0
       , doubleInputPoint *iPt1
       , doubleInputPoint *iPt2
       )
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
        mag3p = sqrt( delx*delx + dely*dely + delz*delz );
        return( mag3p );
}

/********************************************************************/
/*	This function solves the following system		*/
/*	r[0] >= 0;						*/
/*	r[0]**2 - 2*a01*r[0]*r[1] + r[1]**2 = sqd01		*/
/*	r[0]**2 - 2*a02*r[0]*r[2] + r[2]**2 = sqd02		*/
/*	r[1]**2 - 2*a12*r[1]*r[2] + r[2]**2 = sqd12		*/
/*	and returns the number of real solutions.		*/
/*	Actually there are four of them.			*/
/*	But some pathological cases might produce up to		*/
/*	16(!!!), some of which are dupli-, or quadruplated.	*/
/*	Let's keep them all for simplicity.			*/
/********************************************************************/
short SolveTheSystem ( double a01, double sqd01,
	double a02, double sqd02,
	double a12, double sqd12,
	double rs[16][3] )
{
	/* The resulting equation is */
	/* t*r[0]**4 + a*r[0]**3 + b*r[0]**2 + c*r[0] + d = 0 */
	
	double a, b, c, d, t, r[4];
	double a2, b2, c2, d2, t2;
	double a01p2, a02p2, a12p2, sqd01p2, sqd02p2, sqd12p2;
	double a01p3, a02p3, a12p3, sqd01p3, sqd02p3, sqd12p3;
	double a01p4, a02p4, a12p4, sqd01p4, sqd02p4, sqd12p4;
	
	double det01, det02, r0Temp, r1Temp1, r1Temp2, r2Temp;

	double rootCoefficient, rootCoefficientTemp;
	
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
	
	if ( ( ( t2/( a2 + t2 ) ) <= DBL_MIN ) ||
		( ( t2/( b2 + t2 ) ) <= DBL_MIN ) ||
		( ( t2/( c2 + t2 ) ) <= DBL_MIN ) ||
		( ( t2/( d2 + t2 ) ) <= DBL_MIN ) )
	{
		if ( ( ( a2/( b2 + a2 ) ) <= DBL_MIN ) ||
			( ( a2/( c2 + a2 ) ) <= DBL_MIN ) ||
			( ( a2/( d2 + a2 ) ) <= DBL_MIN ) )
		{
			if ( ( ( b2/( c2 + b2 ) ) <= DBL_MIN ) ||
				( ( b2/( d2 + b2 ) ) <= DBL_MIN ) )
			{
                                if ( ( c2/( c2 + d2 ) ) <= DBL_MIN ) {
                                     syslog(LOG_ERR, "925 SolveTheSystem" );
                                     return 0;
                                }
				else
				{
					numberOfRoots = 1;
					rootCoefficient = 1.0;
					r[0] = -d/c;
				}
			}
			else
			{
				t = b;
				a = c/t;
				b = d/t;
				rootCoefficient = 1.0;
				numberOfRoots = Solve2ndDegree ( a, b, r );
			}
		}
		else
		{
			t = a;
			a = b/t;
			b = c/t;
			c = d/t;
			rootCoefficient = 1.0;
			numberOfRoots = Solve3rdDegree ( a, b, c, r );
		}
	}
	else
	{
		rootCoefficient = sqrt ( sqrt ( fabs ( d / t ) ) );
		if ( rootCoefficient <= DBL_MIN )
		{
			a = a/t;
			b = b/t;
			c = c/t;
			d = d/t;
			rootCoefficient = 1.0;
		}
		else
		{
			rootCoefficientTemp = sqrt ( fabs ( d * t ) );
			if ( t > 0.0 )
			{
				a = a / sqrt( fabs( t ) * rootCoefficientTemp );
				b = b / rootCoefficientTemp;
				c = c / sqrt( fabs( d ) * rootCoefficientTemp );
				if ( d > 0.0 ) d = 1.0;
				else d = -1.0;
			}
			else
			{
				a = -a / sqrt(fabs( t ) * rootCoefficientTemp);
				b = -b / rootCoefficientTemp;
				c = -c / sqrt(fabs( d ) * rootCoefficientTemp);
				if ( d > 0.0 ) d = -1.0;
				else d = 1.0;
			}
		}
		numberOfRoots = Solve4thDegree ( a, b, c, d, r );
	}
	
	i = numberOfRoots;
	numberOfSolutions = 0;
	while ( i-- )
	{
		r[i] *= rootCoefficient;
		if ( ( ( det01 = sqd01 - ( 1.0 - a01p2 )*r[i] ) >= 0.0 ) &&
			( ( det02 = sqd02 - ( 1.0 - a02p2 )*r[i] ) >= 0.0 ) )
		{
			/* Catching det01==0 and det02==0 is not very
			   useful, but it's OK */
			det01 = sqrt ( det01 );
			det02 = sqrt ( det02 );
			r0Temp = sqrt ( r[i] );
			r1Temp1 = a01*r0Temp + det01;
			r1Temp2 = a01*r0Temp - det01;
			r2Temp = a02*r0Temp + det02;
			if ( fabs ( ( r1Temp1*r1Temp1 - 2.0*a12*r1Temp1*r2Temp +
				r2Temp*r2Temp )/sqd12 - 1.0 ) < kReallySmall )
			{
				rs[numberOfSolutions][0] = r0Temp;
				rs[numberOfSolutions][1] = r1Temp1;
				rs[numberOfSolutions][2] = r2Temp;
				numberOfSolutions++;
			}
			if (( fabs( ( r1Temp2*r1Temp2 - 2.0*a12*r1Temp2*r2Temp +
				r2Temp*r2Temp )/sqd12 - 1.0 ) < kReallySmall )
					 &&
				( det01 != 0.0 ) )
				/* The last != almost never helps */
			{
				rs[numberOfSolutions][0] = r0Temp;
				rs[numberOfSolutions][1] = r1Temp2;
				rs[numberOfSolutions][2] = r2Temp;
				numberOfSolutions++;
			}
			if ( det02 != 0.0 )
			{
				r2Temp = a02*r0Temp - det02;
				if (
					 fabs ( ( r1Temp1*r1Temp1
				 	       - 2.0*a12*r1Temp1*r2Temp +
					       r2Temp*r2Temp )/sqd12 - 1.0 )
                                         < kReallySmall )
				{
					rs[numberOfSolutions][0] = r0Temp;
					rs[numberOfSolutions][1] = r1Temp1;
					rs[numberOfSolutions][2] = r2Temp;
					numberOfSolutions++;
				}
				if ( ( fabs ( ( r1Temp2*r1Temp2
                                             - 2.0*a12*r1Temp2*r2Temp +
					     r2Temp*r2Temp )/sqd12 - 1.0 )
                                          < kReallySmall ) &&
					( det01 != 0.0 ) )
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
	double a, double b, double c, double d,
	double r[4] )
{
	double r3[3], p, q, rr, a3, b3, c3, a4, a2, b2;
	short numberOf3rdDegreeSolutions;
	
	/* Just some intermediate numbers*/
	p = a * a;
	rr = -0.01171875 * p * p + 0.0625 * p * b - 0.25 * a * c + d;
	q = 0.125 * p * a - 0.50 * a * b + c;
	p = -0.375 * p + b;
	a4 = a * 0.25;
	
	/* Coefficients for 3rd degree equation to be solved */
	a3 = 2.0 * p;
	b3 = p * p - 4.0 * rr;
	c3 = -q * q;
		
	numberOf3rdDegreeSolutions = Solve3rdDegree ( a3, b3, c3, r3 );
	if ( numberOf3rdDegreeSolutions == 3)
	{
		if ( ( r3[0] > 0.0 ) && ( r3[1] > 0.0 ) && ( r3[2] > 0.0 ) )
		{
			r3[0] = sqrt ( r3[0] );
			if ( q >= 0.0 ) r3[0] = -r3[0];
			r3[1] = sqrt ( r3[1] );
			r3[2] = sqrt ( r3[2] );
			r[0] = 0.50*( r3[0] + r3[1] + r3[2] ) - a4;
			r[1] = 0.50*( r3[0] - r3[1] - r3[2] ) - a4;
			r[2] = 0.50*( r3[1] - r3[0] - r3[2] ) - a4;
			r[3] = 0.50*( r3[2] - r3[0] - r3[1] ) - a4;
			return 4;
		}

                if ( ( r3[0] < 0.0 ) || ( r3[1] < 0.0 ) || ( r3[2] < 0.0 ) )
                        {
                               syslog(LOG_ERR, "Solve4thDegree" );
                               return 0;
                        }


/* The probability of an exotic case in the real life is nil */
/* The probability that something in the following code */
/* is not very smart is not nil. Start exotic cases */
		
		if (
		( ( r3[0] == 0.0 ) && ( r3[1] > 0.0 ) && ( r3[2] > 0.0 ) ) ||
		( ( r3[0] > 0.0 ) && ( r3[1] == 0.0 ) && ( r3[2] > 0.0 ) ) ||
		( ( r3[0] > 0.0 ) && ( r3[1] > 0.0 ) && ( r3[2] == 0.0 ) ) )
		{
			if ( r3[0] == 0.0 )
			{
				r3[0] = sqrt ( r3[2] );
				r3[1] = sqrt ( r3[1] );
			}
			else 
			{
				r3[0] = sqrt ( r3[0] );
				if ( r3[1] == 0.0 ) r3[1] = sqrt ( r3[2] );
				else r3[1] = sqrt ( r3[1] );
			}

			r[0] = 0.50*( r3[0] + r3[1] ) - a4;
			r[1] = 0.50*( -r3[0] - r3[1] ) - a4;
			r[2] = r3[0] - r3[1];
			if ( r[2] == 0.0 )
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
		( ( r3[0] == 0.0 ) && ( r3[1] == 0.0 ) && ( r3[2] > 0.0 ) ) ||
		( ( r3[0] == 0.0 ) && ( r3[1] > 0.0 ) && ( r3[2] == 0.0 ) ) ||
		( ( r3[0] > 0.0 ) && ( r3[1] == 0.0 ) && ( r3[2] == 0.0 ) ) )
		{
			if ( r3[2] > 0.0 ) r[0] = 0.50 * sqrt ( r3[2] );
			else
			{
				if ( r3[1] > 0.0 ) r[0] = 0.50*sqrt ( r3[1] );
				else r[0] = 0.50*sqrt ( r3[0] );
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
		if ( ( r3[0] > 0.0 ) && ( r3[1] > 0.0 ) )
		{
			if ( fabs ( 2.0*r3[0] + r3[1] + a3 ) <
					fabs ( 2.0*r3[1] + r3[0] + a3 ) )
			{
				r3[1] = sqrt ( r3[1] );
				if ( q >= 0.0 ) r3[1] = -r3[1];
				r3[0] = 2.0*sqrt ( r3[0] );
				r[0] = 0.50*( r3[1] + r3[0] ) - a4;
				r[1] = 0.50*( r3[1] - r3[0] ) - a4;
				r[2] = -0.50*r3[1] - a4;
				return 3;
			}
			else
			{
				r3[0] = sqrt ( r3[0] );
				if ( q >= 0.0 ) r3[0] = -r3[0];
				r3[1] = 2.0*sqrt ( r3[1] );
				r[0] = 0.50*( r3[0] + r3[1] ) - a4;
				r[1] = 0.50*( r3[0] - r3[1] ) - a4;
				r[2] = -0.50*r3[0] - a4;
				return 3;
			}
		}
		
		if ( ( r3[0] < 0.0 ) )
		{
			if ( r3[1] > 0.0 )
			{
				r[0] = 0.50*sqrt ( r3[1] );
				r[1] = -r[0] - a4;
				r[0] -= a4;
				return 2;
			}
			if ( r3[1] == 0.0 )
			{
				if ( fabs ( 2.0*r3[0] + a3 )
                                   < fabs ( r3[0] + a3 ) )
				{
					r[0] = -a4;
					return 1;
				}
                                else {
                                    syslog(LOG_ERR, "Solve4thDegree" );
                                    return 0;
                                }
			}
                        else {
                             syslog(LOG_ERR, "Solve4thDegree");
                             return 0;
                        }
		}

		if ( ( r3[1] < 0.0 ) )
		{
			if ( r3[0] == 0.0 )
			{
				if ( fabs ( 2.0*r3[1] + a3 )
                                   < fabs ( r3[1] + a3 ) )
				{
					r[0] = -a4;
					return 1;
				}
                                else {
                                   syslog(LOG_ERR, "Solve4thDegree");
                                   return 0;
                                }
			}
			else
			{
				r[0] = 0.50*sqrt ( r3[0] );
				r[1] = -r[0] - a4;
				r[0] -= a4;
				return 2;
			}
		}
		
		if ( ( r3[0] == 0.0 ) && ( r3[1] == 0.0 ) )
		{
			r[0] = -a4;
			return 1;
		}
		
		if ( r3[0] == 0.0 )
		{
			if ( fabs ( 2.0*r3[1] + a3 ) < fabs ( r3[1] + a3 ) )
			{
				r[0] = sqrt ( r3[1] );
				r[1] = -r[0] - a4;
				r[0] -= a4;
				r[2] = -a4;
				return 3;
			}
			else
			{
				r[0] = 0.50*sqrt ( r3[1] );
				r[1] = -r[0] - a4;
				r[0] -= a4;
				return 2;
			}
		}
		
		if ( fabs ( 2.0*r3[0] + a3 ) < fabs ( r3[0] + a3 ) )
		{
			r[0] = sqrt ( r3[0] );
			r[1] = -r[0] - a4;
			r[0] -= a4;
			r[2] = -a4;
			return 3;
		}
		else
		{
			r[0] = 0.50*sqrt ( r3[0] );
			r[1] = -r[0] - a4;
			r[0] -= a4;
			return 2;
		}
	}
/* End exotic cases */

	a2 = a3 + r3[0];
	b2 = b3 + a3*r3[0] + Square ( r3[0] );
	r3[0] = sqrt ( r3[0] );
	if ( q >= 0.0 ) r3[0] = -r3[0];

	if ( ( 4.L*b2 - a2*a2 ) > 0.0 )
	{
		r3[1] = sqrt ( 2.0*sqrt ( b2 ) - a2 );
		r[0] = 0.50*( r3[0] + r3[1] ) - a4;
		r[1] = 0.50*( r3[0] - r3[1] ) - a4;
		return 2;
	}
	else
	{
		/* Another exotic case */
		r[0] = 0.50*r3[0] - a4;
		r[1] = -0.50*r3[0] - a4;
		r[2] = 1.50*r3[0] - a4;
		return 3;
	}
}


/* Solving r**3 + a*r**2 + b*r + c = 0 */
short Solve3rdDegree (
	double a, double b, double c, double r[3] )
{
	double p, q, Q, temp;
	
	p = -a*a/3.0 + b;
	q = 2.0*a*a*a/27.0 - a*b/3.0 + c;
	Q = p*p*p/27.0 + q*q/4.0;
	
	if ( Q == 0.0 )
	{
		if ( ( p == 0.0 ) && ( q == 0.0 ) )
		{
			r[0] = -a/3.0;
			return 1;
		}
		else
		{
			r[0] = CubicRoot ( -4.0 * q );
			r[1] = -0.50 * r[0];
			temp = a / 3.0;
			r[0] -= temp;
			r[1] -= temp;
			return 2;
		}
	}
	if ( Q > 0.0 )
	{
		Q = sqrt ( Q );
		r[0] = CubicRoot ( -0.50 * q + Q ) +
			CubicRoot ( -0.50 * q - Q ) - a / 3.0;
		return 1;
	}
	p = sqrt ( -4.L * p / 3.0 );
	q = acos ( -4.L * q / ( p*p*p ) ) / 3.0;
	temp = a / 3.0;
	r[0] = p * cos ( q ) - temp;
	Q = M_PI / 3.0;
	r[1] = -p * cos ( q - Q ) - temp;
	r[2] = -p * cos ( q + Q ) - temp;
	return 3;
}


/* Solving r**2 + a*r + b = 0 */
short Solve2ndDegree ( double a, double b, double r[2] )
{
	double d = a*a - 4.L*b;
	if ( d < 0.0 ) return 0;
	if ( d == 0.0 )
	{
		r[0] = -0.50 * a;
		return 1;
	}
	d = sqrt ( d );
	r[0] = -0.50 * ( a - d );
	r[1] = -0.50 * ( a + d );
	return 2;
}


double CubicRoot ( double x )
{
	if ( x < 0.0 ) return -pow ( -x, ( 1.0/3.0 ) );
	return pow ( x, ( 1.0/3.0 ) );
}


double Square ( double x )
{
	return x * x;
}


/* Matrix algebra */

void Invert3DMatrix ( double m[3][3], double im[3][3] )
{
	double d = Determinant3D ( m );
	short i, j;
	
	if ( fabs ( d ) <= DBL_MIN )
	{
		i = 3;
		while ( i-- )
		{
			j = 3;
			while ( j-- ) im[i][j] = 0.0;
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


void Multiply3DMatrices
	( double m1[3][3], double m2[3][3], double mp[3][3] )
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
	( double m[3][3], double v[3], double vp[3] )
{
	vp[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
	vp[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
	vp[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
}


double Determinant3D ( double m[3][3] )
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
void FindRotMatrix ( double iV1[3], double iV2[3], 
	double oV1[3], double oV2[3], double rotMat[3][3] )
{
	double cosD, sinD, cosD1;
	double iV2temp[3], oV2temp[3], tempVect[3];
	double rotMatrix1[3][3], rotMatrix2[3][3];
	
	/* First we rotate around rotAxis = iV1 x oV1
	 then we rotate around oV1 */
	
	cosD = oV1[0]*iV1[0] + oV1[1]*iV1[1] + oV1[2]*iV1[2];

	if ( fabs ( 1.0 - cosD ) < DBL_MIN )
	{
		rotMatrix1[0][0] = 1.0;
		rotMatrix1[0][1] = 0.0;
		rotMatrix1[0][2] = 0.0;
		rotMatrix1[1][0] = 0.0;
		rotMatrix1[1][1] = 1.0;
		rotMatrix1[1][2] = 0.0;
		rotMatrix1[2][0] = 0.0;
		rotMatrix1[2][1] = 0.0;
		rotMatrix1[2][2] = 1.0;

		iV2temp[0] = iV2[0];
		iV2temp[1] = iV2[1];
		iV2temp[2] = iV2[2];
	}
	else
	{
		cosD1 = 1.0 + cosD;
		
		if ( fabs ( cosD1 ) < DBL_MIN )
		{
			if ( fabs ( iV1[0] ) > DBL_MIN )
			{
				tempVect[0] = 0.0;
				tempVect[1] = 1.0;
				tempVect[2] = 1.0;
			}
			else
			{
				if ( fabs ( iV1[1] ) > DBL_MIN )
				{
					tempVect[0] = 1.0;
					tempVect[1] = 0.0;
					tempVect[2] = 1.0;
				}
				else
				{
					tempVect[0] = 1.0;
					tempVect[1] = 1.0;
					tempVect[2] = 0.0;
				}
			}
			cosD = iV1[0]*tempVect[0] + iV1[1]*tempVect[1] +
                               iV1[2]*tempVect[2];
			tempVect[0] -= cosD*iV1[0];
			tempVect[1] -= cosD*iV1[1];
			tempVect[2] -= cosD*iV1[2];
			cosD1 = 2.0/ ( Square ( tempVect[0] ) +
			   Square ( tempVect[1] ) + Square ( tempVect[2] ) );
			
			rotMatrix1[0][0] = tempVect[0]*tempVect[0]*cosD1 - 1.0;
			rotMatrix1[0][1] = tempVect[0]*tempVect[1]*cosD1;
			rotMatrix1[0][2] = tempVect[0]*tempVect[2]*cosD1;
	
			rotMatrix1[1][0] = tempVect[1]*tempVect[0]*cosD1;
			rotMatrix1[1][1] = tempVect[1]*tempVect[1]*cosD1 - 1.0;
			rotMatrix1[1][2] = tempVect[1]*tempVect[2]*cosD1;
	
			rotMatrix1[2][0] = tempVect[2]*tempVect[0]*cosD1;
			rotMatrix1[2][1] = tempVect[2]*tempVect[1]*cosD1;
			rotMatrix1[2][2] = tempVect[2]*tempVect[2]*cosD1 - 1.0;
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
	(Square( oV2temp[0] ) + Square( oV2temp[1] ) + Square( oV2temp[2] ) ) *
	(Square( iV2temp[0] ) + Square( iV2temp[1] ) + Square( iV2temp[2] ) ) );
	
	if ( cosD < DBL_MIN ) cosD = 1.0;
	else cosD = ( oV2temp[0]*iV2temp[0] + oV2temp[1]*iV2temp[1] +
		oV2temp[2]*iV2temp[2] )/cosD;
		
	if ( fabs ( 1.0 - cosD ) < DBL_MIN )
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
			) > 0.0 ) sinD = sqrt ( 1.0 - cosD*cosD );
		else sinD = -sqrt ( 1.0 - cosD*cosD );

		cosD1 = 1.0 - cosD;

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

void TransformIntoArray
	( transform *theTransform, double *theArray )
{
	theArray[0] = theTransform->rotMatrix[0][0];
 	theArray[1] = theTransform->rotMatrix[0][1];
 	theArray[2] = theTransform->rotMatrix[0][2];
 	theArray[3] = theTransform->rotMatrix[1][0];
 	theArray[4] = theTransform->rotMatrix[1][1];
 	theArray[5] = theTransform->rotMatrix[1][2];
 	theArray[6] = theTransform->rotMatrix[2][0];
 	theArray[7] = theTransform->rotMatrix[2][1];
 	theArray[8] = theTransform->rotMatrix[2][2];
 	theArray[9] = theTransform->transVector[0];
 	theArray[10] = theTransform->transVector[1];
 	theArray[11] = theTransform->transVector[2];
}

void TransformIntoLongArray
	( transform *theTransform, long double theArray[12] )
{
	theArray[0] = (long double) theTransform->rotMatrix[0][0];
 	theArray[1] = (long double) theTransform->rotMatrix[0][1];
 	theArray[2] = (long double) theTransform->rotMatrix[0][2];
 	theArray[3] = (long double) theTransform->rotMatrix[1][0];
 	theArray[4] = (long double) theTransform->rotMatrix[1][1];
 	theArray[5] = (long double) theTransform->rotMatrix[1][2];
 	theArray[6] = (long double) theTransform->rotMatrix[2][0];
 	theArray[7] = (long double) theTransform->rotMatrix[2][1];
 	theArray[8] = (long double) theTransform->rotMatrix[2][2];
 	theArray[9]  = (long double) theTransform->transVector[0];
 	theArray[10] = (long double) theTransform->transVector[1];
 	theArray[11] = (long double) theTransform->transVector[2];
	return;
}

void ArrayIntoTransform(double *theArray, transform *theTransform )
{
	theTransform->rotMatrix[0][0] = theArray[0];
 	theTransform->rotMatrix[0][1] = theArray[1];
 	theTransform->rotMatrix[0][2] = theArray[2];
 	theTransform->rotMatrix[1][0] = theArray[3];
 	theTransform->rotMatrix[1][1] = theArray[4];
 	theTransform->rotMatrix[1][2] = theArray[5];
 	theTransform->rotMatrix[2][0] = theArray[6];
 	theTransform->rotMatrix[2][1] = theArray[7];
 	theTransform->rotMatrix[2][2] = theArray[8];
 	theTransform->transVector[0] = theArray[9];
 	theTransform->transVector[1] = theArray[10];
 	theTransform->transVector[2] = theArray[11];
	return;
}

void IdentityArray (double *theArray)
{
	theArray [ 0 ] = 1.0;
	theArray [ 1 ] = 0.0;
	theArray [ 2 ] = 0.0;
	theArray [ 3 ] = 0.0;
	theArray [ 4 ] = 1.0;
	theArray [ 5 ] = 0.0;
	theArray [ 6 ] = 0.0;
	theArray [ 7 ] = 0.0;
	theArray [ 8 ] = 1.0;
	theArray [ 9 ] = 0.0;
	theArray [ 10 ] = 0.0;
	theArray [ 11 ] = 0.0;
}

void InvertTransform ( transform *theTransform, transform *invTransform )
{
	Invert3DMatrix ( theTransform->rotMatrix, invTransform->rotMatrix );
	Multiply3DMatrixByVector ( invTransform->rotMatrix,
		theTransform->transVector, invTransform->transVector );
	invTransform->transVector[0] = -invTransform->transVector[0];
	invTransform->transVector[1] = -invTransform->transVector[1];
	invTransform->transVector[2] = -invTransform->transVector[2];
}
