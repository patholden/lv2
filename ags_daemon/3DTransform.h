#include <stdint.h>
/*   $Id: 3DTransform.h,v 1.4 1999/07/29 20:48:38 ags-sw Exp $  */

#ifndef D3DTRANSFORM_H
#define D3DTRANSFORM_H

#include "L3DTransform.h"

/********************************************************************/
/*								*/
/*			Type definitions			*/
/*								*/
/********************************************************************/


/********************************************************************/
/*	This is what is given about a point. The direction at which it	*/
/*	is seen from the new origin in radians (xRad, yRad) and its	*/
/*	coordinates in some system (x, y, z)				*/
/********************************************************************/

typedef struct { double oldLoc[3], xRad, yRad; }
	inputPoint;



/********************************************************************/
/*	The transformation from old system to new is described as	*/
/*	(newX,newY,newZ) = transVector + rotMatrix * (oldX,oldY,oldZ)	*/
/*	rotMatrix (which must be unitary (it transposed must be its	*/
/*	inverse)) reflects how the new system of coordinates is rotated	*/
/*	with respect to the old, while transVector desribes where the	*/
/*	origin is moved.						*/
/********************************************************************/

typedef struct { double rotMatrix[3][3], transVector[3]; }
	transform;

typedef struct { long double rotMatrix[3][3], transVector[3]; }
	int32_ttransform;



/********************************************************/
/*							*/
/*					Headers		*/
/*							*/
/********************************************************/

extern	void DblMakeARotMatrix ( double m[3][3] );

extern	void FindRotMatrix ( double iV1[3], double iV2[3], 
	double oV1[3], double oV2[3], double rotMat[3][3] );

extern	double CubicRoot ( double x );

extern	void		TransformPoint
					( transform *tr, double oldLoc[3],
					double newLoc[3] );
/*
extern	unsigned char		FindBestTransform
					( inputPoint iPt[4], transform *tr,
					double deltaXHeight,
					double * bestCosine );
*/
	
extern	double	Square ( double x );

extern	void		Invert3DMatrix
					( double m[3][3], double im[3][3] );

extern	void		Multiply3DMatrices
					( double m1[3][3], double m2[3][3],
					double mp[3][3] );
	
extern	void		Multiply3DMatrixByVector
						( double m[3][3],
						double v[3], double vp[3] );
	
extern	double	Determinant3D ( double m[3][3] );

extern	short		SolveTheSystem ( double a01, double sqd01,
						double a02, double sqd02,
						double a12, double sqd12,
						double rs[16][3] );
	
extern	short		Solve4thDegree (
						double a, double b,
						double c, double d,
						double r[4] );
	
extern	short		Solve3rdDegree (
						double a, double b,
						double c, double r[3] );
	
extern	short		Solve2ndDegree
						( double a, double b,
						double r[2] );

extern	void		TransformIntoArray ( transform *theTransform,
						double theArray[12] );
					
extern	void		LongArrayIntoTransform ( long double theArray[12],
						transform *theTransform );
					
extern	void		TransformIntoLongArray ( transform *theTransform,
						long double *theArray );
					
extern	void		IdentityArray ( double theArray[12] );

extern	void		InvertTransform ( transform *theTransform,
						transform *invTransform );
void ArrayIntoTransform(double *theArray, transform *theTransform);

extern double p3Dist( doubleInputPoint *iPt0 
                    , doubleInputPoint *iPt1 
                    , doubleInputPoint *iPt2
                    );

#endif
