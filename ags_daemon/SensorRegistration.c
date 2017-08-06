/*
static char rcsid[] = "$Id: SensorRegistration.c,v 1.15 2007/04/02 08:42:12 pickle Exp pickle $";
*/
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
#include <syslog.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "comm_loop.h"
#include "parse_data.h"
#include "L3DTransform.h"
#include "3DTransform.h"
#include "SensorRegistration.h"
#include "permutes.h"
#include "area_order.h"
#include "FOM.h"
#include "LaserInterface.h"
#include "QuickCheckManager.h"
#include "chisqr.h"
#include "amoeba.h"
#include "angles.h"

#ifndef __unix__
enum {
        false,
        true
};
#endif

int       gNPoints;
int       gSaved;
double    gWorstTolReg;
double    gWorstTolAll;
double    gBestTolAll;
double gMaxDiffMag = 0.0;
doubleTransform gBestOfAllTransform;
double gX[kNumberOfFlexPoints];
double gY[kNumberOfFlexPoints];
double gZ[kNumberOfFlexPoints];
double gXfoundAngle[kNumberOfFlexPoints];
double gYfoundAngle[kNumberOfFlexPoints];
double chiX[kNumberOfFlexPoints];
double chiY[kNumberOfFlexPoints];
double chiZ[kNumberOfFlexPoints];
double chiXfoundAngle[kNumberOfFlexPoints];
double chiYfoundAngle[kNumberOfFlexPoints];
static unsigned char gDontFindTranform = false;
double gDiffX[MAXTRANSNUM];
double gDiffY[MAXTRANSNUM];
double gDiffMag[MAXTRANSNUM];
int savePoint[MAXTRANSNUM];
doubleInputPoint gSavePt[MAXTRANSNUM];

double gTolInch = 0.05;

int32_t gNumberOfPoints;

double gChisqr;

short GnOfTrans;

double gFOM;
double gFOMavg;

static double magCross(doubleInputPoint *iPt0, doubleInputPoint *iPt1,
		       doubleInputPoint *iPt2, doubleInputPoint *Pnorm);
static double planeDist(doubleInputPoint *point, doubleInputPoint *iPt0,
			doubleInputPoint *pPerpend);
static int dblsort(const void *elem1, const void *elem2);

uint32_t get_number_of_points(void)
{
    return(gNumberOfPoints);
}
double get_chisquare_val(void)
{
    return(gChisqr);
}
void get_target_info(struct k_targetinfo *pTgtInfo, uint32_t tgt_num)
{
    pTgtInfo->tgt_number = tgt_num;
    pTgtInfo->xdev       = gDiffX[tgt_num];
    pTgtInfo->ydev       = gDiffY[tgt_num];
    return;
}
void SaveFullRegCoordinates(uint16_t numberOfPoints, double *theOriginalCoordinates)
{
	uint16_t i = numberOfPoints;
	double *x = gX, *y = gY, *z = gZ;
	
	while (i--)
	{
		*x++ = *theOriginalCoordinates++;
		*y++ = *theOriginalCoordinates++;
		*z++ = *theOriginalCoordinates++;
	}
	
	return;
}

void DontFindTransform ( unsigned char doOrDont )
{
	gDontFindTranform = doOrDont;
}

unsigned char FindTransformMatrix (struct lg_master *pLgMaster,uint16_t numberOfPoints,
				   double deltaMirror, double tolerance, double *foundAngles,
				   double *theTransform)
{
	doubleInputPoint iPt[4], tempPt[100];
        doubleInputPoint normPt;
	transform tr;
	transform Xtr;
        doubleTransform tempTr;
        transform savetr[MAXTRANSNUM];
        transform testtr;
        int iTr;
        int usedPoints;
        double testFOM;
        double FOMavg;
        double minCos[MAXTRANSNUM];
        double tCos;
        int saveGood[MAXTRANSNUM];
        double saveBest[MAXTRANSNUM];
        double sortBest[MAXTRANSNUM];
        int goodcount;
	uint16_t i, j, k, n, p, nOfTrans;
        int a,b,c,d;
        double oldLoc[3];
        double newLoc[3];
	double newX;
	double newY;
	double newZ;
	double diffMag;
	double foundX;
	double foundY;
	double FOMmin;
	double FOMsum;
        double **Pmatrix;
        double **Qmatrix;
        double *Ys;
        double *Y2s;
        int ndim;
        double ftol;
        int nfunk;
        double roll;
        double pitch;
        double yaw;
        double x_trans;
        double y_trans;
        double z_trans;
        double Xroll;
        double Xpitch;
        double Xyaw;
        double Xx_trans;
        double Xy_trans;
        double Xz_trans;
        double perpend;
        double bestCosine;
        double cross;
        double plane;
        double maxCross;
        int y, foundCount, planeCount;
        int saved;
        int ii, jj;
        double refTol;

        double saveSumDiff;
        double newSumDiff;
        permutes * combos;
        int32_t * revdex;
        int numout, ntries;

#ifdef ZDEBUG
int index;
#endif

        

        gFOM    = 0.0;
        gFOMavg = 0.0;
        gWorstTolReg = 0.0;
        gWorstTolAll = 0.0;
        gBestTolAll = 1.0;
        pLgMaster->gCoplanarCount = 0;


        ii = 3;
        while ( ii-- )
        {
                gBestOfAllTransform.transVector[ii] = 0.0;
                jj = 3;
                while ( jj-- ) gBestOfAllTransform.rotMatrix[ii][jj] = 0.0;
        }

	if ( gDontFindTranform )
	{
		IdentityArray ( theTransform );
		gDontFindTranform = false;
		return true;
	}
	
	if ( numberOfPoints < 4 ) return false;
	
 	i = 0;
 	while ( i < numberOfPoints )
 	{
 		tempPt[i].oldLoc[0] = gX[i];
 		tempPt[i].oldLoc[1] = gY[i];
 		tempPt[i].oldLoc[2] = gZ[i];
 		tempPt[i].xRad = foundAngles[i*2];
 		tempPt[i].yRad = foundAngles[i*2 + 1];

                pLgMaster->gColinear[i] = 0;
                pLgMaster->gCoplanar[i]   = 0;
/*
 *  kludge  for demo, assume only one part
 */
                if ( gRealTimeTransform ) {
 		    gSavePt[i].oldLoc[0] = gX[i];
 		    gSavePt[i].oldLoc[1] = gY[i];
 		    gSavePt[i].oldLoc[2] = gZ[i];
 		    gSavePt[i].xRad = foundAngles[i*2];
 		    gSavePt[i].yRad = foundAngles[i*2 + 1];
                }
#ifdef ZDEBUG
syslog(LOG_NOTICE , "gX %d %lf\n", i, gX[i] );
syslog(LOG_NOTICE , "gY %d %lf\n", i, gY[i] );
syslog(LOG_NOTICE , "gZ %d %lf\n", i, gZ[i] );
syslog(LOG_NOTICE , "xRad %d %.15lf\n", i, tempPt[i].xRad );
syslog(LOG_NOTICE , "yRad %d %.15lf\n", i, tempPt[i].yRad );
#endif

		i++;
 	}
 	

 	n = 3;
	while ( n-- )
	{
		tr.transVector[n] = 0.L;
		p = 3;
		while ( p-- )  tr.rotMatrix[n][p] = 0.L;
	}
	
 	nOfTrans = 0;
        saved = 0;
        for ( i=0; i<MAXTRANSNUM; i++ ) {
            minCos[i] = 2.0;
            saveGood[i] = 0;
            saveBest[i] = 0.0;
            savePoint[i] = 0;
            n = 3;
            while ( n-- ) {
              savetr[i].transVector[n] = 0.0;
              p = 3;
              while ( p-- )
                 savetr[i].rotMatrix[n][p] = 0.0;
            }
        }
        i = numberOfPoints - 2;
        maxCross = -1;
        while ( i-- )
        {
                if (pLgMaster->foundTarget[i] == 0 ) continue;
                j = numberOfPoints - 1;
                while ( --j > i )
                {
                        if (pLgMaster->foundTarget[j] == 0 ) continue;
                        k = numberOfPoints;
                        while ( --k > j )
                        {
                                if (pLgMaster->foundTarget[k] == 0 ) continue;
                                perpend = p3Dist(  &(tempPt[i])
                                                ,  &(tempPt[j])
                                                ,  &(tempPt[k])
                                                );
                                if ( perpend < minL3Distance ) {
                                     pLgMaster->gColinear[i]++;
                                     pLgMaster->gColinear[j]++;
                                     pLgMaster->gColinear[k]++;
                                }
                                cross = magCross(  &(tempPt[i])
                                                ,  &(tempPt[j])
                                                ,  &(tempPt[k])
                                                ,  &normPt
                                                );
                                foundCount = 0;
                                planeCount = 0;
                                if ( cross > maxCross && cross > 0.000001 ) {
                                      maxCross = cross;
                                      for ( y = 0; y < numberOfPoints; y++ ) {
                                          if (pLgMaster->foundTarget[y] != 0 ) {
                                              foundCount++;
                                              plane = planeDist( &(tempPt[y])
                                                               , &(tempPt[j])
                                                               , &normPt
                                                               );
                                              if ( plane < minL3Distance ) {
                                                   planeCount++;
                                                   pLgMaster->gCoplanar[y]++;
                                              }
                                          }
                                      }
                                }
                                if(foundCount == planeCount && planeCount > 3) {
                                      pLgMaster->gCoplanarCount++;
                                }
                        }
                }
        }

        combos = (permutes *)calloc( 200000, sizeof(permutes) );
        revdex = (int32_t *)calloc( 200000, sizeof(int32_t) );

        area_order(pLgMaster, numberOfPoints, tempPt, &numout, combos, revdex, pLgMaster->foundTarget);

        ntries = numout;
        if ( ntries > 100 ) ntries = 100;

        saved = 0;
        for ( i=1; i<= ntries; i++)
	  {
	    a = combos[revdex[i]].index1;
	    b = combos[revdex[i]].index2;
	    c = combos[revdex[i]].index3;
	    d = combos[revdex[i]].index4;
	    memcpy(&(iPt[0]),&(tempPt[a]),sizeof(inputPoint));
	    memcpy(&(iPt[1]),&(tempPt[b]),sizeof(inputPoint));
	    memcpy(&(iPt[2]),&(tempPt[c]),sizeof(inputPoint));
	    memcpy(&(iPt[3]),&(tempPt[d]),sizeof(inputPoint));
	    FindBestTransform(pLgMaster
                             , iPt
                             , &tempTr
                             , deltaMirror
                             , tolerance
                             , &bestCosine
                             );

#ifdef ZDEBUG
for( index=0; index<4; index++) {
syslog(LOG_NOTICE , "pt %d xyz %lf %lf %lf  angs %lf %lf", index,
          iPt[index].oldLoc[0], iPt[index].oldLoc[1],iPt[index].oldLoc[2],
          iPt[index].xRad, iPt[index].yRad );
}
syslog(LOG_NOTICE , "bestCosine %.8lf  ", bestCosine );
syslog(LOG_NOTICE , " %.8lf ", tolerance );
#endif

#ifdef ZDEBUG
syslog(LOG_NOTICE , "XYZ %14.8lf %14.8lf %14.8lf"
              , tempTr.transVector[0]
              , tempTr.transVector[1]
              , tempTr.transVector[2]
              );
syslog(LOG_NOTICE , "matrix " );
syslog(LOG_NOTICE , "%14.8lf ", tempTr.rotMatrix[0][0] );
syslog(LOG_NOTICE , "%14.8lf ", tempTr.rotMatrix[0][1] );
syslog(LOG_NOTICE , "%14.8lf ", tempTr.rotMatrix[0][2] );
syslog(LOG_NOTICE , "matrix " );
syslog(LOG_NOTICE , "%14.8lf ", tempTr.rotMatrix[1][0] );
syslog(LOG_NOTICE , "%14.8lf ", tempTr.rotMatrix[1][1] );
syslog(LOG_NOTICE , "%14.8lf ", tempTr.rotMatrix[1][2] );
syslog(LOG_NOTICE , "matrix " );
syslog(LOG_NOTICE , "%14.8lf ", tempTr.rotMatrix[2][0] );
syslog(LOG_NOTICE , "%14.8lf ", tempTr.rotMatrix[2][1] );
syslog(LOG_NOTICE , "%14.8lf ", tempTr.rotMatrix[2][2] );
#endif




              n = 3;
              while ( n-- ) {
                    savetr[saved].transVector[n] = tempTr.transVector[n];
                    p = 3;
                    while ( p-- )
                             savetr[saved].rotMatrix[n][p] =
                                                  tempTr.rotMatrix[n][p];
              }
              saveBest[saved] = bestCosine;
              tCos = 1.0 - bestCosine;
              if ( tCos > gWorstTolAll ) {
                                     gWorstTolAll = tCos;
              }
                                         // find the "best" transform
              if ( tCos < gBestTolAll ) {

                    pLgMaster->gBestTargetNumber   = 4;
                    pLgMaster->gBestTargetArray[0] = a + 1;
                    pLgMaster->gBestTargetArray[1] = b + 1;
                    pLgMaster->gBestTargetArray[2] = c + 1;
                    pLgMaster->gBestTargetArray[3] = d + 1;

                    gBestTolAll = tCos;
                    ii = 3;
                    while ( ii-- )
                    {
                                       gBestOfAllTransform.transVector[ii] =
                                               tempTr.transVector[ii];
                                       jj = 3;
                                       while ( jj-- )
                                           gBestOfAllTransform.rotMatrix[ii][jj]
                                               = tempTr.rotMatrix[ii][jj];
                    }
	      }
              if ( tCos < tolerance ) {
                                     // if ( tCos > gWorstTolReg ) {
                                     //      gWorstTolReg = tCos;
                                     // }
                       saveGood[saved]++;
                       savePoint[a]++;
                       savePoint[b]++;
                       savePoint[c]++;
                       savePoint[d]++;
                       nOfTrans++;
              }
              if ( tCos < minCos[a] ) { minCos[a] = tCos; }
              if ( tCos < minCos[b] ) { minCos[b] = tCos; }
              if ( tCos < minCos[c] ) { minCos[c] = tCos; }
              if ( tCos < minCos[d] ) { minCos[d] = tCos; }
              saved++;
	}
        free( combos );
        free( revdex );

        GnOfTrans = nOfTrans;
        gSaved = saved;
/*
 *  we must have one valid transform of some sort
 *  unlike the original program
 *
 */
        if ( nOfTrans < 1 ) return false;


/*
 *   copy (1-saveBest) into array
 *   and sort
 */
        if ( gSaved == 1 ) {
            refTol = 3.0 * gBestTolAll;
        } else {
            for ( i=0; i < gSaved; i++ ) {
                sortBest[i] = 1.0 - saveBest[i];
            }
            qsort( sortBest, gSaved, sizeof(double), dblsort );
            if ( gSaved == 2 ) {
               refTol = 3.0 * sortBest[1];
            } else {
               refTol = 3.0 * sortBest[2];
            }
        }
        gWorstTolReg = refTol;
        
#ifdef  FLASHLED
        FlashLed( (int)nOfTrans );
#endif

        goodcount = 0;
        if ( nOfTrans > 0  && tolerance < 0.002 ) {
           k = saved;
           while ( k-- ) {
              if ( saveGood[k] ) {
                 goodcount++;
                 i = 3;
                 while ( i-- )
                 {
                   tr.transVector[i] += savetr[k].transVector[i];
                   j = 3;
                   while ( j-- )  tr.rotMatrix[i][j] +=
                        savetr[k].rotMatrix[i][j];
                 }
              }
           }
           i = 3;
           while ( i-- )
           {
                tr.transVector[i] /= (double)goodcount;
                j = 3;
                while ( j-- )  tr.rotMatrix[i][j] /=
                        (double)goodcount;
           }
        } else {
 /*
  *  if there were too few transforms with tolerance,
  *  try to find something.
  */
           ii = 3;
           while ( ii-- )
           {
                tr.transVector[ii] = gBestOfAllTransform.transVector[ii];
                jj = 3;
                while ( jj-- )  tr.rotMatrix[ii][jj] =
                        gBestOfAllTransform.rotMatrix[ii][jj];
           }
        }


        DblMakeARotMatrix ( tr.rotMatrix );

/*
 * make figure of merit calculation
 */
        FOMmin = 100000.0;
        FOMsum = 0.0;
        usedPoints = 0;
        for ( i=0; i<numberOfPoints ; i++ ) {
             oldLoc[0] = gX[i];
             oldLoc[1] = gY[i];
             oldLoc[2] = gZ[i];
             TransformPoint( &tr, oldLoc, newLoc );
             newX = newLoc[0];
             newY = newLoc[1];
             newZ = newLoc[2];
             XYFromGeometricAnglesAndZ(pLgMaster, foundAngles[2*i], foundAngles[2*i+1],
				       newZ, &foundX, &foundY );
             gDiffX[i] = (double)(foundX - newX);
             gDiffY[i] = (double)(foundY - newY);
             diffMag = sqrt( gDiffX[i]*gDiffX[i] + gDiffY[i]*gDiffY[i] );
             if ( savePoint[i] ) {
                 FOMsum  += diffMag;
                 usedPoints++;
             }
             if( diffMag < FOMmin ) FOMmin = diffMag;
        }
        FOMavg = FOMsum / (double)usedPoints;
        gFOMavg = FOMsum / (double)usedPoints;
        gFOM    = FOMavg;
        testFOM = gFOM;
        for( iTr = 0; iTr < gSaved ; iTr++ ) {
           ii = 3;
           while ( ii-- )
           {
                testtr.transVector[ii] = savetr[iTr].transVector[ii];
                jj = 3;
                while ( jj-- )  {
                     testtr.rotMatrix[ii][jj] = savetr[iTr].rotMatrix[ii][jj];
                }
           }

            
/*
 * make figure of merit calculation
 */
            FOMmin = 100000.0;
            FOMsum = 0.0;
            usedPoints = 0;
            for ( i=0; i<numberOfPoints ; i++ ) {
                 oldLoc[0] = gX[i];
                 oldLoc[1] = gY[i];
                 oldLoc[2] = gZ[i];
                 TransformPoint( &testtr, oldLoc, newLoc );
                 newX = newLoc[0];
                 newY = newLoc[1];
                 newZ = newLoc[2];
                 XYFromGeometricAnglesAndZ(pLgMaster, foundAngles[2*i], foundAngles[2*i+1],
					   newZ, &foundX, &foundY );
                 gDiffX[i] = (double)(foundX - newX);
                 gDiffY[i] = (double)(foundY - newY);
                 diffMag = sqrt( gDiffX[i]*gDiffX[i] + gDiffY[i]*gDiffY[i] );
                 if ( savePoint[i] ) {
                     FOMsum  += diffMag;
                     usedPoints++;
                 }
                 if( diffMag < FOMmin ) FOMmin = diffMag;
            }
            FOMavg = FOMsum / (double)usedPoints;
                // if FOM is reduced, replace the existing transform
            if ( FOMavg < testFOM ) {
               testFOM = FOMavg;
               gFOMavg = FOMsum / (double)usedPoints;
               gFOM    = gFOMavg;
               ii = 3;
               while ( ii-- )
               {
                    tr.transVector[ii] = savetr[iTr].transVector[ii];
                    jj = 3;
                    while ( jj-- )  {
                         tr.rotMatrix[ii][jj] = savetr[iTr].rotMatrix[ii][jj];
                    }
               }
            }
        }


        gNPoints = numberOfPoints;
        gNumberOfPoints = numberOfPoints;

/*
 * there should be only two possibilities,
 * either there were enough transform
 * (and select points
 */

        if ( nOfTrans > 0 ) {
             gNPoints = 0;
             for ( i=0; i<numberOfPoints ; i++ ) {
                if ( savePoint[i] ) {
                  chiXfoundAngle[gNPoints] = foundAngles[2*i];
                  chiYfoundAngle[gNPoints] = foundAngles[2*i+1];
                  chiX[gNPoints] = gX[i];
                  chiY[gNPoints] = gY[i];
                  chiZ[gNPoints] = gZ[i];
                  gNPoints++;
                }
             }
        } else {
             for ( i=0; i<numberOfPoints ; i++ ) {
                  chiXfoundAngle[i] = foundAngles[2*i];
                  chiYfoundAngle[i] = foundAngles[2*i+1];
                  chiX[i] = gX[i];
                  chiY[i] = gY[i];
                  chiZ[i] = gZ[i];
             }
        }
        ndim = 6;
        ftol = 0.000001;
        TransformtoRPY ( &tr,&roll,&pitch,&yaw,&x_trans,&y_trans,&z_trans );
        Ys = (double *)calloc( 8, sizeof(double) );
        Pmatrix = (double **)calloc( 8, sizeof(double *) );
        for ( i=0; i<8; i++ ) {
            Pmatrix[i] = (double *)calloc( 8, sizeof(double) );
        }
        
        Pmatrix[1][1] = roll;
        Pmatrix[1][2] = pitch;
        Pmatrix[1][3] = yaw;
        Pmatrix[1][4] = x_trans;
        Pmatrix[1][5] = y_trans;
        Pmatrix[1][6] = z_trans;
        Ys[1] = chisqr(pLgMaster, &Pmatrix[1][0]);
        for ( i=2; i<=7; i++ ) {
            for ( j=1; j<=6; j++ ) {
                Pmatrix[i][j] = Pmatrix[1][j];
                if( (i-1) == j ) Pmatrix[i][j] += 0.01;
            }
            Ys[i] = chisqr(pLgMaster, &Pmatrix[i][0]);
        }

	/*  execute amoeba only if the tolerance was small */
        if ((nOfTrans > 0)  && (tolerance < 0.0001) && (gNPoints >= 4))
	  amoeba (pLgMaster, Pmatrix, Ys, ndim, ftol, &chisqr, &nfunk );
        roll    = Pmatrix[1][1];
        pitch   = Pmatrix[1][2];
        yaw     = Pmatrix[1][3];
        x_trans = Pmatrix[1][4];
        y_trans = Pmatrix[1][5];
        z_trans = Pmatrix[1][6];
        TransformfromRPY ( roll, pitch, yaw, x_trans, y_trans, z_trans, &tr );
 	TransformIntoArray ( &tr, theTransform );
        Ys[1] = chisqr(pLgMaster, &Pmatrix[1][0]);
        gChisqr = Ys[1];
/*
 *   do another filtering based on the gMaxDiffMag (if greater than zero)
 */
        if ( gMaxDiffMag > 0.000000001 && nOfTrans > 0 ) {
             saveSumDiff = 0.0;
             for ( i=0; i<numberOfPoints ; i++ ) {
                oldLoc[0] = gX[i];
                oldLoc[1] = gY[i];
                oldLoc[2] = gZ[i];
                TransformPoint( &tr, oldLoc, newLoc );
                newX = newLoc[0];
                newY = newLoc[1];
                newZ = newLoc[2];
                XYFromGeometricAnglesAndZ(pLgMaster, foundAngles[2*i], foundAngles[2*i+1],
					  newZ, &foundX, &foundY );
                gDiffX[i] = (double)(foundX - newX);
                gDiffY[i] = (double)(foundY - newY);
                gDiffMag[i] = sqrt( gDiffX[i]*gDiffX[i] + gDiffY[i]*gDiffY[i] );
                if ( savePoint[i] != 0 ) {
                   saveSumDiff += gDiffMag[i];
                } 
             } 
             for ( i=0; i<numberOfPoints ; i++ ) {
                oldLoc[0] = gX[i];
                oldLoc[1] = gY[i];
                oldLoc[2] = gZ[i];
                TransformPoint( &tr, oldLoc, newLoc );
                newX = newLoc[0];
                newY = newLoc[1];
                newZ = newLoc[2];
                XYFromGeometricAnglesAndZ(pLgMaster, foundAngles[2*i], foundAngles[2*i+1],
					  newZ, &foundX, &foundY);
                gDiffX[i] = (double)(foundX - newX);
                gDiffY[i] = (double)(foundY - newY);
                gDiffMag[i] = sqrt( gDiffX[i]*gDiffX[i] + gDiffY[i]*gDiffY[i] );
                if (gDiffMag[i] > gMaxDiffMag)
		  savePoint[i] = 0;
             }
             gNPoints = 0;
             for (i=0; i<numberOfPoints ; i++)
	       {
		 if (savePoint[i])
		   {
		     chiXfoundAngle[gNPoints] = foundAngles[2*i];
		     chiYfoundAngle[gNPoints] = foundAngles[2*i+1];
		     chiX[gNPoints] = gX[i];
		     chiY[gNPoints] = gY[i];
		     chiZ[gNPoints] = gZ[i];
		     gNPoints++;
		   }
	       }
             ndim = 6;
             ftol = 0.000001;
             TransformtoRPY( &tr,&roll,&pitch,&yaw,&x_trans,&y_trans,&z_trans );
             Y2s = (double *)calloc( 8, sizeof(double) );
             Qmatrix = (double **)calloc( 8, sizeof(double *) );
             for (i=0; i<8; i++)
                 Qmatrix[i] = (double *)calloc( 8, sizeof(double) );
             Qmatrix[1][1] = roll;
             Qmatrix[1][2] = pitch;
             Qmatrix[1][3] = yaw;
             Qmatrix[1][4] = x_trans;
             Qmatrix[1][5] = y_trans;
             Qmatrix[1][6] = z_trans;
             Y2s[1] = chisqr(pLgMaster, &Qmatrix[1][0]);

             for ( i=2; i<=7; i++ ) {
                 for ( j=1; j<=6; j++ ) {
                     Qmatrix[i][j] = Qmatrix[1][j];
                     if( (i-1) == j ) Qmatrix[i][j] += 0.01;
                 }
                 Y2s[i] = chisqr(pLgMaster, &Pmatrix[i][0]);
             }
	     /*  execute amoeba only if the tolerance was small */
             if ( nOfTrans > 0  && tolerance < 0.0001 && gNPoints >= 4 )
	       {
		 amoeba(pLgMaster, Qmatrix, Y2s, ndim, ftol, &chisqr, &nfunk);
		 Xroll    = Qmatrix[1][1];
		 Xpitch   = Qmatrix[1][2];
		 Xyaw     = Qmatrix[1][3];
		 Xx_trans = Qmatrix[1][4];
		 Xy_trans = Qmatrix[1][5];
		 Xz_trans = Qmatrix[1][6];
		 TransformfromRPY(Xroll, Xpitch, Xyaw, Xx_trans, Xy_trans, Xz_trans, &Xtr);
		 newSumDiff = 0.0;
		 for ( i=0; i<numberOfPoints ; i++ ) {
                     oldLoc[0] = gX[i];
                     oldLoc[1] = gY[i];
                     oldLoc[2] = gZ[i];
                     TransformPoint( &Xtr, oldLoc, newLoc );
                     newX = newLoc[0];
                     newY = newLoc[1];
                     newZ = newLoc[2];
                     XYFromGeometricAnglesAndZ(pLgMaster, foundAngles[2*i], foundAngles[2*i+1],
					       newZ, &foundX, &foundY);
                     gDiffX[i] = (double)(foundX - newX);
                     gDiffY[i] = (double)(foundY - newY);
                     gDiffMag[i] = sqrt( gDiffX[i]*gDiffX[i] + gDiffY[i]*gDiffY[i] );
                     if ( savePoint[i] != 0 ) {
                        newSumDiff += gDiffMag[i];
                     } 
                  } 
                  if ( newSumDiff < saveSumDiff ) {
                     roll    = Qmatrix[1][1];
                     pitch   = Qmatrix[1][2];
                     yaw     = Qmatrix[1][3];
                     x_trans = Qmatrix[1][4];
                     y_trans = Qmatrix[1][5];
                     z_trans = Qmatrix[1][6];
                     TransformfromRPY(roll, pitch, yaw, x_trans, y_trans, z_trans, &tr);
     	             TransformIntoArray ( &tr, theTransform );
                     Y2s[1] = chisqr(pLgMaster, &Qmatrix[1][0]);
                     gChisqr = Y2s[1];
                  }
             }

        }

#ifdef ZDEBUG
syslog( LOG_NOTICE, "chisqr (after)  %14.8lf", Ys[1]);
syslog( LOG_NOTICE, "RPY %14.8lf %14.8lf %14.8lf", roll, pitch, yaw );
syslog( LOG_NOTICE, "XYZ %14.8lf %14.8lf %14.8lf", x_trans, y_trans, z_trans );
syslog( LOG_NOTICE, "matrix " );
syslog( LOG_NOTICE, "%14.8lf ", tr.rotMatrix[0][0] );
syslog( LOG_NOTICE, "%14.8lf ", tr.rotMatrix[0][1] );
syslog( LOG_NOTICE, "%14.8lf ", tr.rotMatrix[0][2] );
syslog( LOG_NOTICE, "matrix " );
syslog( LOG_NOTICE, "%14.8lf ", tr.rotMatrix[1][0] );
syslog( LOG_NOTICE, "%14.8lf ", tr.rotMatrix[1][1] );
syslog( LOG_NOTICE, "%14.8lf ", tr.rotMatrix[1][2] );
syslog( LOG_NOTICE, "matrix " );
syslog( LOG_NOTICE, "%14.8lf ", tr.rotMatrix[2][0] );
syslog( LOG_NOTICE, "%14.8lf ", tr.rotMatrix[2][1] );
syslog( LOG_NOTICE, "%14.8lf ", tr.rotMatrix[2][2] );
#endif



/*
 * make figure of merit calculation
 */
	FOMmin = 100000.0;
	FOMsum = 0.0;
        usedPoints = 0;
        for ( i=0; i<numberOfPoints ; i++ ) {
             oldLoc[0] = gX[i];
             oldLoc[1] = gY[i];
             oldLoc[2] = gZ[i];
             TransformPoint( &tr, oldLoc, newLoc );
             newX = newLoc[0];
             newY = newLoc[1];
             newZ = newLoc[2];
             XYFromGeometricAnglesAndZ(pLgMaster, foundAngles[2*i], foundAngles[2*i+1],
				       newZ, &foundX, &foundY);
             gDiffX[i] = (double)(foundX - newX);
             gDiffY[i] = (double)(foundY - newY);
             gDiffMag[i] = sqrt( gDiffX[i]*gDiffX[i] + gDiffY[i]*gDiffY[i] );
             if ( savePoint[i] ) {
                 FOMsum  += gDiffMag[i];
                 usedPoints++;
             }
	     if( gDiffMag[i] < FOMmin ) FOMmin = gDiffMag[i];
        } 
        gFOM = FOMsum / (double)usedPoints;
	/*
	 *  free the memory
	 */
        free(Ys);
        for ( i=0; i<8; i++ ) {
            free(Pmatrix[i]);
        }
	// FIXME---PAH---NEED TO FREE UP QMATRIX TOO???
        free(Pmatrix);
	return true;
}


double magCross(doubleInputPoint *iPt0, doubleInputPoint *iPt1,
		doubleInputPoint *iPt2, doubleInputPoint *Pnorm)
{
        double ax1, ax2, ax3;
        double ay1, ay2, ay3;
        double az1, az2, az3;
        double del21x, del21y, del21z;
        double del23x, del23y, del23z;
        double mag;
        double xNorm;
        double yNorm;
        double zNorm;
        double xCross;
        double yCross;
        double zCross;

        ax1 = iPt0->oldLoc[0];
        ay1 = iPt0->oldLoc[1];
        az1 = iPt0->oldLoc[2];
        ax2 = iPt1->oldLoc[0];
        ay2 = iPt1->oldLoc[1];
        az2 = iPt1->oldLoc[2];
        ax3 = iPt2->oldLoc[0];
        ay3 = iPt2->oldLoc[1];
        az3 = iPt2->oldLoc[2];

        del21x = ax2 - ax1;
        del21y = ay2 - ay1;
        del21z = az2 - az1;

        del23x = ax2 - ax3;
        del23y = ay2 - ay3;
        del23z = az2 - az3;

        xCross = del21y * del23z - del21z * del23y;
        yCross = del21z * del23x - del21x * del23z;
        zCross = del21x * del23y - del21y * del23x;

        mag = sqrt( xCross*xCross + yCross*yCross + zCross*zCross );

        if ( mag < 0.0000001 ) {
             xNorm = 0.0;
             yNorm = 0.0;
             zNorm = 0.0;
             return( 0.0 );
        }

        xNorm = xCross / mag;
        yNorm = yCross / mag;
        zNorm = zCross / mag;

        Pnorm->oldLoc[0] = xNorm;
        Pnorm->oldLoc[1] = yNorm;
        Pnorm->oldLoc[2] = zNorm;
        return( mag );
}


double
planeDist ( doubleInputPoint *point
          , doubleInputPoint *iPt0
          , doubleInputPoint *pPerpend
          )
{
        double ax1, ax2;
        double ay1, ay2;
        double az1, az2;
        double del21x, del21y, del21z;
        double mag;
        double xNorm;
        double yNorm;
        double zNorm;
        double xDot;
        double yDot;
        double zDot;

        ax1 = point->oldLoc[0];
        ay1 = point->oldLoc[1];
        az1 = point->oldLoc[2];
        ax2 = iPt0->oldLoc[0];
        ay2 = iPt0->oldLoc[1];
        az2 = iPt0->oldLoc[2];

        xNorm = pPerpend->oldLoc[0];
        yNorm = pPerpend->oldLoc[1];
        zNorm = pPerpend->oldLoc[2];

        del21x = ax2 - ax1;
        del21y = ay2 - ay1;
        del21z = az2 - az1;


        xDot = del21x * xNorm;
        yDot = del21y * yNorm;
        zDot = del21z * zNorm;

        mag = fabs( xDot + yDot + zDot );

        return( mag );
}

int
dblsort( const void *elem1, const void *elem2 )
{
    double numone, numtwo;

    numone = *(const double *)elem1;
    numtwo = *(const double *)elem2;

    if ( numone < numtwo ) return -1;
    if ( numone > numtwo ) return  1;

    return 0;
}

