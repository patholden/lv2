#include <stdint.h>
/*   $Id: SensorSearch.h,v 1.12 1999/07/29 18:36:01 ags-sw Exp $  */

#ifndef DOUBLESENSORSEARCH_H
#define DOUBLESENSORSEARCH_H

int DoubleFineSearch(int16_t *foundX, int16_t *foundY);


extern
int DoubleSearchForASensor(struct lg_master *pLgMaster
                          , int16_t startX
                          , int16_t startY
                          , double *foundX
                          , double *foundY
                          )
;


extern
int DoubleCoarseLeg(struct lg_master *pLgMaster
                          , int16_t Xin
                          , int16_t Yin
                          , int16_t delX
                          , int16_t delY
                          , uint32_t nStepsIn
                          , double  *foundX
                          , double  *foundY
                          )
;

extern
int DoubleFineLevel(struct lg_master *pLgMaster
                          , int16_t inpX
                          , int16_t inpY
                          , double  *foundX
                          , double  *foundY
                          , int16_t *outMinX
                          , int16_t *outMinY
                          , int16_t *outMaxX
                          , int16_t *outMaxY
                          )
;

extern
int doubleFirstLast(struct lg_master *pLgMaster
                          , int16_t *firstX
                          , int16_t *firstY
                          , int16_t *lastX
                          , int16_t *lastY
                          , double  *foundX
                          , double  *foundY
                          , int16_t currentX
                          , int16_t currentY
                          , int16_t xStep
                          , int16_t yStep
                          , uint16_t nSteps
                          )
;

extern
int doubleSuperFirstLast(struct lg_master *pLgMaster
                         , int16_t *firstX
                         , int16_t *firstY
                         , int16_t *lastX
                         , int16_t *lastY
                         , double  *foundX
                         , double  *foundY
                         , int16_t currentX
                         , int16_t currentY
                         , int16_t xStep
                         , int16_t yStep
                         , uint16_t nSteps
                         )
;

extern
int DoubleSuperSearch(struct lg_master *pLgMaster
               , double  *foundX
               , double  *foundY
               , int16_t *MinX
               , int16_t *MinY
               , int16_t *MaxX
               , int16_t *MaxY
               )
;

#endif

#ifndef kStopWasDone
#define kStopWasDone  -1
#endif
