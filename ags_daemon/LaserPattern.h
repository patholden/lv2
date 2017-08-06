/*   $Id: LaserPattern.h,v 1.5 1999/07/29 19:03:59 ags-sw Exp $  */
#ifndef LASERPATTERN_H
#define LASERPATTERN_H

void CloseLaserPattern(void);
int InitLaserPattern(void);
uint32_t PutGoTo2D(struct lg_master *pLgMaster, double x, double y);
uint32_t PutGoTo3D(struct lg_master *pLgMaster, double x, double y, double z);
uint32_t FinishPattern(struct lg_master *pLgMaster);
void PendPenDown(void);
void UnpendPenDown(void);
void SetPenDown(void);
void SetPenUp(void);
void SetDefaultZ(double z);
struct lg_xydata *SetUpLaserPattern(struct lg_master *pLgMaster, double *transform);
void ChangeTransform(double *transform);
uint32_t Transform3DPointToBinary(struct lg_master *pLgMaster, double x, double y, double z,
				  int16_t *xAngle, int16_t *yAngle);
uint32_t PointToBinary(struct lg_master *pLgMaster, double *point, int16_t *xAngle, int16_t *yAngle);
extern double gMaxCos;

extern  double gCurveMin;
extern  double gCurveMax;

extern  double gLongToShort;
#endif
