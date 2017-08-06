/*   $Id: SensorRegistration.h,v 1.7 2007/04/02 08:43:56 pickle Exp pickle $  */
#ifndef SENSORREGISTRATION_H
#define SENSORREGISTRATION_H

#define  MAXTRANSNUM  11000


uint32_t get_number_of_points(void);
double get_chisquare_val(void);
void get_target_info(struct k_targetinfo *pTgtInfo, uint32_t tgt_num);
unsigned char FindTransformMatrix(struct lg_master *pLgMaster, uint16_t numberOfPoints, 
				  double deltaMirror, double tolerance, double *foundAngles,
				  double *theTransform);
void SaveFullRegCoordinates(uint16_t numberOfPoints, double *theOriginalCoordinates);
void DontFindTransform(unsigned char doOrDont);

extern short GnOfTrans;
extern int   gSaved;
extern double gX[kNumberOfFlexPoints];
extern double gY[kNumberOfFlexPoints];
extern double gZ[kNumberOfFlexPoints];
extern double gXfoundAngle[kNumberOfFlexPoints];
extern double gYfoundAngle[kNumberOfFlexPoints];
extern double chiX[kNumberOfFlexPoints];
extern double chiY[kNumberOfFlexPoints];
extern double chiZ[kNumberOfFlexPoints];
extern double chiXfoundAngle[kNumberOfFlexPoints];
extern double chiYfoundAngle[kNumberOfFlexPoints];

extern int  gNPoints;

extern double gDiffX[MAXTRANSNUM];
extern double gDiffY[MAXTRANSNUM];

extern int savePoint[MAXTRANSNUM];

extern double gWorstTolReg;
extern double gWorstTolAll;
extern double gBestTolAll;

#endif
