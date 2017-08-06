/*   $Id: AutoCert.h,v 1.6 1999/07/29 19:49:42 ags-sw Exp $  */
#ifndef AUTOCERT_H
#define AUTOCERT_H

#define	kMaxNumberOfSensorsInBGD	100

int DoAutoCert(double Xin, double Yin, double Zin,
	       double *Xexpect, double *Yexpect,
	       double *Xfound,  double *Yfound);
void HandleAutoCert(int int32_tDoubleFlag, char *parameters,  uint32_t respondToWhom);

extern	double			gTolerance;
extern	double			gExtraTolerance;
extern	double			gInnerTolerance;
extern	int32_t				gProjectorSerialNumber;
extern	char				gBGDFileName[10];
extern	char				gOperatorName[255];

extern	short				gNumberOfDefinedSensors;
extern	short				gCurrentSensor;


typedef struct
{
	unsigned char				found;
	unsigned char				inTolerance;

	double			xDesired;
	double			yDesired;
	double			zDesired;
	double			xAngleDesired;
	double			yAngleDesired;

	double			xExpected;
	double			yExpected;
	double			xAngleExpected;
	double			yAngleExpected;
	uint32_t		xBinaryExpected;
	uint32_t		yBinaryExpected;

	double			xFound;
	double			yFound;
	double			xAngleFound;
	double			yAngleFound;
	uint32_t		xBinaryFound;
	uint32_t		yBinaryFound;
} BGDSensor;

extern	BGDSensor *gSensors;
#endif
