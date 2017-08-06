/*   $Id: LocalComm.h,v 1.2 1996/12/25 18:38:04 ags-sw Exp $  */
#ifndef LOCALCOMM_H
#define LOCALCOMM_H

void SaveResponse ( Ptr theResponse, int32_t length );
void SetLocalMode ( Boolean itsLocal );
void LocalStop ( void );
void LocalDisplayWithoutTransform ( void );
void LocalDisplayWithTransform ( void );
void LocalNewDisplayWithTransform ( void );
void LocalDisplayNoQuickCheck ( void );
void CloseLocalMode ( void );
void SetSensorHere ( short sensorNumber );
void LocalSearchForASensor ( short sensorNumber );
void InitLocalMode ( void );
void LocalSensorSearchFromHere ( void );
void LocalFullRegistration ( void );
void GetLocalSensorCoordinates ( void );
void DoAutoCert ( void );
void DoInspection ( void );
void LocalGoAngle ( double x, double y );
Boolean	LocalGoAngleSuccess ( void );
void DoLocalWindowClick ( Point pt );
#endif
