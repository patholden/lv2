/*
 * static char rcsid[] = "$Id: chisqr.c,v 1.4 2007/04/02 08:46:27 pickle Exp pickle $";
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "parse_data.h"
#include "Protocol.h"
#include "3DTransform.h"
#include "SensorRegistration.h"
#include "LaserInterface.h"
#include "chisqr.h"
#include "angles.h"


double chisqr(struct lg_master *pLgMaster, double params[])
{
    double roll, pitch, yaw;
    double xtrans, ytrans, ztrans;
    transform tr;
    int i;
    double oldLoc[3];
    double newLoc[3];
    double newX;
    double newY;
    double newZ;
    double angX;
    double angY;
    double delAngX;
    double delAngY;
    double chisum;
    double angSQR;
    
    roll   = params[1];
    pitch  = params[2];
    yaw    = params[3];
    xtrans = params[4];
    ytrans = params[5];
    ztrans = params[6];

    TransformfromRPY( roll, pitch, yaw, xtrans, ytrans, ztrans, &tr );

    chisum = 0.0;
    for ( i=0; i<gNPoints; i++ ) {
        oldLoc[0] = chiX[i];
        oldLoc[1] = chiY[i];
        oldLoc[2] = chiZ[i];
        TransformPoint( &tr, oldLoc, newLoc );
        newX = newLoc[0];
        newY = newLoc[1];
        newZ = newLoc[2];
        
        GeometricAnglesFrom3D(pLgMaster,newX,newY,newZ,&angX,&angY);
        delAngX = fabs( angX - chiXfoundAngle[i] );
        delAngY = fabs( angY - chiYfoundAngle[i] );
        angSQR  = delAngX*delAngX + delAngY*delAngY;
          /*
           *  this normalization factor of 7.4e6
           *  is supposed to give a chi squared of 1.0
           *  if all the errors are 0.015 inches at 10 feet
           */
        angSQR *= 7.4e6;
        chisum += angSQR;
    }

    return chisum;
}
