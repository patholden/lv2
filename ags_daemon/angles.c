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
#include "3DTransform.h"
#include "angles.h"

int TransformtoRPY (transform *m, double *roll, double *pitch, 
		double *yaw, double *x_trans, double *y_trans, 
		double *z_trans)

{

	double sinph,cosph,sinka,sinom,w,k,p,w_plus_k;
	int OK;
        /* Conventions are from : P.R. Wolf, Elements of Photogrammetry,
         *    2nd Edition, New York:McGraw-Hill, 1983
         */
        /* Wolf omega = roll */
        /*      phi     pitch  */
        /*      kappa   yaw    */
	
	if (!m)
	  return (0);
	OK = 1;
        /* note possibility of numerical difficulties
         * if divisors below are small
         */
	sinph = m->rotMatrix[2][0];
	if (fabs(sinph) < 1.0) {
		p = asin(sinph);
		cosph = cos(p);
		sinom = -(m->rotMatrix[2][1]/cosph);
		if (fabs(sinom) <= 1.0) w = asin(sinom);
		else OK = 0;
		sinka = -(m->rotMatrix[1][0]/cosph);
		if (fabs(sinka) <= 1.0) k = asin(sinka);
		else OK = 0;
		/* change omega and kappa if in wrong quadrant */
		if (m->rotMatrix[0][0] < 0.0) k = M_PI - k;
		if (m->rotMatrix[2][2] < 0.0) w = M_PI - w;
	}
	else {
		if (fabs(m->rotMatrix[0][1]) <= 1.0) {
			w_plus_k = asin(m->rotMatrix[0][1]);
			if (m->rotMatrix[0][2] > 0.0) {
                            w_plus_k = M_PI - w_plus_k;
                        }
			k = w_plus_k*0.5;
			if (m->rotMatrix[0][1] > 0.0) w = k;
			else w = -k;
		}
		else OK = 0;
	}
	if (OK) {
                *roll = w;
                *pitch = p;
                *yaw = k;
                *x_trans = m->transVector[0];
                *y_trans = m->transVector[1];
                *z_trans = m->transVector[2];
	}
	return (OK);
}

int TransformfromRPY (double roll, double pitch, double yaw,
		double x_trans, double y_trans, double z_trans,
		transform *tr)
{
	double sinph,cosph,sinka,coska,sinom,cosom;
 
        /* Conventions are from : P.R. Wolf, Elements of Photogrammetry,
         * 2nd Edition,
	 * New York:McGraw-Hill, 1983
         */
        /* Wolf omega = roll */
        /*      phi     pitch  */
        /*      kappa   yaw    */

	if (!tr)
	  return(0);
	
        sinph = sin(pitch);
        cosph = cos(pitch);
        sinka = sin(yaw);
        coska = cos(yaw);
        sinom = sin(roll);
        cosom = cos(roll);

	tr->rotMatrix[0][0] = cosph*coska;
	tr->rotMatrix[0][1] = sinom*sinph*coska + cosom*sinka;
	tr->rotMatrix[0][2] = -cosom*sinph*coska + sinom*sinka;
	tr->rotMatrix[1][0] = -cosph*sinka;
	tr->rotMatrix[1][1] = -sinom*sinph*sinka + cosom*coska;
	tr->rotMatrix[1][2] = cosom*sinph*sinka + sinom*coska;
	tr->rotMatrix[2][0] = sinph;
	tr->rotMatrix[2][1] = -sinom*cosph;
	tr->rotMatrix[2][2] = cosom*cosph;
	tr->transVector[0] = x_trans;
	tr->transVector[1] = y_trans;
	tr->transVector[2] = z_trans;


	return (1);
}
