/*   $Id: AngleCorrections.h,v 1.4 1999/07/29 19:56:56 ags-sw Exp $  */

#ifndef ANGLECORR_H
#define ANGLECORR_H

void RemovePoly(struct lg_master *pLgMaster, double *x, double *y);
void ApplyPoly(struct lg_master *pLgMaster, double *x, double *y);
void RemoveCorrection(struct lg_master *pLgMaster, double *x, double *y);
void ApplyCorrection(struct lg_master *pLgMaster, double *x, double *y);
unsigned char InitAngleCorrections(struct lg_master *pLgMaster);
unsigned char InitPoly(struct lg_master *pLgMaster);
void CloseAngleCorrections(void);

#endif // ANGLECORR_H
