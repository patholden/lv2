#include <stdint.h>
/*
 * $Id: amoeba.h,v 1.1 1999/03/19 18:39:25 ags-sw Exp $
 */

void amoeba(struct lg_master *pLgMaster, double **p, double *y, int ndim, double ftol,
	    double (*funk)(struct lg_master *,double []), int *nfunk);
