/*
 * static char rcsid[] = "$Id: amoeba.c,v 1.2 1999/05/04 15:35:47 ags-sw Exp $";
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <stddef.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <sys/io.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/ioctl.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "amoeba.h"

#define NMAX 500

#define  GET_PSUM \
            for (j=1;j<=ndim;j++) { \
            for (sum=0.0,i=1;i<=mpts;i++) sum += p[i][j]; \
            psum[j]=sum; }

#define SWAP(a,b) {swap=(a);(a)=(b);(b)=swap;}

static double amotry(struct lg_master *pLgMaster, double **p, double *y, double *psum, int ndim,
		     double(*funk)(struct lg_master *,double []), int ihi, double fac);

static double *vector(int32_t nl, int32_t nh);

static void free_vector( double *v, int32_t nl, int32_t nh );

void amoeba(struct lg_master *pLgMaster, double **p, double *y, int ndim, double ftol,
            double (*funk)(struct lg_master *, double []), int *nfunk)
{
  int i, ihi, ilo, inhi, j, mpts;
  double rtol, sum, swap, ysave, ytry, *psum;

  mpts=ndim+1;

  psum=vector(1,ndim);
  *nfunk=0;
  GET_PSUM
  for (;;) {
    ilo=1;
    if ( y[1]>y[2] ) {
      inhi=2;
      ihi=1;
    } else {
      inhi=1;
      ihi=2;
    }
    for (i=1;i<mpts;i++) {
      if (y[i] <= y[ilo]) ilo=i;
      if (y[i] >  y[ihi]) {
        inhi=ihi;
        ihi=i;
      } else if ( (y[i]>y[inhi]) && (i!=ihi) ) inhi=i;
    }
    rtol=2.0*fabs(y[ihi]-y[ilo])/(fabs(y[ihi])+fabs(y[ilo]));
    if (rtol < ftol) {
       SWAP(y[1],y[ilo])
       for (i=1;i<=ndim;i++) SWAP(p[1][i],p[ilo][i])
       break;
    }
    if (*nfunk >= NMAX) {
      syslog(LOG_ERR, "NMAX  exceeded");
      return;
    }
    *nfunk += 2;
    ytry=amotry(pLgMaster, p,y,psum,ndim,funk,ihi,-1.0);
    if (ytry <= y[ilo]) {
      ytry=amotry(pLgMaster, p,y,psum,ndim,funk,ihi,2.0);
    } else if (ytry >= y[inhi]) {
      ysave=y[ihi];
      ytry=amotry(pLgMaster, p,y,psum,ndim,funk,ihi,0.5);
      if (ytry >= ysave) {
        for (i=1;i<=mpts;i++) {
          if (i != ilo) {
            for (j=1;j<=ndim;j++) {
              psum[j]=0.5*(p[i][j]+p[ilo][j]);
              p[i][j]=psum[j];
            }
            y[i]=(*funk)(pLgMaster, psum);
          }
        }
        *nfunk += ndim;
        GET_PSUM
      }
    } else --(*nfunk);
  }
  free_vector(psum,1,ndim);
  return;
}

double amotry(struct lg_master *pLgMaster, double **p, double *y, double *psum, int ndim,
	      double(*funk)(struct lg_master *,double []), int ihi, double fac)
{
  int j;
  double fac1,fac2,ytry,*ptry;

  ptry=vector(1,ndim);
  fac1=(1.0-fac)/ndim;
  fac2=fac1-fac;
  for (j=1;j<=ndim;j++)
      ptry[j]=psum[j]*fac1-p[ihi][j]*fac2;
  ytry=(*funk)(pLgMaster, ptry);
  if (ytry < y[ihi]) {
    y[ihi]=ytry;
    for (j=1;j<=ndim;j++) {
      psum[j] += ptry[j]-p[ihi][j];
      p[ihi][j]=ptry[j];
    }
  }
  free_vector(ptry,1,ndim);
  return ytry;
}

#define NR_END 1

double *vector(int32_t nl, int32_t nh)
{
  double *v;

  v=(double *)calloc((size_t)(nh-nl+1+NR_END),sizeof(double));
  if (!v) {
    syslog(LOG_ERR, "allocation failure in vector()\n" );
  }
  return(v-nl+NR_END);
}

void free_vector( double *v, int32_t nl, int32_t nh )
{
  free((char *)(v+nl-NR_END));
}
