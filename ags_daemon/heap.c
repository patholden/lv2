#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "heap.h"

#define SWAP(a,b) itemp=(a);(a)=(b);(b)=itemp;

void heap( int32_t n, double *arr, int32_t *indx )
{
  int32_t i, indxt, ir, j, l;
  double rra;
  double raj, raj1;

  if ( n < 2 ) return;
  l = (n >> 1) + 1;
  ir = n;
 
  for (j=1;j<=n;j++) indx[j]=j;

  for (;;) {
    if ( l > 1 ) {
       // rra = ra[--l];
       l--;
       indxt = indx[l];
       rra = arr[indxt];
    } else {
       // rra = ra[ir];
       indxt = indx[ir];
       rra = arr[indxt];
       // ra[ir]=ra[1];
       indx[ir] = indx[1];
       if ( --ir == 1 ) {
         // ra[1] = rra;
         indx[1] = indxt;
         break;
       }
    }
    i = l;
    j = l+1;
    while ( j <= ir ) {
      if ( j < ir ) {
        raj  = arr[indx[j]];
        raj1 = arr[indx[j+1]];
        // raj  = ra[j];
        // raj1 = ra[j+1];
        if (raj < raj1) j++;
      }
      raj  = arr[indx[j]];
      // raj  = ra[j];
      if ( rra < raj) {
        // ra[i] = ra[j];
        indx[i] = indx[j];
        i = j;
        j <<= 1;
      } else break;
    }
    // ra[i]=rra;
    indx[i] = indxt;

  }
}
