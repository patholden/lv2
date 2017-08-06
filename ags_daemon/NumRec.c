#include <stdint.h>
static	double	LUDCMP ( double a[6][6], short indx[6] );
static	void		LUBKSB ( double a[6][6], short indx[6],
						double b[6] );

void LUBKSB ( double a[6][6], short indx[6], double b[6] )
{
	short ii, i, j, ll;
	double sum;
	ii = -1;
	for ( i = 0; i < 6; i++ )									/* 12 */
	{
		ll = indx[i];
		sum = b[ll];
		b[ll] = b[i];
		if ( ii != -1 )
		{
			for ( j = ii; j < i; j++ )							/* 11 */
			{
				sum = sum - a[i][j] * b[j];
			}													/* 11 */
		}
		else 
		{
			if ( fabs ( sum ) > FLT_EPSILON ) ii = i;
		}
		b[i] = sum;
	}															/* 12 */
	for ( i = 5; i >= 0; i-- )									/* 14 */
	{
		sum = b[i];
		for ( j = i + 1; j < 6; j++ )							/* 13 */
		{
			sum = sum - a[i][j] * b[j];
		}														/* 13 */
		b[i] = sum / a[i][i];
	}															/* 14 */
}

double LUDCMP ( double a[6][6], short indx[6] )
{
	double d, sum, aamax, vv[6], dum;
	short i, j, k, imax;
	d = 1.L;
	for ( i = 0; i < 6; i++ )									/* 12 */
	{
		aamax = 0.L;
		for ( j = 0; j < 6; j++ )								/* 11 */
		{
			if ( fabs ( a[i][j] ) > aamax ) aamax = fabs ( a[i][j] );
		}														/* 11 */
		if ( fabs ( aamax ) < FLT_EPSILON ) return 0.L;
		vv[i] = 1.L / aamax;
	}															/* 12 */
	for ( j = 0; j < 6; j++ )									/* 19 */
	{
		for ( i = 0; i < j; i++ )								/* 14 */
		{
			sum = a[i][j];
			for ( k = 0; k < i; k++ )
			{													/* 13 */
				sum = sum - a[i][k] * a[k][j];
			}													/* 13 */
			a[i][j] = sum;	
		}														/* 14 */
		aamax = 0.L;
		for ( i = j; i < 6; i++ )								/* 16 */
		{
			sum = a[i][j];
			for ( k = 0; k < j; k++ )							/* 15 */
			{
				sum = sum - a[i][k] * a[k][j];
			}													/* 15 */
			a[i][j] = sum;
			dum = vv[i] * fabs ( sum );
			if ( dum >= aamax )
			{
				imax = i;
				aamax = dum;
			}
		}														/* 16 */
		if ( j != imax )
		{
			for ( k = 0; k < 6; k++ )							/* 17 */
			{
				dum = a[imax][k];
				a[imax][k] = a[j][k];
				a[j][k] = dum;
			}													/* 17 */
			d *= -1.L;
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if ( fabs ( a[j][j] ) < FLT_EPSILON ) return 0.L;
		if ( j != 5 )
		{
			dum = 1.L / a[j][j];
			for ( i = j + 1; i < 6; i++ )						/* 18 */
			{
				a[i][j] = a[i][j] * dum;
			}													/* 18 */
		}
	}															/* 19 */
	return d;
}
