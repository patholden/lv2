#include <stdint.h>
/*   $Id: ROI.h,v 1.2 1997/10/10 20:32:05 ags-sw Exp $   */

extern      void    HandleROI ( char * parameters,
                                uint32_t respondToWhom );

extern int32_t  g_percent;
extern int32_t  g_half_pattern;
extern int32_t  g_roi_on;
