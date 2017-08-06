#include <stdint.h>
// static char rcsid[] = "$Id: Events.c,v 1.2 1997/05/11 23:25:30 ags-sw Exp $";

#include "AppCommon.h"
#include <time.h>

uint32_t TickCount ( void )
{
	return (uint32_t)( 60L * clock (  ) );
}

