#include <stdint.h>
/*   $Id: LinuxErrors.h,v 1.2 1996/12/25 18:38:04 ags-sw Exp $  */

#include <errno.h>
#include <string.h>

#define LinuxError() (errno)
#define LinuxErrStr(n) (strerror(n))
#define LinuxErrorInit() (errno = 0)
