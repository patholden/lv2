#include <stdint.h>
static char rcsid[] = "$Id: AutoCertLinux.c,v 1.7 1999/10/22 13:25:50 ags-sw Exp $";

#include <stdio.h>
#include <string.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "AppResponses.h"
#include "3DTransform.h"
#include "AngleCorrections.h"
#include "SensorSearch.h"
#include "AutoCert.h"
#include "Protocol.h"

int32_t			gProjectorSerialNumber = -1L;
double		gTolerance = 0.0300;

