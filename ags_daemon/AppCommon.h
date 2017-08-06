#ifndef APPCOMMN_H
#define APPCOMMN_H

/*   $Id: AppCommon.h,v 1.6 2000/09/28 19:04:27 ags-sw Exp ags-sw $  */
#ifdef __unix__

#define GENERATINGPOWERPC 1
#define pascal 
#endif

#define STRICT_WINDOWS 1
#define STRICT_MENUS 1
#define STRICT_CONTROLS 0
#define STRICT_LISTS 0
#define GARGTOL_DEFAULT 8.0E-8
#define GQCCOUNT_DEFAULT -1
#define GBEAMLNRNG_DEF 100.0
#define GBAUD_DEFAULT 115200

enum {
        false,
        true
};

extern uint32_t TickCount ( void );

#define SystemSevenOrLater 1

/*
	#include <ConditionalMacros.h>
	#include <Traps.h>
	#include <Types.h>
	#include <IntlResources.h>
	#include <MixedMode.h>
	#include <QuickdrawText.h>
	#include <Quickdraw.h>
	#include <Memory.h>
	#include <Menus.h>
	#include <Controls.h>
	#include <OSUtils.h>
	#include <Events.h>
	#include <Windows.h>
	#include <TextEdit.h>
	#include <Errors.h>
	#include <Dialogs.h>
	#include <SegLoad.h>
	#include <Files.h>
	#include <Fonts.h>
	#include <Gestalt.h>
	#include <Resources.h>
	#include <StandardFile.h>
	#include <Strings.h>
	#include <Script.h>
	#include <TextUtils.h>
	#include <Timer.h>
	#include <FixMath.h>
	#include <Icons.h>
	#include <ToolUtils.h>
	#include <Devices.h>
	#include <Components.h>
	#include <Sound.h>
	#include <AppleTalk.h>
	#include <PPCToolBox.h>
	#include <Processes.h>
	#include <EPPC.h>
	#include <Notification.h>
	#include <AppleEvents.h>
	#include <QDOffscreen.h>
*/

#include "Protocol.h"

extern	unsigned char		gDoneFlag;
extern	unsigned char		gLocalMode;
extern	unsigned char		gAutoCert;
extern	unsigned char		gInspectionMode;
extern	unsigned char		gAEMode;

extern	uint32_t		gThisAppSignature;
extern	char		gThisAppName[32];

extern  int  gSearchCurrentSensor;

extern  int32_t * shmReadErr;

extern int gForceTransform;

extern double gMaxDiffMag;
#endif // APPCOMMN_H
