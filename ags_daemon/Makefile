#
#        -Wall -W -Wbad-function-cast -Wcast-qual \
#	-Wstrict-prototypes \
#	-Wmissing-prototypes -Wmissing-declarations -Wredundant-decls \
#
#       $Id: Makefile,v 1.30 2000/09/28 19:57:01 ags-sw Exp ags-sw $
#
BUILDROOTDIR = ../buildroot
STAGING_DIR = $(BUILDROOTDIR)/output/host/usr/x86_64-buildroot-linux-gnu/sysroot
TOOLDIR = $(BUILDROOTDIR)/output/host/usr/bin/x86_64-buildroot-linux-gnu
BUILDROOTFSDIR = $(BUILDROOTDIR)/board/agslaser/rootfs_overlay
BUILDROOTTGTDIR = $(BUILDROOTDIR)/output/target
AGSCFGDIR = ../ags-config-files
LNXHDRDIR = ../linux_headers
CC=$(TOOLDIR)-gcc
LD=$(TOOLDIR)-ld
AS=$(TOOLDIR)-as
AR=$(TOOLDIR)/x86_64-buildroot-linux-uclibc-ar

# COMPILE AND LINK OPTIONS AND DEPENDENCIES
LDFLAGS=-Wl,--sysroot=$(STAGING_DIR) -Wl,--error-poison-system-directories -L$(STAGING_DIR)/lib -L$(STAGING_DIR)/usr/lib -lc -lm -lpthread
### EXTRA_CFLAGS = -DAGS_DEBUG -DZDEBUG
EXTRA_CFLAGS = 
USERINCLUDE    := \
	-I./ \
	-I$(LNXHDRDIR)/include
### CFLAGS = -g -DLASER_DEFINED -march=atom  -Wall -Wmissing-prototypes -Wstrict-prototypes -Wunused -Werror $(USERINCLUDE)
#CFLAGS = -D NEW_TFIND -DLASER_DEFINED -march=atom  -Wall -Wmissing-prototypes -Wstrict-prototypes -Wunused -Werror $(USERINCLUDE)
CFLAGS = -D NEW_TFIND -DLASER_DEFINED -march=atom  -Wall -Wmissing-prototypes -Wstrict-prototypes -Wunused -Werror $(USERINCLUDE)
AGS_OBJECTS = Main.o \
	      L3DTransform.o \
	      AngleCorrections.o \
              APTParser.o \
              CRCHandler.o  \
              comm_loop.o  \
              parse_data.o  \
	      DoAutoFocusCmd.o \
              LaserCmds.o \
	      LaserPattern.o \
	      LaserInterface.o \
              BoardComm.o Events.o \
              SensorRegistration.o \
              3DTransform.o \
              QuickCheckManager.o \
	      SystemSpecifics.o Video.o Web.o \
	      Init.o Net.o FOM.o \
	      angles.o chisqr.o amoeba.o \
              ShowTargets.o \
	      Hobbs.o \
	      SetBit.o \
	      RightOnFullReg.o RightOnCert.o DoFindOneTarget.o \
	      FullRegWithFeedback.o \
	      ParseAutoFocus.o \
	      ParseVisionFocus.o \
	      LaserFlex.o \
	      FlexCalculateTransform.o \
	      FlexCalWithFeedback.o  \
	      FlexRegWithFeedback.o \
	      DoTakePicture.o \
	      RefreshRate.o \
	      ChangeDisplayPeriod.o \
	      ChangeTransformTolerance.o \
	      GetTargetsUsed.o \
	      DoCoarseScan.o \
	      DoCoarseScan2.o \
	      CalibXY.o \
	      segtest.o segpoly.o pnpoly.o tan_init.o \
	      area_order.o heap.o allace4.o shoelace4.o \
	      CalculateTransform.o \
	      ParseVision.o \
	      RemoteSerial.o \
	      L2VtakePicture.o \
	      readnewcal.o \
	      polyfunc.o \
	      FracCert.o  DoubleSensorSearch.o \
	      SensorSearch.o  TargetFind.o Laser_lv2_If.o Files.o
#
#
agsd : compile.h asciiID.h $(AGS_OBJECTS) BoardComm.h $(LNXHDRDIR)/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(AGS_OBJECTS)
	mv -f compile.h old.compile.h.old


%.o: %.c BoardComm.h parse_data.h $(LNXHDRDIR)/include/linux/laser_api.h
	$(CC) -c -o $@ $(CFLAGS) $(EXTRA_CFLAGS) $<


compile.h:
	@echo \#define AGS_COMPILE_TIME \"`date`\" >> .ver
	@echo \#define AGS_UNIX_TIME \"`date +%s`\" >> .ver
	@echo \#define AGS_YEAR \"`date +%Y`\" >> .ver
	@echo \#define AGS_COMPILE_BY \"`whoami`\" >> .ver
	@echo \#define AGS_COMPILE_HOST \"assemblyguide.com\" >> .ver
	@mv -f .ver $@

clean:
	rm -f *.o agsd compile.h *~
usb_install:
	cp $(AGSCFGDIR)/S95agsd.USBSTICK $(BUILDROOTFSDIR)/etc/init.d/S95agsd
	cp agsd $(BUILDROOTFSDIR)/usr/sbin
	chmod 777 $(BUILDROOTFSDIR)/etc/init.d/S95agsd
	chmod 777 $(BUILDROOTFSDIR)/usr/sbin/agsd
# NOTE:  After running "make usb_install", need to re-run buildroot "make".
#        This will rebuild the output/images/rootfs.ext2 file-system package
#        used to burn to Projector USB stick.
burnusb:
	if [ ! -d /mnt/stick/lost+found ] ; then sudo mount /dev/sdb1 /mnt/stick ; fi
	sudo umount /dev/sdb1
	sudo mount /dev/sdb1 /mnt/stick
	sudo mount -o loop,ro $(BUILDROOTDIR)/output/images/rootfs.ext2 $(BUILDROOTDIR)/output/ext2
	sudo cp -avrf $(BUILDROOTDIR)/output/ext2/* /mnt/stick
	sudo cp $(BUILDROOTDIR)/output/images/bzImage /mnt/stick
	sudo cp -L $(AGSCFGDIR)/extlinux.conf /mnt/stick
	sudo echo "\0" > /tmp/zero
	sudo cp /tmp/zero /mnt/stick/laser/data/hobbs
	sudo umount /dev/sdb1
	sudo umount $(BUILDROOTDIR)/output/ext2
# FIXME---PMA---ADD REFERENCE TO UPDATE SCRIPT & ANY OTHER STUFF FOR BUILDING & BURNING
#               TO ONBOARD FLASH
#mmc_install:
#burnflash:
