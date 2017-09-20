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
	cp $(AGSCFGDIR)/S95agsd $(BUILDROOTTGTDIR)/etc/init.d
	chmod 777 $(AGSCFGDIR)/S50sshd
	cp $(AGSCFGDIR)/S50sshd $(BUILDROOTTGTDIR)/etc/init.d
	cp $(AGSCFGDIR)/sshd_config $(BUILDROOTTGTDIR)/etc/ssh
	cp $(AGSCFGDIR)/usb_fstab $(BUILDROOTTGTDIR)/etc/fstab
	cp $(AGSCFGDIR)/gdbinit $(BUILDROOTFSDIR)/.gdbinit
	cp $(AGSCFGDIR)/skeleton.mk $(BUILDROOTDIR)/package/skeleton/
	cp $(AGSCFGDIR)/ags-busybox-config $(BUILDROOTDIR)/package/busybox
	cp $(AGSCFGDIR)/ags-buildroot-config $(BUILDROOTDIR)/.config
mmc_install:
	cp $(AGSCFGDIR)/autofocus.txt $(BUILDROOTFSDIR)/lv/data/autofocus
	dos2unix $(BUILDROOTFSDIR)/lv/data/autofocus
	cp $(AGSCFGDIR)/visionfocus.txt $(BUILDROOTFSDIR)/lv/data/focusvision
	dos2unix $(BUILDROOTFSDIR)/lv/data/focusvision
	cp $(AGSCFGDIR)/visionparameters.txt $(BUILDROOTFSDIR)/lv/data/vision
	dos2unix $(BUILDROOTFSDIR)/lv/data/vision
	cp $(AGSCFGDIR)/calibration.txt $(BUILDROOTFSDIR)/lv/data/calib
	dos2unix $(BUILDROOTFSDIR)/lv/data/calib
	cp $(AGSCFGDIR)/version.txt $(BUILDROOTFSDIR)/lv/data/version
	dos2unix $(BUILDROOTFSDIR)/lv/data/version
	cp $(AGSCFGDIR)/information.txt $(BUILDROOTFSDIR)/lv/data/info
	dos2unix $(BUILDROOTFSDIR)/lv/data/info
	cp $(AGSCFGDIR)/initialization.txt $(BUILDROOTFSDIR)/lv/data/init
	dos2unix $(BUILDROOTFSDIR)/lv/data/init
	cp $(AGSCFGDIR)/polarizer.txt $(BUILDROOTFSDIR)/lv/data/polarizer
	dos2unix $(BUILDROOTFSDIR)/lv/data/polarizer
	touch $(BUILDROOTFSDIR)/lv/data/hobbs
	chmod +w $(BUILDROOTFSDIR)/lv/data/*
	chmod 777 agsd
	cp agsd $(BUILDROOTFSDIR)/lv/sbin
	chmod 777 $(AGSCFGDIR)/S95agsd
	cp $(AGSCFGDIR)/S95agsd $(BUILDROOTTGTDIR)/etc/init.d
	chmod 777 $(AGSCFGDIR)/S50sshd
	cp $(AGSCFGDIR)/S50sshd $(BUILDROOTTGTDIR)/etc/init.d
	cp $(AGSCFGDIR)/sshd_config $(BUILDROOTTGTDIR)/etc/ssh
	cp $(AGSCFGDIR)/mmc_fstab $(BUILDROOTTGTDIR)/etc/fstab
	cp $(AGSCFGDIR)/gdbinit $(BUILDROOTFSDIR)/.gdbinit
	cp $(AGSCFGDIR)/skeleton.mk $(BUILDROOTDIR)/package/skeleton/
	cp $(AGSCFGDIR)/ags-busybox-config $(BUILDROOTDIR)/package/busybox
	cp $(AGSCFGDIR)/ags-buildroot-config $(BUILDROOTDIR)/.config
burnusb:
	if [ ! -d /mnt/stick/lost+found ] ; then sudo mount /dev/sdb1 /mnt/stick ; fi
	if [ ! -d /mnt/stick/data ] ; then sudo mkdir /mnt/stick/data ; fi
	if [ ! -d /mnt/stick/sbin ] ; then sudo mkdir /mnt/stick/sbin ; fi
	sudo cp $(AGSCFGDIR)/autofocus.txt /mnt/stick/data/autofocus
	sudo unix2dos /mnt/stick/data/autofocus
	sudo cp $(AGSCFGDIR)/visionfocus.txt /mnt/stick/data/focusvision
	sudo unix2dos /mnt/stick/data/focusvision
	sudo cp $(AGSCFGDIR)/visionparameters.txt /mnt/stick/data/vision
	sudo unix2dos /mnt/stick/data/vision
	sudo cp $(AGSCFGDIR)/calibration.txt /mnt/stick/data/calib
	sudo unix2dos /mnt/stick/data/calib
	sudo cp $(AGSCFGDIR)/version.txt /mnt/stick/data/version
	sudo unix2dos /mnt/stick/data/version
	sudo cp $(AGSCFGDIR)/information.txt /mnt/stick/data/info
	sudo unix2dos /mnt/stick/data/info
	sudo cp $(AGSCFGDIR)/initialization.txt /mnt/stick/data/init
	sudo unix2dos /mnt/stick/data/init
	sudo cp $(AGSCFGDIR)/polarizer.txt /mnt/stick/data/polarizer
	sudo unix2dos /mnt/stick/data/polarizer
	sudo echo "\0" > /tmp/zero
	sudo cp /tmp/zero /mnt/stick/data/hobbs
	sudo chmod +w /mnt/stick/data/*
	sudo chmod 777 agsd
	sudo cp agsd /mnt/stick/sbin
	sudo umount /dev/sdb1
	sudo mount /dev/sdb1 /mnt/stick
	sudo mount -o loop,ro $(BUILDROOTDIR)/output/images/rootfs.ext2 $(BUILDROOTDIR)/output/ext2
	sudo cp -avrf $(BUILDROOTDIR)/output/ext2/* /mnt/stick
	sudo cp $(BUILDROOTDIR)/output/images/bzImage /mnt/stick
	sudo cp $(AGSCFGDIR)/extlinux.conf /mnt/stick
	sudo umount /dev/sdb1
	sudo umount $(BUILDROOTDIR)/output/ext2
burnflash:
	sudo umount /dev/sdb1
	sudo mount /dev/sdb1 /mnt/stick
	sudo mount -o loop,ro $(BUILDROOTDIR)/output/images/rootfs.ext2 $(BUILDROOTDIR)/output/ext2
	sudo cp -avrf $(BUILDROOTDIR)/output/ext2/* /mnt/stick
	sudo cp $(BUILDROOTDIR)/output/images/bzImage /mnt/stick
	sudo cp $(AGSCFGDIR)/mmc_extlinux.conf /mnt/stick/extlinux.conf
	sudo chmod -R 777 /mnt/stick/lv/data
	sudo chmod -R 777 /mnt/stick/usr/sbin/agsd
	sudo umount /dev/sdb1
	sudo umount $(BUILDROOTDIR)/output/ext2
