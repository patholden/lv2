BUILDROOTDIR = ../../buildroot
STAGING_DIR = $(BUILDROOTDIR)/output/host/usr/x86_64-buildroot-linux-gnu/sysroot
TOOLDIR = $(BUILDROOTDIR)/output/host/usr/bin/x86_64-buildroot-linux-gnu
TEST_DIR = $(BUILDROOTDIR)/board/agslaser/rootfs_overlay/lv/sbin
INCLUDES = -I ../ -I ../../linux_headers/include -I ../../ags_daemon
CC=$(TOOLDIR)-gcc
LD=$(TOOLDIR)-ld
AS=$(TOOLDIR)-as
AR=$(TOOLDIR)/x86_64-buildroot-linux-uclibc-ar
LDFLAGS=-Wl,--sysroot=$(STAGING_DIR) -Wl,--error-poison-system-directories -L$(STAGING_DIR)/lib -L$(STAGING_DIR)/usr/lib -lc -lm
CFLAGS = -g -march=atom -Wall -Wmissing-prototypes -Wunused -Werror $(INCLUDES)
BOX_TEST_OBJS = lv_box_test.o  \
		../../ags_daemon/Laser_lv2_If.o
SENSE_ONE_TEST_OBJS = lv_sense_one_test.o  \
		../../ags_daemon/Laser_lv2_If.o
SENSE_LOOP_TEST_OBJS = lv_sense_loop_test.o  \
		../../ags_daemon/Laser_lv2_If.o
COARSE_TEST_OBJS = 	lv_coarse_scan_test.o \
		../../ags_daemon/Laser_lv2_If.o \
		../../ags_daemon/TargetFind.o
TFIND_TEST_OBJS = 	lv_tf_test.o \
		../../ags_daemon/Laser_lv2_If.o \
		../../ags_daemon/TargetFind.o

tf_test: $(TFIND_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(TFIND_TEST_OBJS)
box_test: $(BOX_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(BOX_TEST_OBJS)
coarse_scan_test: $(COARSE_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(COARSE_TEST_OBJS)
sense_one_test: $(SENSE_ONE_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(SENSE_ONE_TEST_OBJS)
sense_loop_test: $(SENSE_LOOP_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(SENSE_LOOP_TEST_OBJS)
all:  tf_test box_test coarse_scan_test sense_one_test sense_loop_test

%.o: %.c ../../linux_headers/include/linux/laser_api.h
	$(CC) -c -o $@ $(CFLAGS) -I$(INCLUDES) $<
clean:
	rm *.o box_test coarse_scan_test sense_one_test sense_loop_test tf_test
install:
	chmod 755 tf_test box_test coarse_scan_test sense_one_test sense_loop_test
	cp tf_test $(TEST_DIR)
	cp box_test $(TEST_DIR)
	cp coarse_scan_test $(TEST_DIR)
	cp sense_one_test $(TEST_DIR)
	cp sense_loop_test $(TEST_DIR)
burnusb:
	sudo umount /dev/sdb1
	sudo mount /dev/sdb1 /mnt/stick
	sudo mount -o loop,ro $(BUILDROOTDIR)/output/images/rootfs.ext2 $(BUILDROOTDIR)/output/ext2
	sudo cp -avrf $(BUILDROOTDIR)/output/ext2/* /mnt/stick
	sudo cp $(BUILDROOTDIR)/output/images/bzImage /mnt/stick
	sudo cp $(BUILDROOTDIR)/output/images/bzImage /mnt/stick/boot
	sudo cp $(HOME)/ags/ags-demo/rev1.0/extlinux.conf /mnt/stick
	sudo umount /dev/sdb1
	sudo umount $(BUILDROOTDIR)/output/ext2

