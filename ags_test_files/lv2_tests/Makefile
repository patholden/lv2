BUILDROOTDIR = ../../buildroot
STAGING_DIR = $(BUILDROOTDIR)/output/host/usr/x86_64-buildroot-linux-gnu/sysroot
TOOLDIR = $(BUILDROOTDIR)/output/host/usr/bin/x86_64-buildroot-linux-gnu
TEST_DIR = $(BUILDROOTDIR)/board/agslaser/rootfs_overlay/debug/lv/sbin
INCLUDES = -I ../ -I ../../linux_headers/include -I ../../ags_daemon
CC=$(TOOLDIR)-gcc
LD=$(TOOLDIR)-ld
AS=$(TOOLDIR)-as
AR=$(TOOLDIR)/x86_64-buildroot-linux-uclibc-ar
LDFLAGS=-Wl,--sysroot=$(STAGING_DIR) -Wl,--error-poison-system-directories -L$(STAGING_DIR)/lib -L$(STAGING_DIR)/usr/lib -lc -lm
CFLAGS = -D NEW_TFIND -g -march=atom -Wall -Wmissing-prototypes -Wunused -Werror $(INCLUDES)
TEST_EXEC = tf_test find_sscoords_test box_test coarse_scan_test dark_scan_test sense_one_test sense_loop_test
BOX_TEST_OBJS = lv_box_test.o  \
		../../ags_daemon/Laser_lv2_If.o
SENSE_ONE_TEST_OBJS = lv_sense_one_test.o  \
		../../ags_daemon/Laser_lv2_If.o
SENSE_LOOP_TEST_OBJS = lv_sense_loop_test.o  \
		../../ags_daemon/Laser_lv2_If.o
COARSE_TEST_OBJS = lv_coarse_scan_test.o \
		../../ags_daemon/Laser_lv2_If.o \
		../../ags_daemon/TargetFind.o
DARK_SCAN_TEST_OBJS = lv_coarse_scan_dark_test.o \
		../../ags_daemon/Laser_lv2_If.o \
		../../ags_daemon/TargetFind.o
TFIND_TEST_OBJS = 	lv_tf_test.o \
		../../ags_daemon/Laser_lv2_If.o \
		../../ags_daemon/TargetFind.o
FIND_SSCOORDS_TEST_OBJS = 	lv_find_sscoords_test.o \
		../../ags_daemon/Laser_lv2_If.o \
		../../ags_daemon/TargetFind.o

tf_test: $(TFIND_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(TFIND_TEST_OBJS)
find_sscoords_test: $(FIND_SSCOORDS_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(FIND_SSCOORDS_TEST_OBJS)
box_test: $(BOX_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(BOX_TEST_OBJS)
coarse_scan_test: $(COARSE_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(COARSE_TEST_OBJS)
dark_scan_test: $(DARK_SCAN_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(DARK_SCAN_TEST_OBJS)
sense_one_test: $(SENSE_ONE_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(SENSE_ONE_TEST_OBJS)
sense_loop_test: $(SENSE_LOOP_TEST_OBJS) ../../linux_headers/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(SENSE_LOOP_TEST_OBJS)
all:  $(TEST_EXEC)

%.o: %.c ../../linux_headers/include/linux/laser_api.h
	$(CC) -c -o $@ $(CFLAGS) -I$(INCLUDES) $<
clean:
	rm *.o box_test coarse_scan_test sense_one_test sense_loop_test tf_test find_sscoords_test dark_scan_test
install:
	chmod 777 $(TEST_EXEC);	cp $(TEST_EXEC) $(TEST_DIR)

