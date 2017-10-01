BUILDROOTDIR = $(HOME)/buildroot
STAGING_DIR = $(BUILDROOTDIR)/output/host/usr/x86_64-buildroot-linux-gnu/sysroot
TOOLDIR = $(BUILDROOTDIR)/output/host/usr/bin/x86_64-buildroot-linux-gnu
TEST_DIR = $(BUILDROOTDIR)/board/agslaser/rootfs_overlay/debug/sbin
INCLUDES = -I ./ -I ../linux_headers/include
CC=$(TOOLDIR)-gcc
LD=$(TOOLDIR)-ld
AS=$(TOOLDIR)-as
AR=$(TOOLDIR)/x86_64-buildroot-linux-uclibc-ar
LDFLAGS=-Wl,--sysroot=$(STAGING_DIR) -Wl,--error-poison-system-directories -L$(STAGING_DIR)/lib -L$(STAGING_DIR)/usr/lib -lc -lm
CFLAGS = -g -march=atom -D NEW_TFIND
TEST_EXEC = ramp ramp1 ramp2 ramp9 tgfind_in step xy xytest xy_io ttyloop1 ttyloop2 sinusoid1 lg_drv_test lg_drv_sine readio writeio tgfind2 tgfind_timing
TEST_OBJS = ramp.o ramp2.o ramp9.o tgfind_in.o step.o xy.o xytest.o xy_io.o ttyloop1.o ttyloop2.o sinusoid.o sinusoid1.o lg_drv_test.o lg_drv_sine.o readio writeio tgfind2.o tgfind_timing.o

default:  $(TEST_EXEC)
all:  $(TEST_EXEC)
readio.o: readio.c
	$(CC) -c $(CFLAGS) $(INCLUDES) readio.c
readio: readio.o
	$(CC) $(LDFLAGS) -o readio readio.o $(LDFLAGS)
writeio.o: writeio.c
	$(CC) -c $(CFLAGS) $(INCLUDES) writeio.c
writeio: writeio.o
	$(CC) $(LDFLAGS) -o writeio writeio.o $(LDFLAGS)
lg_drv_sine.o: lg_drv_sine.c
	$(CC) -c $(CFLAGS) $(INCLUDES) lg_drv_sine.c
lg_drv_sine: lg_drv_sine.o
	$(CC) $(LDFLAGS) -o lg_drv_sine lg_drv_sine.o $(LDFLAGS)
lg_drv_test.o: lg_drv_test.c
	$(CC) -c $(CFLAGS) $(INCLUDES) lg_drv_test.c
lg_drv_test: lg_drv_test.o
	$(CC) $(LDFLAGS) -o lg_drv_test lg_drv_test.o $(LDFLAGS)
ramp: ramp.o
	$(CC) $(LDFLAGS) -o ramp ramp.o $(LDFLAGS)
ramp.o: ramp.c
	$(CC) -c $(CFLAGS) $(INCLUDES) ramp.c
ramp1: ramp1.o
	$(CC) $(LDFLAGS) -o ramp1 ramp1.o $(LDFLAGS)
ramp1.o: ramp1.c
	$(CC) -c $(CFLAGS) $(INCLUDES) ramp1.c
ramp2: ramp2.o
	$(CC) $(LDFLAGS) -o ramp2 ramp2.o $(LDFLAGS)
ramp2.o: ramp2.c
	$(CC) -c $(CFLAGS) $(INCLUDES) ramp2.c
ramp9: ramp9.o
	$(CC) $(LDFLAGS) -o ramp9 ramp9.o $(LDFLAGS)
ramp9.o: ramp9.c
	$(CC) -c $(CFLAGS) $(INCLUDES) ramp9.c
sinusoid1: sinusoid1.o
	$(CC) $(LDFLAGS) -o sinusoid1 sinusoid1.o $(LDFLAGS)
sinusoid1.o: sinusoid1.c
	$(CC) -c $(CFLAGS) $(INCLUDES) sinusoid1.c
step: step.o
	$(CC) $(LDFLAGS) -o step step.o $(LDFLAGS)
step.o: step.c
	$(CC) -c $(CFLAGS) $(INCLUDES) step.c
ttyloop1: ttyloop1.o
	$(CC) $(LDFLAGS) -o ttyloop1 ttyloop1.o $(LDFLAGS)
ttyloop1.o: ttyloop1.c
	$(CC) -c $(CFLAGS) $(INCLUDES) ttyloop1.c
ttyloop2: ttyloop2.o
	$(CC) $(LDFLAGS) -o ttyloop2 ttyloop2.o $(LDFLAGS)
ttyloop2.o: ttyloop2.c
	$(CC) -c $(CFLAGS) $(INCLUDES) ttyloop2.c
xy: xy.o
	$(CC) $(LDFLAGS) -o xy xy.o $(LDFLAGS)
xy.o: xy.c
	$(CC) -c $(CFLAGS) $(INCLUDES) xy.c
xy_io: xy_io.o
	$(CC) $(LDFLAGS) -o xy_io xy_io.o $(LDFLAGS)
xy_io.o: xy_io.c
	$(CC) -c $(CFLAGS) $(INCLUDES) xy_io.c
xytest: xytest.o
	$(CC) $(LDFLAGS) -o xytest xytest.o $(LDFLAGS)
xytest.o: xytest.c
	$(CC) -c $(CFLAGS) $(INCLUDES) xytest.c
tgfind2.o: tgfind2.c
	$(CC) -c $(CFLAGS) $(INCLUDES) tgfind2.c
tgfind2: tgfind2.o
	$(CC) $(LDFLAGS) -o tgfind2 tgfind2.o $(LDFLAGS)
tgfind_in.o: tgfind_in.c
	$(CC) -c $(CFLAGS) $(INCLUDES) tgfind_in.c
tgfind_in: tgfind_in.o
	$(CC) $(LDFLAGS) -o tgfind_in tgfind_in.o $(LDFLAGS)
tgfind_timing: tgfind_timing.o
	$(CC) $(LDFLAGS) -o tgfind_timing tgfind_timing.o $(LDFLAGS)
tgfind_timing.o: tgfind_timing.c
	$(CC) -c $(CFLAGS) $(INCLUDES) tgfind_timing.c
xycenter: xycenter.o
	$(CC) $(LDFLAGS) -o xycenter xycenter.o $(LDFLAGS)
xycenter.o: xycenter.c
	$(CC) -c $(CFLAGS) -I$(INCLUDES) xycenter.c
clean:
	rm *.o $(TEST_EXEC)
install:
	chmod 777 $(TEST_EXEC); cp $(TEST_EXEC) $(TEST_DIR)
