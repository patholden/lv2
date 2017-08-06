# lv2---this repository contains new files for LaserVision changes made for new TargetFind operation.
#       There is a new Linux driver file so both driver files are now contained in "drivers/misc/lv2"
#       directory.  New commands were added to the CMDW struct for new driver.
#       A new ags_daemon file for the new TargetFind operation was added called TargetFind.c with an
#           accompanying header file called TargetFind.h.
#       A new test directory was added for specific tests to run components of the TargetFind operation
#       such as exercising the new driver functions that only write to 1 point along one axis (i.e. move
#       X along Y-axis, move Y along X-axis).  "ags_test_files/lv2_tests" is name of directory.
#       There are tests that exercise the new write-sense driver function.
#       All tests have syntax:
#                          "<test> <inputX> <inputY> <step> <numPoints> <sense-delay(us)>".
#       >>>"sensor_one_test" performs a loop in user space to calculate the time it takes to run one
#            write-sense operation including latency of the write command from user space.
#       >>>"box_test" performs an X-axis line test, then a Y-axis line test, then a sensor test where the
#           the write-sense operation in a single line along either X or Y axis is done in the driver.
#           TBD---make-box test that will expand X/Y line tests to form a box and then just keep expanding
#                 X/Y lines.  This will allow some timing analysis so we can utilize best algorithm for
#                 actual coarse-scan operation.
#       >>>"coarse_scan_test" exercises the actual CoarseScan step of the new TargetFind operation. It
#           currently exercises the Coarse Scan operation that is controlled at the user level and calls
#           the single write-sense operation function in the kernel driver.  This test is used to debug
#           the Coarse-Scan step of TargetFind so that LaserGuide is not needed.
#       TargetFind.c only has coarse-scan operation coded because we are still in phase of determining timing for
#       basic CoarseScan operation.
#
#       Build instructions are in the README_developers file in DOCUMENTATION directory.
#       Go to test-files directory, run "make clean; make all; make install"
#       Go to buildroot directory, run "make linux-rebuild; make"
#       Go to ags_daemon directory, run "make burnusb".  We're not using daemon yet, so no need to build it at
#       this point.
