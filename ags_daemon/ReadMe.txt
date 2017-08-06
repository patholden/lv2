================================================================
=                                                              =
=                      GIT-some stuff                          =
=                                                              =
================================================================
Setting up AGS Multiuser Git repository (NOTE:  you'll need an account):
git init
git remote add master https://github.com/AlignedVision/lv2
git config --global push.default
git remote -v  (Returns fetch & push URL)
git remote show (returns master)
git pull https://github.com/AlignedVision/lv2 master
GIT Making changes (applies to any git repository):
git commit -a (makes a local copy)
git push
GIT Syncing with upstream master(applies to any git repository):
git checkout master
git pull <URL> master
(fix any merge conflicts)
git commit -a (commits to local repository)
git push origin master
GIT adding files:
git add <filename>
git commit -a  (popup will have commit description you need to fill in and save)
git push
GIT recovering deleted file:
(Delete file)
git pull <URL> master
================================================================
=                                                              =
=                      BUILD-some stuff                        =
=                                                              =
================================================================
BUILD Environment changes:
Go to your home directory.  Create $(HOME)/newbuildroot directory, cd to it, download buildroot from https://buildroot.org/downloads/ and get last tar.gz from November, 2015.  gunzip & extract file (in your buildroot directory). Rename (mv) this directory $(HOME)/buildroot.
Go back to home directory.
follow GIT instructions for https://github.com/AlignedVision/lv2.
When you pull the master branch, you'll end up with the following directories:
$(HOME)/ags_daemon
$(HOME)/ags-config-files
$(HOME)/linux-headers
$(HOME)/ags_test_files
$(HOME)/buildroot
$(HOME)/syslinux-4.04

Go to buildroot directory and do following:
cd $(HOME)/buildroot
cp $(HOME)/ags-config-files/ags-buildroot-config $(HOME)/buildroot/.config
cd $(HOME)/buildroot/board
mkdir agslaser
cd agslaser
mkdir etc
mkdir debug
mkdir lv2
cd etc; mkdir ags; cd ags; mkdir conf
cd ../../../lv2
mkdir sbin
make menu-config (NOTE:  Make sure System configuration "rootoverlay" directory is set to $(HOME)/buildroot/board/agslaser.  Make sure kernel configuration Local directory is set to your linux directory.  Change kernel configuration kernel config to your linux directory/.config).  Exit, say yes to save-config question.
make  (NOTE:  this takes a few hours).
Now prepare to create executables for the AV file-system
cd $(HOME)/ags_daemon  (NOTE:  Open the Makefile & make sure BUILDROOTDIR is set to your buildroot directory, it should be already done for you).
cd $(HOME)/ags_linux
make headers_install arch=x86_64 INSTALL_HDR_PATH=../linux_headers
cp arch/x86/configs/agslaser_config .config
cd $(HOME)/ags_daemon;make clean; make all;make install
cd $(HOME)/ags_test_files;make clean; make all;make install
cd $(HOME)/ags_test_files/lv2_tests;make clean; make all;make install
cd $(HOME)/buildroot;make linux-rebuild;make;

If you made it this far, you're ready to burn a USB stick.
================================================================
=                                                              =
=                    FORMAT USB STICK                          =
=                                                              =
================================================================
Make a bootable USB stick:
     Need USB memory stick with at least 16GB (I use SanDisk Cruzer 16GB).
     Figure out what linux assigns as /dev for USB stick.  Mine is /dev/sdb1.
     Make sure it's not currently mounted.
     Enter ¨sudo fdisk /dev/sdb¨.  Enter following at prompts:
         o<CR>
    	 n<CR>
    	 p<CR>
    	 1<CR>
	 (default first cylinder)<CR>
   	 (default last cylinder)<CR>
	 t<CR>
	 83<CR>
	 a<CR>  (Note:  here you may need to enter 1 if boot flag isn´t set)
    	 w<CR>
    	 quit
Now run following commands as su:
sudo su
sudo mkfs -t ext2 /dev/sdb1
(wait)
sudo tune2fs -L <labelname> /dev/sdb1 (this is optional but I use it)
sudo mkdir /mnt/usb
sudo mount /dev/sdb1 /mnt/usb
cd $(HOME)/syslinux-4.04
cd mbr
sudo su
sudo cat mbr.bin >/dev/sdb  (NOTE:  this puts MBR in correct place at beginning of USB)
cd ../extlinux
./extlinux --install /mnt/usb
cd ~/ags
cp $(HOME)/ags-config-files/extlinux.conf /mnt/usb
exit   (Get out of su mode)

================================================================
=                                                              =
=                      BURN-some stuff                         =
=                                                              =
================================================================
cd $(HOME)/ags_daemon;make burnusb
NOTE:  MAKE SURE TO USE EJECT to remove USB stick once programmed.
To run AGS operational code on system:
With power off, insert USB stick into top USB slot to right of HDMI connector.
Turn power on, you'll see Linux kernel go through it's initialization,
At "ags-lgs login:" prompt, enter "root".
At "Password:" prompt, enter "temp1234".

To run tests:  cd /lv2/sbin.  You'll see all the tests available.
The daemon starts automatically, you can use strace to debug it or use logs.
Syslogs are located in /var/log/user.  (NOTE:  There are a lot of logs, use grep on a specific buzz word you put into your syslog calls in the daemon code).
dmesg can be used to see output from debug printk() calls in kernel driver.

