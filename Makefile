obj-m += tas5805m.o

all:
	make -j$(nproc) ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C ~/rpi-linux_6.1.77/ M=`pwd` modules
	#make -j$(nproc) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C ~/rpi-linux_6.1.77/ M=`pwd` modules
	#make -C ~/lib/modules/`uname -r`/build M=`pwd` modules
clean:
	make -j$(nproc) ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C ~/rpi-linux_6.1.77/ M=`pwd` clean
	#make -j$(nproc) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C ~/rpi-linux_6.1.77/ M=`pwd` clean
