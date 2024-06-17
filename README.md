# TAS5805m

This driver has a minimal config that works with the TAS5805m, the TAS5825m and the TAS5828m, probably also with the other members of this family of audio amplifiers with DSP. For full configuration 3 different binary firmwares for corresponding DSP rates have to be created from TI's PPC3 output.
A Device Tree overlay for the Raspberry Pi with i2creg=<> and firmware="" is provided.
A working overlay file for the Armbian allwinner sun8i-h3 is also provided.
Compile ppc2bin.c command "gcc -o ./ppc2bin ppc2bin.c" and use to convert ppc3_output.h to ppc3_output.bin, store in /lib/firmware, ready to use with the tas5805m driver.
