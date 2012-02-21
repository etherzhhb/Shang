# Makefile for the adpcm project float version 
CC = CC -I/usr/local/include -g

flt_adpcm.exe: flt_adpcm.c
	$(CC) -o flt_adpcm.exe flt_adpcm.c

flt_adpcm: flt_adpcm.c
#create asm files
	g56k -alo -S -D__DSP56K__ -D__SIMULATION__ -o flt_adpcm.asm flt_adpcm.c
	g56k -alo -S -D__DSP56K__ -o flt_adpcm_1.asm flt_adpcm.c
#create cld files and simulate
	g56k -alo -D__DSP56K__ -D__SIMULATION__ -o flt_adpcm.cld flt_adpcm.c
	g56k -alo -D__DSP56K__ -o flt_adpcm_1.cld flt_adpcm.c
	run56sim -t flt_adpcm.cld
	run56sim -t flt_adpcm_1.cld

