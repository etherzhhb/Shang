%.exe : %.s portab.h g721.h g721.c adsp2101.ach Makefile
	g21 $*.s -map -a adsp2101.ach -mstatic-spill\
	-runhdr ${ADI_DSP}/21xx/lib/2101_hdr.obj -o $*.exe
	xsim2101 -w adsp2101.win -e $*.exe -a adsp2101&

%.s : %.c
	g21 $*.c -Wall -S -D__ADSP2101__ -a adsp2101.ach

%.ach : %.sys
	bld21 $*

clean:
	rm -f *.cde *.i *.int *.is *.obj *.s *.exe

