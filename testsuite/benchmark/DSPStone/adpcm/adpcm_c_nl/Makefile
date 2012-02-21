#
# Makefile for adpcm and test programs
# -------------------
CFLAGS=-O
# May need this on a sun:
# CFLAGS=-O -I/usr/demo/SOUND

ALLSRC=README adpcm.2 Makefile adpcm.h libst.h adpcm.c timing.c rawcaudio.c \
	rawdaudio.c suncaudio.c sgicaudio.c sgidaudio.c hello.adpcm.uu

all: adpcm.o rawcaudio rawdaudio timing

sgi: all sgicaudio sgidaudio

sun: all suncaudio sundaudio

adpcm.o: adpcm.h

suncaudio: suncaudio.o adpcm.o
	$(CC) adpcm.o suncaudio.o -o suncaudio -L/usr/demo/SOUND -laudio

sundaudio: suncaudio
	cp suncaudio sundaudio

sgicaudio: sgicaudio.o adpcm.o
	$(CC) sgicaudio.o adpcm.o -o sgicaudio -laudiofile -lm

sgidaudio: sgidaudio.o adpcm.o
	$(CC) sgidaudio.o adpcm.o -o sgidaudio -laudiofile -lm

rawcaudio: rawcaudio.o adpcm.o
	cc rawcaudio.o adpcm.o -o rawcaudio

rawdaudio: rawdaudio.o adpcm.o
	cc rawdaudio.o adpcm.o -o rawdaudio

timing: timing.o adpcm.o
	cc timing.o adpcm.o -o timing

adpcm.shar: $(ALLSRC)
	shar $(ALLSRC) > adpcm.shar
