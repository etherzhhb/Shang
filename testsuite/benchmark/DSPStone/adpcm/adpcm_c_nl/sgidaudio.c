/*
** sgidaudio - Simple AIFF file decompressor.
**
** Needs libaf, which is in the Digital Media developers option.
*/

#include <stdio.h>
#include <audio.h>
#include <audiofile.h>
#include "adpcm.h"

#define NFRAMES 8000

struct adpcm_state astate;
char ibuf[NFRAMES/2];
short obuf[NFRAMES];

main(argc, argv)
    int argc;
    char **argv;
{
    AFfilehandle af;
    AFfilesetup afsetup;
    FILE *of;
    long fmt, width;
    long count;
    int outindex;
    
    if ( argc < 2 || argc > 3 ) {
	fprintf(stderr, "Usage: %s [adpcmfile] aifffile\n", argv[0]);
	exit(1);
    }
    /*
    ** Open input file
    */
    if ( argc == 3 ) {
	if ( (of = fopen(argv[1], "r")) == NULL ) {
	    perror(argv[1]);
	    exit(1);
	}
	outindex = 2;
    } else {
	of = stdin;
	outindex = 1;
    }
    /*
    ** Open output file
    */
    afsetup = AFnewfilesetup();
    AFinitchannels(afsetup, AF_DEFAULT_TRACK, 1);
    AFinitrate(afsetup, AF_DEFAULT_TRACK, 8000.0);
    AFinitsampfmt(afsetup, AF_DEFAULT_TRACK, AF_SAMPFMT_TWOSCOMP, 16);
    if ( (af = AFopenfile(argv[outindex], "w", afsetup)) == 0 ) {
	perror(argv[outindex]);
	exit(1);
    }
    /*
    ** Copy loop
    */
    while (1) {
	count = fread(ibuf, 1, NFRAMES/2, of);
	if ( count <= 0 ) break;
	count = count*2;
	adpcm_decoder(ibuf, obuf, count, &astate);
	AFwriteframes(af, AF_DEFAULT_TRACK, obuf, count);
    }
    fclose(of);
    AFclosefile(af);
    exit(0);
}
