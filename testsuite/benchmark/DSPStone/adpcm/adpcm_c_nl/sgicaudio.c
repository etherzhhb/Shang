/*
** sgicaudio - Simple AIFF file compressor.
**
** Needs libaf, which is in the Digital Media developers option.
*/

#include <stdio.h>
#include <audio.h>
#include <audiofile.h>
#include "adpcm.h"

#define NFRAMES 8000

struct adpcm_state astate;
short ibuf[NFRAMES];
char obuf[NFRAMES/2];

main(argc, argv)
    int argc;
    char **argv;
{
    AFfilehandle af;
    FILE *of;
    long fmt, width;
    long count;
    
    if ( argc < 2 || argc > 3 ) {
	fprintf(stderr, "Usage: %s aifffile [adpcmfile]\n", argv[0]);
	exit(1);
    }
    /*
    ** Open input file and check format
    */
    if ( (af = AFopenfile(argv[1], "r", NULL)) == 0 ) {
	perror(argv[1]);
	exit(1);
    }
    AFgetsampfmt(af, AF_DEFAULT_TRACK, &fmt, &width);
    if ( AFgetchannels(af, AF_DEFAULT_TRACK) != 1 || fmt != AF_SAMPFMT_TWOSCOMP
	|| width != 16 ) {
	fprintf(stderr, "%s: %s: only 16bit mono integer files supported\n",
		argv[0], argv[1]);
	exit(1);
    }
    if ( AFgetrate(af, AF_DEFAULT_TRACK) != 8000.0 )
      fprintf(stderr, "%s: %s: WARNING: not 8Khz\n", argv[0], argv[1]);
    /*
    ** Open output file
    */
    if ( argc == 3 ) {
	if ( (of=fopen(argv[2], "w")) == 0 ) {
	    perror(argv[2]);
	    exit(1);
	}
    } else {
	of = stdout;
    }
    /*
    ** Copy loop
    */
    while (1) {
	count = AFreadframes(af, AF_DEFAULT_TRACK, ibuf, NFRAMES);
	if ( count <= 0 ) break;
	if ( count & 1 ) {
	    ibuf[count] = ibuf[count-1];
	    count++;
	}
	adpcm_coder(ibuf, obuf, count, &astate);
	fwrite(obuf, 1, (count+1)/2, of);
    }
    fclose(of);
    AFclosefile(af);
    exit(0);
}
