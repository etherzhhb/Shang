/*
** Timing - Test timing on adpcm coder and decoder.
**
** The program creates 10Kb garbage, and runs the compressor and
** the decompressor on it.
*/

#include <stdio.h>
#include <math.h>
#include "adpcm.h"

#define DATASIZE 10*1024	/* Data block size */
#define DURATION 10		/* How many seconds to measure */

short pcmdata[DATASIZE];
char adpcmdata[DATASIZE/2];
short pcmdata_2[DATASIZE];

struct adpcm_state coder_1_state, coder_2_state, decoder_state;

main() {
    int i;
    int t0, t1, t2, t3;
    int count = 0, count2;

    for(i=0; i<DATASIZE; i++)
      pcmdata[i] = random() & 0xffff;

    t0 = time(0);
    do {
	adpcm_coder(pcmdata, adpcmdata, DATASIZE, &coder_1_state);
	t1 = time(0);
	count++;
    } while (t1 < t0+DURATION);
    printf("Coder: %d Ksample/second\n", count*DATASIZE/(1000*(t1-t0)));
    printf("  (coded %d blocks of %d samples in %d seconds)\n",
	   count, DATASIZE, t1-t0);
    t2 = time(0);
    count2 = count;
    while ( count2 > 0 ) {
	adpcm_coder(pcmdata, adpcmdata, DATASIZE, &coder_2_state);
	adpcm_decoder(adpcmdata, pcmdata_2, DATASIZE, &decoder_state);
	count2--;
    }
    t3 = time(0);
    printf("Decoder: %d Ksample/second\n",
	   count*DATASIZE/(1000*(t3-t2-t1+t0)));
    printf("  (coded&decoded %d blocks of %d samples in %d seconds)\n",
	   count, DATASIZE, t3-t2);
    exit(0);
}
