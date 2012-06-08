#include <stdio.h>
#include <strings.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <multimedia/ulaw2linear.h>
#include <multimedia/libaudio.h>
#include "adpcm.h"

#ifdef __STDC__
#define A(x) x
#else
#define A(x) ()
#endif

extern char *strdup A((char *));
extern char *strstr A((char *, char *));


main(argc, argv)
    int argc;
    char *argv[];
{

int file,outfile;		/* input file */
Audio_hdr audiohdr;		/* Audio File header */
int cnt;			/* how much read ? */
register char *ulaw;		/* pointer to ulaw output from pcm conv */
register short *pcm;		/* pointer to pcm output from ulaw conv */
char ulawbuf[BUFSIZ];		/* to hold ulaw data */
short pcmbuf[BUFSIZ];		/* to hold pcm data */
int decompress=0;		/* just a flag */
struct stat statbuf;
char *newfn;			/* output file name */
char *p;			/* temporary character pointer */
struct adpcm_state state;
int pos;


/* this program can be called with two different names, if it
   is called as caudio then we want to compress the file and if
   it is called with uaudio then we want to uncompress it */

        pos = strlen(argv[0]) - strlen("caudio");
        if (strcmp(argv[0]+pos,"daudio") == 0) 
		decompress=1;
        else if ( strcmp(argv[0]+pos,"caudio") != 0) {
	    puts("Progname should end in 'caudio' or 'daudio'");
	    exit(1);
	}

	if (argc < 2) {
		puts("No input file(s) specified");
		exit(1);
	}

/* Now we pass through the entire list of input files and compress
   each file found */

	while (argc-- > 1) {
		if ((file = open(*++argv,O_RDONLY)) == NULL) {
			perror("couldn't open input file");
			exit(1);
		}
		if (!decompress) {
			if (! audio_isaudiofile(*argv)) {
				perror("not an audio file");
				exit(2);
			}
			if (audio_read_filehdr(file, &audiohdr, 
					(char *)NULL, 0) != AUDIO_SUCCESS) {
				perror("bad header");
				exit(3);
			}
			newfn=(char *) malloc(strlen(*argv)+5);
			sprintf(newfn,"%s.pcm",*argv);

			if ((outfile = open(newfn, O_RDWR |O_CREAT | O_EXCL,
					    0644)) < 0) {
				printf("%s already exists\n",newfn);
				free(newfn);
				continue;
			}
			while ((cnt = read(file, (char *) ulawbuf,
					   sizeof ulawbuf)) > 0) {
				ulaw = ulawbuf;
				pcm = pcmbuf;

				/* translate all from ulaw to 16 bit pcm */
				while (ulaw != ulawbuf+cnt) {
					*pcm++ = (ushort) audio_u2s(*ulaw++);
				}

				/* now quantize it using adpcm */
				adpcm_coder(pcmbuf, ulawbuf, cnt, &state);
				write(outfile, ulawbuf, cnt/2);
			}
			close(outfile);
			free(newfn);
		}
		else {
			newfn=strdup(*argv);
			if ((p=strstr(newfn,".pcm")) == NULL) {
				puts("not a valid pcm file");
				continue;
			}
			*p='\0';		/* take off .pcm suffix */

			if ((outfile = open(newfn, O_RDWR |O_CREAT | O_EXCL,
					    0644)) < 0) {
				printf("%s already exists\n",newfn);
				free(newfn);
				continue;
			}
			fstat(file,&statbuf);
			audiohdr.sample_rate = 8000;
			audiohdr.samples_per_unit = 1;
			audiohdr.bytes_per_unit = 1;
			audiohdr.channels = 1;
			audiohdr.encoding = AUDIO_ENCODING_ULAW;

			/* now tell it how big the audio file is. The original
			 * file has exactly twice as many playable bytes as
			 * this one, so we double the current number.
			 * There are also 32Bytes in
			 * the sound header, but they don't count */
			audiohdr.data_size = statbuf.st_size * 2;

			if (audio_write_filehdr(outfile, &audiohdr, NULL, 0) 
							!= AUDIO_SUCCESS) {
				perror("writing header");
				exit(5);
			}
			while ((cnt = read(file, ulawbuf, sizeof(ulawbuf)/2))
			                                     > 0) {
				ulaw = ulawbuf;
				pcm = pcmbuf;

				/* first we decode from adpcm format */
				adpcm_decoder(ulawbuf, pcmbuf, cnt*2, &state);

				/* now translate from 16bit pcm to Sun
				 * mulaw format */
				while (pcm != pcmbuf+cnt*2) {
					*ulaw++ = audio_s2u(*pcm++);
				}
				write(outfile, ulawbuf, cnt*2);
			}
		}
	}
	exit(0);
}
