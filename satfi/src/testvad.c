#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <speex/speex.h>

#include "speex/speex_preprocess.h"
#include <stdio.h>

#define NN 160

#if 0
gcc testvad.c -o testvad -lspeexdsp
./testvad dec.raw vad.raw
#endif

int main(int argc, char **argv)
{
	short in[NN];
	SpeexPreprocessState *st;
	int speechcnt=0;
	int quitecnt=0;
	int count=0;

	st = speex_preprocess_state_init(NN, 8000);

	int vad = 1;
	int vadProbStart = 90;
	int vadProbContinue = 100;
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_VAD, &vad); //静音检测
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_START , &vadProbStart); //Set probability required for the VAD to go from silence to voice
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_CONTINUE, &vadProbContinue); //Set probability required for the VAD to stay in the voice state (integer percent)

	FILE *fin = fopen(argv[1], "r");
	FILE *fout= fopen(argv[2], "w");
	
	while (1)
	{
		int vad;
		fread(in, sizeof(short), NN, fin);
		if (feof(fin))
		break;
		vad = speex_preprocess_run(st, in);
		//fprintf (stderr, "vad=%d\n", vad);
		//fwrite(in, sizeof(short), NN, fout);
		if(vad==1)
		{
			fwrite(in, sizeof(short), NN, fout);
			++speechcnt;
		}
		if(vad==0)++quitecnt;
		++count;
	}
	
	fprintf (stderr, "speechcnt=%d\n", speechcnt);
	fprintf (stderr, "quitecnt=%d\n", quitecnt);
	fprintf (stderr, "count=%d\n", count);
	
	speex_preprocess_state_destroy(st);
	fclose(fin); 
	fclose(fout); 
	return 0;
}
