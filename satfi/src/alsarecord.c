/*

This example reads from the default PCM device
and writes to standard output for 5 seconds of data.

*/

/* Use the newer ALSA API */
#define ALSA_PCM_NEW_HW_PARAMS_API

#include <alsa/asoundlib.h>
#include <linux/soundcard.h>

//录制pcm数据音频文件
void RecordPcmAudio(int rate, int bits, int channels, char *filename, int second)
{	
    int dspfd;
    int pcmfd; /* pcm文件的描述符 */
    int arg;   /* ioctl arg */
    int ret;   /* return value */

    unsigned char buff[rate * bits * channels / 8]; //buff里面正好放一秒钟的音频
    /* open device */
    dspfd = open("/dev/dsp", O_RDONLY);
    if (dspfd < 0) 
	{
        printf("open of /dev/dsp failed");
        exit(1);
    }
	
    pcmfd = open(filename, O_WRONLY|O_CREAT, 0644);
    if (pcmfd < 0) 
	{
        printf("open of wav failed\n");
        close(dspfd);
        exit(1);
    }

    /* set bits */
    arg = bits;
    ret = ioctl(dspfd, SOUND_PCM_WRITE_BITS, &arg);
    if (ret == -1)
    {
		perror("SOUND_PCM_WRITE_BITS ioctl failed");
	}

    /* set channels */
    arg = channels;
    ret = ioctl(dspfd, SOUND_PCM_WRITE_CHANNELS, &arg);
    if (ret == -1)
    {
        perror("SOUND_PCM_WRITE_CHANNELS ioctl failed");
	}

    /* set rate */
    arg = rate;
    ret = ioctl(dspfd, SOUND_PCM_WRITE_RATE, &arg);
    if (ret == -1)
    {
		perror("SOUND_PCM_WRITE_WRITE ioctl failed");
	}       

	int i;
	for(i=0; i<second; i++)
	{
		ret = read(dspfd, buff, sizeof(buff));
		if (ret != sizeof(buff))
			perror("read wrong number of bytes");
		
		ret = write(pcmfd, buff, ret);
		if (ret != sizeof(buff))
      		perror("wrote wrong number of bytes");

	}

    close(dspfd);
    close(pcmfd);
}

//second单位秒
int record_audio(int rate, int bits, int channels, char *filename, int second)
{
	long loops;
	int size;
	int rc;
	snd_pcm_t *handle;
	snd_pcm_hw_params_t *params;
	snd_pcm_uframes_t frames;
	unsigned int val;
	int dir;
	char *buffer;

	int fd_f;
	if(( fd_f = open(filename, O_CREAT|O_RDWR, 0777)) == -1)
	{
		perror("cannot creat the sound file");
	}

	/* Open PCM device for recording (capture). */
	rc = snd_pcm_open(&handle, "default", SND_PCM_STREAM_CAPTURE, 0);
	if (rc < 0) {
		fprintf(stderr, "unable to open pcm device: %s\n", snd_strerror(rc));
		exit(1);
	}

	/* Allocate a hardware parameters object. */
	snd_pcm_hw_params_alloca(&params);

	/* Fill it in with default values. */
	snd_pcm_hw_params_any(handle, params);

	/* Set the desired hardware parameters. */

	/* Interleaved mode */
	snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);

	/* Signed 16-bit little-endian format */
	if(bits == 16)
	{
		snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE); 	//BITS
	}
	else if(bits == 8)
	{
		snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S8); 	//BITS
	}

	/* Two channels (stereo) */
	snd_pcm_hw_params_set_channels(handle, params, channels);		//CHANNELS

	/* 44100 bits/second sampling rate (CD quality) */
	val = rate;//RATE
	snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);
	
	/* Set period size to 32 frames. */
	frames = 32;
	snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);
	
	/* Write the parameters to the driver */
	rc = snd_pcm_hw_params(handle, params);
	if (rc < 0) 
	{
		fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc));
		exit(1);
	}

	/* Use a buffer large enough to hold one period */
	snd_pcm_hw_params_get_period_size(params, &frames, &dir);									  
	size = frames * (bits/8) * channels; /* 2 bytes/sample, 2 channels */
	buffer = (char *) malloc(size);

	/* We want to loop for 5 seconds */
	snd_pcm_hw_params_get_period_time(params, &val, &dir);
	loops = second*1000000 / val;

	while (loops > 0) 
	{
		loops--;
		rc = snd_pcm_readi(handle, buffer, frames);
		if (rc == -EPIPE) 
		{
		  /* EPIPE means overrun */
		  fprintf(stderr, "overrun occurred\n");
		  snd_pcm_prepare(handle);
		} 
		else if (rc < 0) 
		{
		  fprintf(stderr,"error from read: %s\n",snd_strerror(rc));
		} 
		else if (rc != (int)frames) 
		{
		  fprintf(stderr, "short read, read %d frames\n", rc);
		}
		
		rc = write(fd_f, buffer, size);
		if (rc != size)
		{
			fprintf(stderr, "short write: wrote %d bytes\n", rc);
		}
	}

	snd_pcm_drain(handle);
	snd_pcm_close(handle);
	free(buffer);

	return 0;
}
