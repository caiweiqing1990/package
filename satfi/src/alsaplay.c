/*

This example reads standard from input and writes
to the default PCM device for 5 seconds of data.

*/

/* Use the newer ALSA API */
#define ALSA_PCM_NEW_HW_PARAMS_API

#include <alsa/asoundlib.h>
#include <linux/soundcard.h>

//播放pcm数据音频文件
void PlayPcmAudio(int rate, int bits, int channels, char *filename)
{	
    int dspfd;
    int pcmfd; /* pcm文件的描述符 */
    int arg;   /* ioctl arg */
    int ret;   /* return value */
	
    unsigned char buff[rate * bits * channels / 8]; //buff里面正好放一秒钟的音频
    /* open device */
    //dspfd = open("/dev/dsp", O_WRONLY);
    dspfd = open("/dev/dsp", O_RDWR);
    if (dspfd < 0) 
	{
        printf("open of /dev/dsp failed");
        exit(1);
    }
	
    pcmfd = open(filename, O_RDONLY);
    if (pcmfd < 0) 
	{
        printf("open of wav failed");
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

    /* 从wav文件中读buff大小的内容，然后写入/dev/dsp中，直到文件结束 */
    while (1) 
	{
		//start = time(NULL);
		if((ret = read(dspfd, buff, sizeof(buff))) <= 0)
		{
			break;
		}
        write(dspfd, buff, ret);
		
        /* 下面的代码用于在更改播放文件的参数时，播放掉缓冲区内的内容 */
        //ret = ioctl(dspfd, SOUND_PCM_SYNC, 0);
        //if (ret == -1)
        //{
         //  perror("SOUND_PCM_SYNC ioctl failed");
		//}
       // end = time(NULL);
		//printf("val = %f\n", difftime(end, start));
    }

    close(dspfd);
    close(pcmfd);
}

int play_audio(int rate, int bits, int channels, char *filename)
{
	int rc;
	int size;
	snd_pcm_t *handle;
	snd_pcm_hw_params_t *params;
	snd_pcm_uframes_t frames;
	unsigned int val;
	int dir;
	char *buffer;

	int fd_f;
	if((fd_f = open(filename, O_RDONLY)) == -1)//open a pcm audio file
	{
		perror("cannot open the sound file");
		exit(1);
	}

	/* Open PCM device for playback. */
	rc = snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
	if (rc < 0)
	{
		fprintf(stderr,"unable to open pcm device: %s\n",snd_strerror(rc));
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
	
	/* one/Two channels (mono/stereo) */
	snd_pcm_hw_params_set_channels(handle, params, channels); 	//CHANNELS

	/* 44100 bits/second sampling rate (CD quality) */
	val = rate;											//RATE
	snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);

	/* Set period size to 32 frames. */
	frames = 32;
	snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

	/* Write the parameters to the driver */
	rc = snd_pcm_hw_params(handle, params);
	if (rc < 0) {
		fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc));
		exit(1);
	}

	/* Use a buffer large enough to hold one period */
	snd_pcm_hw_params_get_period_size(params, &frames, &dir);
	size = frames * (bits/8) * channels; /* 2 bytes/sample, 2 channels */
	
	buffer = (char *) malloc(size);

	/* We want to loop for 5 seconds */
	snd_pcm_hw_params_get_period_time(params, &val, &dir);
	/* 5 seconds in microseconds divided by
	* period time */
	//loops = 5000000 / val;

	while (1) 
	{
		rc = read(fd_f, buffer, size);
		if (rc == 0) //没有读取到数据 
		{
			fprintf(stderr, "end of file on input\n");
			break;
		} 
		else if (rc != size)//实际读取 的数据 小于 要读取的数据 
		{
			fprintf(stderr,"short read: read %d bytes\n", rc);
		}
		else if (rc < 0)
		{
			fprintf(stderr,"error\n");
			break;//error
		}

		rc = snd_pcm_writei(handle, buffer, frames);//写入声卡  （放音） 
		if (rc == -EPIPE) 
		{
			/* EPIPE means underrun */
			fprintf(stderr, "underrun occurred\n");
			snd_pcm_prepare(handle);
		} 
		else if (rc < 0) 
		{
			fprintf(stderr,"error from writei: %s\n",snd_strerror(rc));
		}  
		else if (rc != (int)frames) 
		{
			fprintf(stderr,"short write, write %d frames\n", rc);
		}
	}

	snd_pcm_drain(handle);
	snd_pcm_close(handle);
	free(buffer);

	return 0;
}
