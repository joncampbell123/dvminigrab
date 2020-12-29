/* minimalist camcorder DV firewire capture
 * (C) 2009-2010 Impact Studio Pro ALL RIGHTS RESERVED
 * Written by Jonathan Campbell
 *
 * WARNING: This code has been massively modified for the latest kernel firewire
 *          interface and libraw1394 API. The O_NONBLOCK trick no longer works,
 *          and even if it did, the iterator function apparently will block for
 *          a significant period of time and cause us a bad capture. So the new
 *          code uses select() as recommended by the libraw1394 devs, and uses
 *          some new more efficient methods to do it's job.
 *
 * Beats the pants of off dvgrab 3.4 in memory utilization and efficiency,
 * primarily because this code doesn't concern itself with trying to support
 * direct to QT or AVI capture. Just raw capture, that's all we need :)
 *
 * dvgrab-3.4:           %3.0 CPU util,    100MB memory utilization
 * dvminigrab:           %1.0 CPU util,    8MB memory utilization
 *
 * TODO: QuickTime seems to support raw DVC50, and Nathan's HVX will emit
 *       DVC-50. See if you can adapt this code to support it!
 *
 * also allows a host program to signal when to split, and other useful "staging" features
 *
 * NOTE: This code relies on libclocksync, because the most likely use of this program
 *       is in a live video switching environment where strict synchronization with the
 *       master clock is a must.
 *
 * UPDATE: New edition uses pthreads to run frame assembly+capture in separate frame. */

#define _XOPEN_SOURCE  500

#include <sys/param.h>
#include <sys/mount.h>
#include <sys/select.h>
#include <sys/statfs.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <signal.h>
#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>

#include <endian.h>

#include <pthread.h>

#include "libclocksync.h"

#include <libraw1394/raw1394.h>
#include <libavc1394/avc1394.h>
#include <libavc1394/rom1394.h>
#include <libraw1394/ieee1394.h>
#include <libiec61883/iec61883.h>

#include "dvminigrab.h"

/* hey jerks: it would be nice if the functions to just mess with oPCR regs were NOT private!
 * leaving public only a bunch of high-level crap is NOT useful, especially when all they allow
 * me to do is high-level connect-to-connect crap when I need to do LOW LEVEL STUFF! */
#include "libiec61883-private.h"

volatile int global_death = 0;

void sigma(int x) {
	alarm(5);
	global_death++;
}

#ifdef __amd64__
signed long lseek64(int fd,signed long w,int whence);
#else
signed long long lseek64(int fd,signed long long w,int whence);
#endif

/* macros */
void wr_u16(unsigned char *x,uint16_t v) {
	x[0] = (unsigned char)v;
	x[1] = (unsigned char)(v>>8);
}

void wr_u32(unsigned char *x,uint32_t v) {
	x[0] = (unsigned char)v;
	x[1] = (unsigned char)(v>>8);
	x[2] = (unsigned char)(v>>16);
	x[3] = (unsigned char)(v>>24);
}

pthread_mutex_t		capture_mutex = PTHREAD_MUTEX_INITIALIZER;

int capture_mutex_lock() {
	return (pthread_mutex_lock(&capture_mutex) == 0);
}

int capture_mutex_try_lock() {
	return (pthread_mutex_trylock(&capture_mutex) == 0);
}

int capture_mutex_unlock() {
	return (pthread_mutex_unlock(&capture_mutex) == 0);
}

pthread_t		capture_thread = 0;
volatile int		capture_thread_die = 0;

/* when both threads are active, access to these variables must be protected with the capture mutex */
raw1394handle_t		raw1394 = NULL;		/* raw1394 handle */
int			host_index = -1;	/* host index (i.e. "port" number used in plugctl) */
int			node_index = -1;	/* node index (i.e. "node" number used in plugctl) */
int			sensereset = 1;
int			autosense = 1;
int			channel = -1;		/* channel to get video by */
int			passive = 0;		/* 1=don't change channels, passively listen to broadcast */
uint64_t		guid = 0;		/* GUID of device we're going for */
int			unfiltered = 0;		/* Store DV data as-is, no filtering */
int			bcast = 0;		/* capture using bcast method (or 0 = p2p method) */
/* these too */
int			capture_buffer_frame_size = 12000*10*4;
volatile int		capture_buffer_i = 0,capture_buffer_o = 0;
double*			capture_buffer_time = NULL;
int			capture_buffer_size = 0;
unsigned char*		capture_buffer = NULL;
int			capture_buffer_is_shmem = 0;	/* 1=capture_buffer is actually shared memory segment in /dev/shm */
const char*		capture_buffer_shmem = NULL;	/* non-null, use this /dev/shm path */

volatile dv_capture_shmem_header*	capture_shmem_header = NULL;

static unsigned char	last_DBC=0,last_DBC_init=0;
static long long	DBC_packets_lost=0;

const int		page_size = 4096;

int			isp_header = 0;		/* at start of every capture, write ISP "marker" frame */
int			holding_pattern = 0;	/* capture in "holding pattern" style */
enum raw1394_iso_dma_recv_mode recv_mode = RAW1394_DMA_PACKET_PER_BUFFER;//RAW1394_DMA_DEFAULT;
int			iso_active = 0;
int			no_writing = 0;

char			capture_prefix[128] = {0};

long long		capture_pos = 0;
int			capture_fd = -1;

double			log_fd_interval = 15.0;
double			log_fd_next = -1;
int			log_fd = -1;

void bell() {
	/* on an actual terminal this causes an audible beep */
	char *c = "\x07\x07";
	write(1,c,2);
}

void free_capture_buffer() {
	if (capture_buffer) {
		if (capture_buffer_is_shmem)
			munmap(capture_buffer,capture_buffer_frame_size * capture_buffer_size);
		else
			free(capture_buffer);

		capture_buffer = NULL;
	}

	if (capture_buffer_time) {
		free(capture_buffer_time);
		capture_buffer_time = NULL;
	}

	if (capture_shmem_header) {
		munmap((void*)capture_shmem_header,page_size);
		capture_shmem_header = NULL;
	}

	capture_buffer_i = capture_buffer_o = capture_buffer_size = 0;
}

int init_capture_buffer(int sz,unsigned int frame_size) {
	if (sz < 2) sz = 2;
	if (!capture_buffer || capture_buffer_size < sz)
		free_capture_buffer();

	capture_buffer_frame_size = frame_size;
	capture_buffer_i = capture_buffer_o = 0;
	capture_buffer_size = sz;

	if (capture_buffer_shmem) {
		capture_buffer_is_shmem = 1;

		int fd = open(capture_buffer_shmem,O_RDWR|O_CREAT|O_TRUNC,0644);
		if (fd < 0) {
			fprintf(stderr,"Error! Cannot open shared memory segment %s\n",capture_buffer_shmem);
			return 1;
		}

		/* mapping layout:
		 *    PAGE 0 = header + info + tracking
		 *    PAGE 1...N = DV data */

		if (ftruncate(fd,(capture_buffer_size * capture_buffer_frame_size)+page_size+65536) < 0) {
			fprintf(stderr,"Cannot ftruncate out the shared memory segment\n");
			return 1;
		}

		capture_shmem_header = (dv_capture_shmem_header*)
			mmap(NULL,page_size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);

		capture_buffer = (unsigned char*)mmap(NULL,(capture_buffer_size*capture_buffer_frame_size)+65536,PROT_READ|PROT_WRITE,MAP_SHARED,fd,page_size);
		if (capture_buffer == (unsigned char*)(-1)) {
			capture_buffer = NULL;
			fprintf(stderr,"Cannot mmap, %s\n",strerror(errno));
		}

		close(fd);	/* the memory mapping persists even after closing the file */
		printf("Buffering with shared memory segment %s\n",capture_buffer_shmem);

		memcpy((char*)capture_shmem_header->sig,"DV25",4);
		capture_shmem_header->version = DVMINIGRAB_STRUCT_VERSION;
		capture_shmem_header->frame_size = capture_buffer_frame_size;
		capture_shmem_header->frames = capture_buffer_size;
		capture_shmem_header->output = capture_buffer_o;

		/* and write a note of it too */
		fd = open("shmem",O_WRONLY|O_CREAT|O_TRUNC,0644);
		write(fd,capture_buffer_shmem,strlen(capture_buffer_shmem));
		close(fd);
	}
	else {
		capture_buffer_is_shmem = 0;
		capture_buffer = malloc(capture_buffer_size * capture_buffer_frame_size);
	}

	if (!capture_buffer)
		return 1;

	capture_buffer_time = malloc(capture_buffer_size * sizeof(double));
	if (!capture_buffer_time)
		return 1;

	return 0;
}

/* WARNING: caller must hold the mutex while calling this! */
int capture_buffer_frames_ready() {
	int x = capture_buffer_o - capture_buffer_i;
	if (x < 0) x += capture_buffer_size;
	return x;
}

typedef struct {
	unsigned char           *raw;
	unsigned char           SCT,Arb,Dseq,FSC,DBN;   // decoded from DIF header
} DIFchunk;

static DIFchunk ReadDIFtmp;
static int dv25_discard = 0;	/* discard first frame, because it's likely incomplete */

DIFchunk *PickDIF(unsigned char *raw)
{
	DIFchunk *d = &ReadDIFtmp;

	d->raw =   raw;
	/* pick apart DIF header */
	d->SCT =   d->raw[0] >> 5;
	d->Arb =   d->raw[0]       & 0xF;
	d->Dseq =  d->raw[1] >> 4;
	d->FSC =  (d->raw[1] >> 3) & 1;
	d->DBN =   d->raw[2];
	return d;
}

void modify_header_enable_audio(unsigned char *frame,int sz) {
	unsigned char *fence = frame + sz;

	/* modify the header DIF to enable audio.
	 * or else QuickTime will not play the audio despite it being there in the stream */
	for (;frame < fence;frame += 150*80*4) {
		DIFchunk *hdr = PickDIF(frame);
		if (hdr->SCT == 0) {
			unsigned char *H = hdr->raw;
			H[4] = 0x78;
			H[5] = 0x78;
			H[6] = 0x78;
			H[7] = 0x78;
		}
	}
}

int dv_read_aspect_ratio(unsigned char *frame) {
	unsigned char *DV = frame;
	unsigned char *fence = DV + capture_buffer_frame_size;
	int ar = -1;

	for (;DV < fence && ar < 0;DV += 80) {
		DIFchunk *c = PickDIF(DV);
		if (c->SCT == 2) {
			unsigned char *B = c->raw+3;
			unsigned char *Bfence = B+(15*5);
			for (;B < Bfence && ar < 0;B += 5) {
				if (B[0] == 0x61) { /* source control VAUX */
					unsigned char arc = B[2] & 7;

					switch (arc) {
						case 0:
						case 2:
							ar = arc;
							break;
					};
				}
			}
		}
	}

	return ar;
}

int capture_file_sequence = 1;

int close_capture() {
	if (capture_fd >= 0) {
		if (++capture_file_sequence >= 0xFE00)
			capture_file_sequence = 0;

		close(capture_fd);
		capture_pos = 0;
		capture_fd = -1;
	}
	if (log_fd >= 0) {
		log_fd_next = -1;
		close(log_fd);
		log_fd = -1;
	}
	return 0;
}

int open_capture() {
	if (capture_fd >= 0)
		return 1;

	char name[257];
	if (holding_pattern)
		snprintf(name,sizeof(name),"dvcap-%08lu-%s.holding.dv",time(NULL),capture_prefix);
	else
		snprintf(name,sizeof(name),"dvcap-%08lu-%s.dv",time(NULL),capture_prefix);

	if ((capture_fd = open(name,O_RDWR|O_CREAT,0644)) < 0) {
		fprintf(stderr,"Cannot open %s\n",name);
		return 1;
	}

	capture_pos = lseek64(capture_fd,0,SEEK_END);
	capture_pos -= capture_pos % (12000 * 10 * 4);
	if (capture_pos < 0) capture_pos = 0;

	char logname[257];
	{
		snprintf(logname,sizeof(logname),"%s.log",name);
		if ((log_fd = open(logname,O_RDWR|O_CREAT|O_APPEND,0644)) < 0)
			fprintf(stderr,"Cannot open %s (LOG)\n",logname);
	}

	printf("Capture to file changed:\n");
	printf("   DV=%s\n",name);
	if (log_fd >= 0) printf("   LOG=%s\n",logname);
	printf("   hold=%d\n",holding_pattern);
	printf("   firewire_recv_mode=%s\n",bcast ? "broadcast" : "p2p");

	log_fd_next = -1;
	if (log_fd >= 0) {
		char hostname[128] = {0};
		char tmp[256];

		gethostname(hostname,sizeof(hostname)-1);
		sprintf(tmp,"\n");					write(log_fd,tmp,strlen(tmp));
		sprintf(tmp,"HOST %s\n",hostname);			write(log_fd,tmp,strlen(tmp));
		sprintf(tmp,"STARTED AT %.3f\n",clocksync_master());	write(log_fd,tmp,strlen(tmp));
		sprintf(tmp,"\n");					write(log_fd,tmp,strlen(tmp));
	}

	/* next, you write the file that other programs can use to quickly figure out
	 * which file is the one capturing to now */
	int fd = open("now-capturing~",O_CREAT|O_TRUNC|O_WRONLY,0644);
	if (fd > 0) {
		write(fd,name,strlen(name));
		close(fd);
		rename("now-capturing~","now-capturing");
	}

	return 0;
}

void mode_capture() {
	if (holding_pattern) {
		holding_pattern=0;
		close_capture();
		open_capture();
	}
}

void mode_hold() {
	if (!holding_pattern) {
		holding_pattern=1;
		close_capture();
		open_capture();
	}
}

static inline uint16_t bsw16(uint16_t v) {
	return (v >> 8) | (v << 8);
}

static int upsample12(int y)
{
	int shift = (y >> 8) & 0xF;

	if (shift >= 2 && shift < 8) {
		shift--;
		return (y - (shift<<8)) << shift;
	}
	else if (shift >= 8 && shift <= 0xd) {
		shift = 0xe - shift;
		return ((y + ((shift<<8)+1)) << shift) - 1;
	}

	return y;
}

/* DV decoding function */
int dv_audio_p[2] = {0,0};	/* previous samples, for upsampling 32KHz -> 48KHz */
int quants[8] = {16,12,-1,-1,-1,-1,-1,-1};
int srates[4] = {48000,44100,32000,-1};
int min_samples[4] = {1580,1452,1053,-1};
int dv_decode_audio(unsigned char *frame,int len,int16_t *audio) {
	int16_t tempa[2048*2],*audec = audio;
	unsigned char *blk;
	int audio_len = -1;
	int samples = -1;
	int srate = -1;
	int quant = -1;
	int i,j,bn,N;

	/* go through the audio DIF blocks and decode audio */
	for (i=0;i < 40;i++) {
		for (j=0;j < 9;j++) {
			bn = (i * 150) + (j << 4) + 6;
			blk = frame + (bn * 80);

			if ((blk[0] >> 5) == 3 && blk[3] == 0x50) {
				unsigned char *data = blk+3;
				unsigned char more_samples = data[1] & 0x3F;
				unsigned char sidx = (data[4] >> 3) & 3;
				quant = quants[data[4] & 7];
				srate = srates[sidx];
				audio_len = min_samples[sidx] + more_samples;
			}
		}
	}

	if (quant < 0 || srate <= 0 || audio_len <= 0)
		return -1;

	if (srate != 48000)
		audec = tempa; /* decode to temp, upsample to caller */

	/* fill with invalid sample */
	for (N=0;N < audio_len*2;N++)
		audec[N] = (int16_t)0x8000;

	if (quant == 16) {
		for (N=0;N < audio_len;N++) {
			int ds1,ds2,da,b,bn1,bn2;
			signed short *ptr;

			ds1 = ((N / 3) + ((N % 3) * 2)) % 5;
			ds2 = ds1 + 5;
			da  = (3 * (N % 3)) + ((N % 45) / 15);
			b   = 2 * (N / 45);
			bn1 = (ds1 * 150) + (da << 4) + 6;
			bn2 = (ds2 * 150) + (da << 4) + 6;

			if (frame[bn1 * 80] != 0xFF) {
				ptr = (signed short*)(frame + (bn1 * 80) + 8 + b);
#if BYTE_ORDER == BIG_ENDIAN
				audec[N<<1] = *ptr;
#else
				audec[N<<1] = (signed short)bsw16(*ptr);
#endif
			}

			if (frame[bn2 * 80] != 0xFF) {
				ptr = (signed short*)(frame + (bn2 * 80) + 8 + b);
#if BYTE_ORDER == BIG_ENDIAN
				audec[(N<<1)+1] = *ptr;
#else
				audec[(N<<1)+1] = (signed short)bsw16(*ptr);
#endif
			}
		}
	}
	else if (quant == 12) {
		/* FIXME: this doesn't work! */
		static int dv_audio_unshuffle_60[5][9] = {
			{ 0, 15, 30, 10, 25, 40,  5, 20, 35 },
			{ 3, 18, 33, 13, 28, 43,  8, 23, 38 },
			{ 6, 21, 36,  1, 16, 31, 11, 26, 41 },
			{ 9, 24, 39,  4, 19, 34, 14, 29, 44 },
			{12, 27, 42,  7, 22, 37,  2, 17, 32 }};
		unsigned char *blk;
		int stride = 45;
		int bp,i,i_base;
		unsigned char my,mz,l;
		int ds,dif,ch;

		for (ch=0;ch < 1;ch++) {
			for (ds=0;ds < 5;ds++) {
				for (dif=0;dif < 9;dif++) {
					int dsb = ds + (ch * 5);
					int bn = (dsb * 150) + (dif << 4) + 6;

					blk = frame + (bn * 80);
					if (*blk == 0xFF) continue;
					i_base = dv_audio_unshuffle_60[ds][dif];
					for (bp=8;bp < 80;bp += 3) {
						i = i_base + (bp - 8)/3 * stride;
						my = blk[bp];
						mz = blk[bp+1];
						l  = blk[bp+2];

						int y = (my << 4) | (l >> 4);
						int z = (mz << 4) | (l & 0xF);
						if (y > 2048) y -= 4096;
						if (z > 2048) z -= 4096;

						if (y == 2048)  y = 0x8000;
						else            y = upsample12(y);

						if (z == 2048)  z = 0x8000;
						else            z = upsample12(z);

						audec[i<<1] = y;
						audec[(i<<1)+1] = z;
					}
				}
			}
		}
	}

	/* fill in "invalid" samples */
	{
		int l=0,r=0;
		for (i=0;i < audio_len;i++) {
			if (audec[i<<1] == (int16_t)0x8000)
				audec[i<<1] = l;
			else
				l = (int)audec[i<<1];

			if (audec[(i<<1)+1] == (int16_t)0x8000)
				audec[(i<<1)+1] = r;
			else
				r = (int)audec[(i<<1)+1];
		}
	}

	if (audec != audio) {
		int i=0,o=0,sc=0;

		while (o < (2048*2) && i < (audio_len*2)) {
			audio[o++] = (
				((int)audec[i] * sc) +
				(dv_audio_p[0] * (48000 - sc))
				) / 48000;
			audio[o++] = (
				((int)audec[i+1] * sc) +
				(dv_audio_p[1] * (48000 - sc))
				) / 48000;
			sc += srate;
			if (sc >= 48000) {
				dv_audio_p[0] = audec[i];
				dv_audio_p[1] = audec[i+1];
				sc -= 48000;
				i += 2;
			}
		}

		samples = o>>1;
	}
	else {
		samples = audio_len;
	}

	return samples;
}

int holding_counter = 0;
int frame_count = 0;
int announced_frame_count = 0;
long long holding_limit = ( 12000 * 10 * 4 ) * 30 * 60;	// 1 min
double last_timestamp = 0;
int known_aspect_ratio = -1;
/* Caller must hold mutex! */
int empty_capture_buffer(int max) {
	int wrote = 0;

	if (max < 1)
		max = 100;

	if (capture_fd < 0)
		return 0;

	while (capture_buffer_i != capture_buffer_o && wrote < max) {
		if ((capture_pos % (12000 * 10 * 4)) != 0) {
			printf("Warning: capture pos is not frame aligned (%lu)\n",(unsigned long)(capture_pos % (12000 * 10 * 4)));
			capture_pos += 12000 * 10 * 4;
			capture_pos -= capture_pos % (12000 * 10 * 4);
		}

		signed long long wpos = lseek64(capture_fd,capture_pos,SEEK_SET);
		if (wpos != capture_pos) {
			printf("Whoah! Capture file descriptor lseek'd without my guidance. "
				"Splitting capture now (%llu != %llu)\n",wpos,capture_pos);

			close_capture();
			open_capture();
			if (capture_fd < 0) break;
			continue;
		}

		unsigned char *frame = capture_buffer + (capture_buffer_i * capture_buffer_frame_size);
		signed long long start = lseek64(capture_fd,0,SEEK_CUR);
		int length = 0;

		if (start != capture_pos) {
			printf("Whoah! Capture file descriptor lseek'd without my guidance (2). "
				"Splitting capture now (%llu != %llu)\n",start,capture_pos);

			close_capture();
			open_capture();
			if (capture_fd < 0) break;
			continue;
		}

again:
		/* ISP marker frame. at offset == 0 (begin of file) we write a "marker" frame.
		 * as well as providing visual evidence this is ISP property, we also embed into this
		 * frame proprietary data */
		if (start == 0) {
			known_aspect_ratio = dv_read_aspect_ratio(frame);
		}

		/* optimization trick: for consecutive available frames, just write more! */
		int capi = capture_buffer_i;
		int wrote_here = 0;
		do {
			/* also: strictly limit ourself if it would allow writing past the holding pattern limit */
			if (holding_pattern && (start+length) >= holding_limit)
				break;

			wrote++;
			wrote_here++;
			length += capture_buffer_frame_size;
			if (++capture_buffer_i >= capture_buffer_size) {
				capture_buffer_i = 0;
				break;
			}
		} while (capture_buffer_i != capture_buffer_o && wrote < max);

		/* must add "holding pattern" marking */
		if (holding_pattern) {
			unsigned char *p = frame;
			unsigned char *f = frame + length;
			for (;p < f;p += 12000*10*4) {
				int seq;
				for (seq=0;seq < 10;seq++) {
					unsigned char *d = p + (seq*150*80);
					strcpy((char*)(d+70),"hold");
					d[75] = holding_counter>>8;
					d[76] = holding_counter;
					d[77] = capture_file_sequence>>8;
					d[78] = capture_file_sequence;
				}
				holding_counter++;
			}
		}

		if (write(capture_fd,frame,length) < length)
			printf("Warning: write() could not write the entire block of data!\n");
		long long endo = lseek64(capture_fd,0,SEEK_CUR);
		if (endo != (start+length)) printf("Warning: write+lseek() not consistent (%llu != %llu)\n",endo,start+length);
		capture_pos = start+length;

		/* and log frame times */
		{
			int i,o,c=wrote_here;
			char tmp[512];
			for (i=capi,o=0;c-- > 0;capi++,o++) {
				assert(capi < capture_buffer_size);

				if (capture_buffer_time[capi] < 0)
					continue;

				sprintf(tmp,"@%.3f: DV=%lld wr=%d/%d idx=%d/%d\n",capture_buffer_time[capi],(int64_t)start+(o*12000*10LL*4),
					o,wrote,capi,capture_buffer_size);
				write(log_fd,tmp,strlen(tmp));

				if (capture_buffer_time[capi] < last_timestamp) {
					sprintf(tmp,"!Whoah! Discontinious timestamp! Possible pthread concurrency problem! %.3f < %.3f\n",
						capture_buffer_time[capi],last_timestamp);
					write(log_fd,tmp,strlen(tmp));
					printf("%s",tmp);
				}
				else {
					last_timestamp = capture_buffer_time[capi];
				}
			}
		}

		if (holding_pattern) {
			if (endo >= holding_limit) { /* 1 min. holding pattern */
				printf("Holding mode: rolling back around (%llu >= %llu)\n",endo,holding_limit);
				lseek64(capture_fd,0,SEEK_SET);
				capture_pos = 0;
			}
		}

		if ((frame_count-announced_frame_count) >= 30) {
			announced_frame_count = frame_count;
			if (isatty(1)) printf("\x0D");
			printf("Frame %d (lost packets=%llu)     ",frame_count,DBC_packets_lost);
			if (isatty(1)) printf("\x0D");
			else printf("\n");
			fflush(stdout);
		}
		frame_count += length/capture_buffer_frame_size;
	}

	return wrote;
}

int init_raw1394() {
	if (raw1394 != NULL)
		return 1;
	if ((raw1394 = raw1394_new_handle()) == NULL)
		return 1;

	raw1394_set_userdata(raw1394,NULL);
	return 0;
}

int free_raw1394() {
	if (raw1394 != NULL) raw1394_destroy_handle(raw1394);
	raw1394 = NULL;
	return 0;
}

int find_device_by_guid() {
	struct raw1394_portinfo pi[64];
	int ports,port;

	if (init_raw1394())
		return 1;

	memset(pi,0,sizeof(pi));
	ports = raw1394_get_port_info(raw1394,pi,64);
	if (ports < 0)
		return 1;

	for (port=0;port < ports;port++) {
		free_raw1394();
		if (init_raw1394()) break;
		if (raw1394_set_port(raw1394,port) < 0) continue;
		struct raw1394_portinfo *p = pi+port;
		int nodes = p->nodes,node;
		for (node=0;node < nodes;node++) {
			if (guid == rom1394_get_guid(raw1394,node)) {
				fprintf(stderr,"GUID %llx matches port %d node %d\n",(long long)guid,port,node);

				raw1394_busreset_notify(raw1394,RAW1394_NOTIFY_OFF);

				/* BUT: is it an AV/C device? */
				rom1394_directory rom_dir;
				if (rom1394_get_directory(raw1394,node,&rom_dir) < 0) {
					fprintf(stderr,"Rom1394 cannot read directory\n");
					return 1;
				}
				if (rom1394_get_node_type(&rom_dir) != ROM1394_NODE_TYPE_AVC) {
					fprintf(stderr,"Not an AVC device\n");
					return 1;
				}

				host_index = port;
				node_index = node;

				free_raw1394();
				return 0;
			}
		}
	}

	free_raw1394();
	return 1;
}

static void help() {
	fprintf(stderr,"dvminigrab [options]\n");
	fprintf(stderr,"Minimalist DV camcorder capture program (C) 2009 Jonathan Campbell\n");
	fprintf(stderr,"Unlike dvgrab, has low memory footprint and simple raw DV capture design\n");
	fprintf(stderr,"\n");
	fprintf(stderr," --guid <number>            Which device to connect to\n");
	fprintf(stderr," --channel <n>              Firewire channel to capture by (default=63)\n");
	fprintf(stderr," --passive                  Don't reprogram PROM to change channel, passively listen\n");
	fprintf(stderr," --unfiltered               Store incoming DV data as-is, no filtering\n");
	fprintf(stderr," --isp-marker               At the start of every capture write ISP marker frame\n");
	fprintf(stderr," --hold                     Start capture in 'holding pattern' mode\n");
	fprintf(stderr," --prefix                   Preprend this prefix to the capture file name\n");
	fprintf(stderr," --bcast                    Use broadcast capture (default)\n");
	fprintf(stderr," --shmem <path>             Use shared memory segment for capture\n");
	fprintf(stderr," --p2p                      Use peer-to-peer capture\n");
	fprintf(stderr,"                             ! may not work in some combinations of computer and firewire host\n");
	fprintf(stderr," --no-autosense             Do not try to auto-sense disconnect/reconnect of device\n");
	fprintf(stderr," --no-sensereset            Do not poll device to detect reset scenarios\n");
}

static int parse_args(int argc,char **argv) {
	int i,nosw=0;

	for (i=1;i < argc;) {
		char *a = argv[i++];

		if (*a == '-') {
			do { a++; } while (*a == '-');

			if (!strcmp(a,"help")) {
				help();
				return 1;
			}
			else if (!strcmp(a,"shmem")) {
				capture_buffer_shmem = argv[i++];
			}
			else if (!strcmp(a,"guid")) {
				guid = strtoull(argv[i++],NULL,16);
			}
			else if (!strcmp(a,"passive")) {
				passive = 1;
			}
			else if (!strcmp(a,"bcast")) {
				bcast = 1;
			}
			else if (!strcmp(a,"p2p")) {
				bcast = 0;
			}
			else if (!strcmp(a,"unfiltered")) {
				unfiltered = 1;
			}
			else if (!strcmp(a,"isp-marker")) {
				isp_header = 1;
			}
			else if (!strcmp(a,"hold")) {
				holding_pattern = 1;
			}
			else if (!strcmp(a,"prefix")) {
				memset(capture_prefix,0,sizeof(capture_prefix));
				strncpy(capture_prefix,argv[i++],sizeof(capture_prefix)-1);
			}
			else if (!strcmp(a,"channel")) {
				channel = atoi(argv[i++]);
				if (channel < 0 || channel > 63) {
					fprintf(stderr,"Invalid channel number. Valid range: 0-63\n");
					return 1;
				}
			}
			else if (!strcmp(a,"no-autosense")) {
				autosense = 0;
			}
			else if (!strcmp(a,"no-sensereset")) {
				sensereset = 0;
			}
		}
		else {
			nosw++;
		}
	}

	if (channel < 0)
		channel = 63;
	if (channel == 63) {
		printf("Warning! You're capturing on channel 63. Camcorders tend to default to this!\n");
		printf("         This may cause conflicts if another camcorder is attached or multi-camera\n");
		printf("         capture is attempted! In those scenarios please consider another channel.\n");
	}

	if (guid == 0) {
		fprintf(stderr,"No GUID specified\n");
		help();
		return 1;
	}

	return 0;
}

/* caller must have mutex locked */
int capture_buffer_last_frame_written = -1;
void store_output(unsigned char *data,size_t len) {
	if (capture_fd >= 0 && !no_writing) {
		if (len > capture_buffer_frame_size) {
			printf("BUG: Emitting frame larger than capture buffer frame size (%u > %u)\n",(unsigned)len,(unsigned)capture_buffer_frame_size);
			len = capture_buffer_frame_size;
		}

		if ((capture_buffer_o+1)%capture_buffer_size == capture_buffer_i) {
			printf("Whoah! Buffer is full. This means that data isn't being written fast enough\n");
			return;
		}

		capture_buffer_time[capture_buffer_o] = -1.0;
		capture_buffer_last_frame_written = capture_buffer_o;
		memcpy(capture_buffer+(capture_buffer_o*capture_buffer_frame_size),data,len);
		if (++capture_buffer_o >= capture_buffer_size)
			capture_buffer_o = 0;

		if (capture_shmem_header)
			capture_shmem_header->output = capture_buffer_o;
	}
}

#define DV25_PACKET_SIZE 80
/* filtered DV25 frame assembly and management */
const int dv25_max_packet = 1801;
unsigned char dv25_assembly[12000*12*4]; /* enough even for the PAL version */
/* TODO: code to handle PAL */

void init_dv25_assembly() {
	memset(dv25_assembly,0xFF,sizeof(dv25_assembly));
}

void next_dv25_assembly() {
}

void emit_assembled_dv25() {
	store_output(dv25_assembly,12000*10*4);
}

int DIFNull(unsigned char *DV) {
	if (DV[0] == DV[1] && DV[1] == DV[2] && DV[2] == DV[3] && DV[3] == DV[4] && DV[4] == DV[5]) {
		if (DV[0] == 0xFF || DV[0] == 0x00)
			return 1;
	}

	return 0;
}

void dv25_assemble(unsigned char *DV,int packets) {
	static int packetcount = 0;
	for (;packets-- > 0;DV += DV25_PACKET_SIZE) {
		memcpy(dv25_assembly + (packetcount * 80),DV,80);
		packetcount++;

		if (packetcount >= (150*10*4)) {
			packetcount = 0;
			emit_assembled_dv25();
			next_dv25_assembly();
		}
	}
}

static enum raw1394_iso_disposition iso_handler_dv25(raw1394handle_t handle, unsigned char *data, 
	unsigned int length, unsigned char fchannel,
	unsigned char tag, unsigned char sy, unsigned int cycle, 
	unsigned int dropped)
{
	/* what we're capturing: 480 byte DV packet + 8 byte CIP */
	unsigned char *CIP = data;
	unsigned char *DV = data+8;
	unsigned char *fence = data+length;

	if (length <= 8)
		return RAW1394_ISO_OK;
	if (fchannel != channel) {
		fprintf(stderr,"BUG: Libraw1394 + f/w stack let us have a packet not belonging to the channel we asked for (ch=%u)\n",fchannel);
		return RAW1394_ISO_OK;
	}

	/* FMT==0 right (DV)? */
	/* unused variables commented out */
//	unsigned char FMT = CIP[4] & 0x3F;
	unsigned char DBC = CIP[3];	/* data block counter, to see discontinuities */
	unsigned char DBS = CIP[1];	/* lemme guess.... length in DWORDs? That seems correct... */
	if (DBS == 0) DBS = (length-8)>>2;

	/* NTS: Panasonic HVX: DBC counts 2 at a time per callback? */

	if (!last_DBC_init) {
		last_DBC_init = 1;
		last_DBC = DBC;
	}
	else if (dropped > 0) {
		DBC_packets_lost += dropped;
	}

	/* work through the DV packets */
	if (unfiltered) {
		size_t sz = (size_t)fence - (size_t)DV;
		sz -= sz % 80;
		if (sz > 0) {
			// TODO: buffered "assembly" into a DV25 frame even though we're not assembling it
		}
	}
	else {
		size_t sz = ((size_t)fence - (size_t)DV) / DV25_PACKET_SIZE;
		if (sz > 0) dv25_assemble(DV,sz);
	}

	last_DBC = DBC;
	return RAW1394_ISO_OK;
}

/* caller must take mutex */
static unsigned long long DBC_packets_lost_last_alarm = 0;
signed long long lost_packets() {
	if (DBC_packets_lost_last_alarm != DBC_packets_lost) {
		DBC_packets_lost_last_alarm = DBC_packets_lost;
		return (int)DBC_packets_lost;
	}
	return -1LL;
}

/* set the channel */
void set_channel() {
	struct iec61883_oMPR ompr;

	memset(&ompr,0,sizeof(ompr));
	if (iec61883_get_oMPR(raw1394,0xFFC0 | node_index,&ompr)) {
		fprintf(stderr,"Cannot read oMPR\n");
		return;
	}
	if (ompr.n_plugs == 0) {
		fprintf(stderr,"oMPR: no plugs?\n");
		return;
	}

	struct iec61883_oPCR opcr;
	memset(&opcr,0,sizeof(opcr));
	if (iec61883_get_oPCR0(raw1394,0xFFC0 | node_index,&opcr)) {
		fprintf(stderr,"Cannot read oPCR[0]\n");
		return;
	}

	/* apparently some cameras require that we shut off bcast & p2p, then set the info and then turn it back on */
	opcr.bcast_connection = 0;
	opcr.n_p2p_connections = 0;
	opcr.channel = 0;
	iec61883_set_oPCR0(raw1394,0xFFC0 | node_index,opcr);

	if (bcast) {
		opcr.bcast_connection = 1;
		opcr.n_p2p_connections = 0;
		opcr.data_rate = 2;
		opcr.payload = 1928/4;
	}
	else {
		opcr.bcast_connection = 0;
		opcr.n_p2p_connections = 1;
		opcr.channel = channel;
		opcr.data_rate = 2;
		opcr.payload = 1928/4;
	}

	if (bcast) {
		opcr.channel = channel;
		iec61883_set_oPCR0(raw1394,0xFFC0 | node_index,opcr);

		/* modify the oMPR where the bcast channel is */
		ompr.bcast_channel = channel;
		memset(&ompr,0,sizeof(ompr));
		if (iec61883_set_oMPR(raw1394,0xFFC0 | node_index,ompr))
			printf("Cannot write oMPR\n");

		/* set the broadcast bit mofo */
		memset(&opcr,0,sizeof(opcr));
		iec61883_get_oPCR0(raw1394,0xFFC0 | node_index,&opcr);
		opcr.bcast_connection = 1;
		opcr.channel = channel;
		iec61883_set_oPCR0(raw1394,0xFFC0 | node_index,opcr);

		/* modify the oMPR where the bcast channel is */
		ompr.bcast_channel = channel;
		memset(&ompr,0,sizeof(ompr));
		if (iec61883_set_oMPR(raw1394,0xFFC0 | node_index,ompr))
			printf("Cannot write oMPR\n");
	}
	else {
		if (iec61883_set_oPCR0(raw1394,0xFFC0 | node_index,opcr))
			printf("Cannot write oPCR[0]\n");
	}
}

int (*last_start_iso)() = NULL;
int start_iso_recv_dv25() {
	last_start_iso = start_iso_recv_dv25;
	if (iso_active) return 0;

	if (raw1394_iso_recv_init(raw1394, iso_handler_dv25, (1500/6) * 3 * 30, 8+(80*6*4), channel, recv_mode, 64) < 0) { /* up to 3 seconds */
		printf("iso_recv_init() failed\n");
		return 1;
	}

	/* we do this now this way because apparently this is the only way to force my x86_64 system to properly set up the
	 * oPCR ROM values to do the capture. in most cases it doesn't seem to set the bcast enable bit, and doesn't change
	 * the right channel. so we've gotta whack it around */
	/* Note to iec61883: fuck you and your vague semi-useful high level functions too.
	 *                   we want to manipulate the PROM directly thank you very much,
	 *                   so we're going to dig into your "private" API to get at the
	 *                   meat we really need. */
	if (!passive)
		set_channel();

	if (raw1394_iso_recv_start(raw1394,-1,-1,0) < 0) {
		printf("iso_recv_start() failed\n");
		return 1;
	}

	iso_active = 1;
	return 0;
}

int stop_iso_recv() {
	iso_active = 0;
	if (raw1394) {
		raw1394_iso_stop(raw1394);
		raw1394_iso_shutdown(raw1394);
		iec61883_cmp_disconnect(raw1394,0xFFC0 | node_index,0,raw1394_get_local_id(raw1394),-1,channel,0);
	}
	return 0;
}

#if 0
static char stdin_cmd[1024];
int stdin_cmdi=0;
void handle_stdin() {
	char c;

	while (read(0,&c,1) == 1) {
		if (c == '\n') {
			stdin_cmd[stdin_cmdi] = 0;
			stdin_cmdi = 0;

			if (!strcmp(stdin_cmd,"hold")) {
				start_iso_recv_dv25();
				mode_hold();
			}
			else if (!strcmp(stdin_cmd,"rec")) {
				start_iso_recv_dv25();
				mode_capture();
			}
			else if (!strcmp(stdin_cmd,"stop")) {
				stop_iso_recv();
				mode_hold();
			}
			else if (!strcmp(stdin_cmd,"split")) {
				close_capture();
				open_capture();
			}
			else if (!strncmp(stdin_cmd,"prefix ",7)) {
				char *p = stdin_cmd+7;
				while (*p == ' ') p++;
				char *n = strchr(p,' ');
				if (n) *n++ = 0;
				if (strlen(p) > 0 && strcmp(capture_prefix,p)) {
					memset(capture_prefix,0,sizeof(capture_prefix));
					strncpy(capture_prefix,p,sizeof(capture_prefix)-1);
					close_capture();
					open_capture();
				}
			}
			else if (!strncmp(stdin_cmd,"redirect ",9)) {
				char *p = stdin_cmd+9;
				while (*p == ' ') p++;
				char *n = strchr(p,' ');
				if (n) *n++ = 0;
				if (strlen(p) > 0) {
					/* this is a path where to redirect */
					printf("Moving to: %s\n",p);
					if (chdir(p) == 0) {
						/* now ... */
						{
							char cwd[256];
							getcwd(cwd,sizeof(cwd));
							printf("Now in: %s\n",cwd);

							/* and clear the now capturing */
							{
								char tmp[512];
								sprintf(tmp,"%s/now-capturing",cwd);
								unlink(tmp);
							}
						}
						/* take effect */
						close_capture();
						open_capture();
					}
					else {
						printf(" ! Failed to move!\n");
					}
				}
			}
			else
				printf("What does that mean?\n");
		}
		else {
			if (stdin_cmdi < 1023)
				stdin_cmd[stdin_cmdi++] = c;
		}
	}
}
#endif

void check_low_disk_space() {
	struct statfs fs;

	if (capture_fd < 0)
		return;
	if (fstatfs(capture_fd,&fs) < 0)
		return;

	long long left = (long long)fs.f_bsize * (long long)fs.f_bfree;

	/* the host app running us is supposed to be watching free disk space.
	 * therefore, if free disk space falls below 500MB immediately transition to holding pattern mode
	 * (500MB leaves room for one 1-minute holding pattern while the admin fixes things up therefore
	 * assuring that in the event of low disk space, the admin has plenty of time to plug in a new drive
	 * and redirect capture to that drive. */
	if (left < (500LL*1024LL*1024LL)) {
		if (capture_fd >= 0 && !holding_pattern) {
			printf("Low disk space! Transitioning capture to holding pattern.\n");
			printf("  You (the admin) have 1 minute to plug in another hard disk\n");
			printf("  and redirect capture to that disk before you start to lose\n");
			printf("  capture data.\n");
			mode_hold();
		}
	}
}

void *capture_thread_proc(void *arg) {
	while (!capture_thread_die) {
		int count,ret=0;

		/* loop for exactly the number of packets it takes for one frame to complete */
		for (count=0,ret=0;count < (1500/6) && ret == 0;count++) {
			struct timeval tv;
			tv.tv_usec = 100000;
			tv.tv_sec = 0;
			fd_set fd;
			FD_ZERO(&fd);
			FD_SET(raw1394_get_fd(raw1394),&fd);
			if (select(raw1394_get_fd(raw1394)+1,&fd,NULL,NULL,&tv) == 1) {
				if (capture_mutex_lock()) {
					if (capture_buffer_time[capture_buffer_last_frame_written] < 0)
						capture_buffer_time[capture_buffer_last_frame_written] = clocksync_master();

					ret = raw1394_loop_iterate(raw1394);
					assert(capture_mutex_unlock());
				}
			}
			else {
				break;
			}
		}

		if (errno == EAGAIN)
			ret = 0;	/* expected: non-blocking */
		else if (errno == ENOTTY)
			ret = 0;	/* what the fuck? ignore */
		else if (ret != 0)
			fprintf(stderr,"ret = %d errno=%d %s\n",ret,errno,strerror(errno));
	}

	return NULL;
}

int start_capture_thread() {
	assert(capture_thread == 0);
	capture_thread_die = 0;
	if (pthread_create(&capture_thread,NULL,capture_thread_proc,NULL))
		return -1;
	if (pthread_detach(capture_thread))
		return -1;
	printf("Capture thread started\n");
	return 0;
}

void stop_capture_thread() {
	if (capture_thread != 0) {
		printf("Capture thread stopped\n");
		capture_thread_die = 1;
		pthread_join(capture_thread,NULL);
		capture_thread = 0;
	}
}

int main(int argc,char **argv) {
	time_t now_t=0;

	if (parse_args(argc,argv))
		return 1;

	if (find_device_by_guid()) {
		fprintf(stderr,"Device with GUID %llx not found or not AV/C device\n",(long long)guid);
		return 1;
	}
	if (init_raw1394()) {
		fprintf(stderr,"Cannot open 1394\n");
		return 1;
	}
	if (raw1394_set_port(raw1394,host_index) < 0) {
		fprintf(stderr,"Cannot set port\n");
		return 1;
	}

	signal(SIGINT, sigma);
	signal(SIGQUIT,sigma);
	signal(SIGTERM,sigma);

	dv25_discard = 1;
	init_dv25_assembly();
	if (init_capture_buffer(30*4,12000*10*4)) { /* 4 second video buffer of NTSC DV */
		fprintf(stderr,"Cannot init capture buffer\n");
		return 1;
	}

	if (start_iso_recv_dv25()) {
		fprintf(stderr,"Cannot start ISO reception\n");
		return 1;
	}

	/* run as real-time process to ensure no dropping frames */
	struct sched_param p;
	memset(&p,0,sizeof(p));
	p.sched_priority = 99;

	if (sched_setscheduler(0,SCHED_RR,&p) < 0)
		if (nice(-20) < 0)
			fprintf(stderr,"Warning, cannot make myself realtime\n");

#if 0
	{
		int x = fcntl(0,F_GETFL);
		fcntl(0,F_SETFL,x | O_NONBLOCK);
	}
#endif

	if (open_capture())
		return 1;

	pthread_mutex_init(&capture_mutex,NULL);

	/* here we go! */
	if (start_capture_thread()) {
		fprintf(stderr,"Cannot start capture thread\n");
		return 1;
	}

	printf("Autosense=%d\n",autosense);

	int ret = 0,wrote;
	while (ret == 0 && !global_death) {
		now_t = time(NULL);

		/* only for testing */
//		clocksync_master();

		wrote = 0;
		if (capture_mutex_lock()) {
			/* write a maximum of 1/4th the buffer, so the capture thread may have a chance to fill another part */
			wrote = empty_capture_buffer(capture_buffer_size/2);

			signed long long lost = lost_packets();
			if (lost > 0) {
				if (isatty(1)) bell();
				printf("Warning: more lost packets, %lld since capture\n",lost);
			}

			/* check other conditions */
			check_low_disk_space();

#if 0
			/* parent process/user commands entered at STDIN? */
			handle_stdin();
#endif

			/* done. back to you, capture thread */
			assert(capture_mutex_unlock());
		}

		/* wait */
		if (wrote == 0) {
			usleep(1000000/30);
		}
	}

	stop_capture_thread();
	pthread_mutex_destroy(&capture_mutex);

	unlink("now-capturing");
	free_capture_buffer();
	close_capture();
	stop_iso_recv();
	free_raw1394();
	return 0;
}

