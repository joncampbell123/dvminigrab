
#include <stdint.h>

#define DVMINIGRAB_STRUCT_VERSION		0x1111

typedef struct {
	char		sig[4];			/* 0x000 = 'DV25' */
	uint32_t	version;		/* 0x004 = version */
	uint32_t	frame_size;		/* 0x008 = 12000*10 or 12000*12 */
	uint32_t	frames;			/* 0x00C = number of frames */
	uint32_t	output;			/* 0x010 = output frame number */
} __attribute__((aligned(4))) dv_capture_shmem_header;

