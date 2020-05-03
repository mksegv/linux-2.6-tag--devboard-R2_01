#ifndef SSI_H
#define SSI_H

#define SSI_SKIP_START 0
#define SSI_SKIP_END   1
#define SSI_GET_BUFFER 2

// Request to pass through the ioctl interface to read data with timestamps

struct ssi_request {
	char *buf;
	size_t length;
	struct timeval timestamp;
};

#endif

