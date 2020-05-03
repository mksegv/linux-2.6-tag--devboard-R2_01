/*
 * Copyright (c) 2002-2005 Axis Communications AB
 *
 */

#ifndef __CIOBUF_H__
#define __CIOBUF_H__

struct ciobuf {
	struct page **pages;    /* Pointer to array of page pointers. */
	unsigned int nbr_pages; /* Number of pages in array. */
	size_t length;          /* Total length in bytes. */
	size_t offset;          /* Offset from start of first page in bytes. */
	int rw;                 /* Userspace will READ or WRITE to pages. */
};

int ciobuf_map(struct ciobuf **bufp,
	       unsigned long uaddr,
	       size_t length,
	       int rw);

void ciobuf_unmap(struct ciobuf *ciobuf);

void mark_dirty_ciobuf(struct ciobuf *iobuf, int bytes);

void memcpy_to_ciobuf(struct ciobuf *kbuf, unsigned int offset,
		      char *source, unsigned int length);

#endif
