/*
 * Copyright (c) 2002-2005 Axis Communications AB
 *
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/page.h>
#include <asm/io.h>
#include <asm/errno.h>

#include <asm/arch/ciobuf.h>

#define D(x)

int
ciobuf_map(struct ciobuf **bufp, unsigned long uaddr, size_t length, int rw)
{
	struct ciobuf *ciobuf = NULL;
	int i;
	int mapped = 0;

	D(printk("%s: uaddr=0x%08lx length=%d rw=%d\n", __FUNCTION__, uaddr,
		 length, rw));

	/* Allocate memory for struct ciobuf. */
	if ((ciobuf = kmalloc(sizeof *ciobuf, GFP_KERNEL)) == NULL) {
		D(printk("%s: kmalloc of ciobuf\n", __FUNCTION__));
		goto out;
	}

	ciobuf->rw = rw;
	ciobuf->length = length;
	ciobuf->offset = uaddr & ~PAGE_MASK;
	ciobuf->nbr_pages =
		(ciobuf->offset + ciobuf->length + PAGE_SIZE - 1) >> PAGE_SHIFT;

	D(printk("%s: nbr_pages=%d offset=%d\n", __FUNCTION__,
		 ciobuf->nbr_pages, ciobuf->offset));

	if (ciobuf->nbr_pages == 0)
		printk("Mapping of zero pages\n");

	/* Allocate memory for struct page array. */
	ciobuf->pages = kmalloc(sizeof(struct page *)*ciobuf->nbr_pages,
				GFP_KERNEL);
	if (ciobuf->pages == NULL) {
		D(printk("%s: kmalloc of page[]\n", __FUNCTION__));
		goto out;
	}

	/* Try to fault in all of the necessary pages */
	down_read(&current->mm->mmap_sem);
	mapped = get_user_pages(current,
				current->mm,
				uaddr,
				ciobuf->nbr_pages,
				ciobuf->rw == READ,
				0, /* don't force */
				ciobuf->pages,
				NULL);
	up_read(&current->mm->mmap_sem);

	D(printk("%s: Mapped %d/%d\n",__FUNCTION__,mapped,ciobuf->nbr_pages));

	// Errors and too few pages mapped should return here.
	// Notice that get_user_pages can return incomplete mappings if
	// memory is scarce (mapped can be < 0 or > 0 and < nbr_pages).
	
	if (mapped < ciobuf->nbr_pages) {
		goto out;
	}

	// TODO: flush_dcache_range ? 

	*bufp = ciobuf;

	return 0;

out:
	// Release the pages we actually got, if any
	for(i=0; i < mapped; i++) {
		put_page(ciobuf->pages[i]);
	}
	if (ciobuf)
		kfree(ciobuf->pages);
	kfree(ciobuf);
	*bufp = NULL;
	return -ENOMEM;
}

// Does NOT take care of dirtying used pages. Use mark_dirty_ciobuf for
// that (which can take a sub-range as well).

void
ciobuf_unmap(struct ciobuf *ciobuf)
{
	int i;
	if (ciobuf) {
		for(i = 0; i < ciobuf->nbr_pages; i++) {
			struct page *page = ciobuf->pages[i];
			put_page(page);
		}
		kfree(ciobuf->pages);
		kfree(ciobuf);
	}
}

/*
 * Mark the selected number of bytes in a ciobuf as dirty 
 *
 * Must be called from process context - set_page_dirty() takes VFS locks.
 */

void 
mark_dirty_ciobuf(struct ciobuf *iobuf, int bytes)
{
	int index, offset, remaining;
	struct page *page;
	
	index = iobuf->offset >> PAGE_SHIFT;
	offset = iobuf->offset & ~PAGE_MASK;
	remaining = bytes;
	if (remaining > iobuf->length)
		remaining = iobuf->length;
	
	while (remaining > 0 && index < iobuf->nbr_pages) {
		page = iobuf->pages[index];
		
		if (!PageReserved(page))
			set_page_dirty(page);

		remaining -= (PAGE_SIZE - offset);
		offset = 0;
		index++;
	}
}

void
memcpy_to_ciobuf(struct ciobuf *kbuf, unsigned int offset,
                 char *source, unsigned int length)
{
        unsigned int left_in_page, to_copy;
        unsigned int p;
	
        // stay safe

        if(offset + length > kbuf->length) {
                printk("ciobuf: out of bounds offset %d len %d (size %d)\n",
                       offset, length, kbuf->length);
                return;
        }

        // which page is offset in ? 

        left_in_page = PAGE_SIZE - kbuf->offset;
        
        if(offset >= left_in_page) {
                /* at least in page 1 */
                p = 1 + ((offset - left_in_page) >> PAGE_SHIFT);
                /* what's the new offset within the page */
                offset = (offset - left_in_page) & ~PAGE_MASK;
                left_in_page = PAGE_SIZE - offset;
        } else {
                p = 0;
                left_in_page -= offset;
                offset += kbuf->offset;
        }

        // copy
        
        while(length > 0) {
                to_copy = length > left_in_page ? left_in_page : length;
                //printk("ciobuf copy to page %d (0x%p), offset %d, %d bytes from 0x%p\n",
		//       p, page_address(kbuf->pages[p]), offset, to_copy, source);
                memcpy(page_address(kbuf->pages[p]) + offset,
                       source,
                       to_copy);
                source += to_copy;
                length -= to_copy;
                p++;
                /* after the first page, it's all full pages */
                offset = 0;
                left_in_page = PAGE_SIZE;
        }

}

EXPORT_SYMBOL(mark_dirty_ciobuf);
EXPORT_SYMBOL(memcpy_to_ciobuf);
EXPORT_SYMBOL(ciobuf_map);
EXPORT_SYMBOL(ciobuf_unmap);
