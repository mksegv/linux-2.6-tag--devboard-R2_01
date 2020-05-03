/*
 * Physical mapping layer for MTD using the Axis partitiontable format
 *
 * Copyright (c) 2001, 2002 Axis Communications AB
 *
 * This file is under the GPL.
 *
 * First partition is always sector 0 regardless of if we find a partitiontable
 * or not. In the start of the next sector, there can be a partitiontable that
 * tells us what other partitions to define. If there isn't, we use a default
 * partition split defined below.
 *
 * $Log: axisflashmap.c,v $
 * Revision 1.4  2005/07/29 17:44:04  bjornw
 * Dont crash if there is no romfs at all
 *
 * Revision 1.3  2005/06/13 19:55:38  bjornw
 * Do cfi_probe_init before probing cfi. I dont know why this is still necessary but it is.
 *
 * Revision 1.2  2005/06/07 16:14:30  bjornw
 * 2.6 version
 *
 * Revision 1.1  2005/06/07 13:33:19  bjornw
 * Argus for linux 2.6 initial temptations
 *
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/config.h>

#include <linux/mtd/concat.h>
#include <linux/mtd/map.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/mtdram.h>

#include <asm/arch/axisflashmap.h>
#include <asm/arch/argus_addr.h>

#define FLASH_UNCACHED_ADDR         EXT0_NOCACHE_V
#define FLASH_CACHED_ADDR           EXT0_CACHE_V
#define CONFIG_ETRAX_PTABLE_SECTOR  65536

/*
 * WINDOW_SIZE is the total size where the flash chips may be mapped.
 * MTD probes should find all devices there and it does not matter
 * if there are unmapped gaps or aliases (mirrors of flash devices).
 * The MTD probes will ignore them.
 */

#define WINDOW_SIZE  (32 * 1024 * 1024)

/* Initialize, because these are set BEFORE the BSS is cleared and
 * so should not end up in BSS themselves...
 */

unsigned long romfs_start = 42, romfs_length = 42,
	romfs_in_flash = 42; /* set in head-armv.S */

/* The master mtd for the entire flash. */
struct mtd_info* axisflash_mtd = NULL;

#if CONFIG_ARGUS_FLASH_BUSWIDTH==1
#define flash_data __u8
#elif CONFIG_ARGUS_FLASH_BUSWIDTH==2
#define flash_data __u16
#elif CONFIG_ARGUS_FLASH_BUSWIDTH==4
#define flash_data __u32
#endif

/* 
 * Map driver
 *
 * We run into tricky coherence situations if we mix cached with uncached
 * accesses to we use the uncached version here.
 */

static map_word flash_read(struct map_info *map, unsigned long ofs)
{
	map_word tmp;
	tmp.x[0] = *(flash_data *)(FLASH_UNCACHED_ADDR + ofs);
	return tmp;
}

static void flash_copy_from(struct map_info *map, void *to,
			    unsigned long from, ssize_t len)
{
	memcpy(to, (void *)(FLASH_UNCACHED_ADDR + from), len);
}

static void flash_write(struct map_info *map, map_word d, unsigned long adr)
{
	*(flash_data *)(FLASH_UNCACHED_ADDR + adr) = (flash_data)d.x[0];
}

// TODO: set .virt and .cached to let MTD handle the accesses itself, we don't
// really need to go through any advanced mappings here. Do it like this for
// now though.

static struct map_info axis_map = {
	name: "Axis flash",
	size: WINDOW_SIZE,
	bankwidth: CONFIG_ARGUS_FLASH_BUSWIDTH,
	read: flash_read,
	copy_from: flash_copy_from,
	write: flash_write
};

/* If no partition-table was found, we use this default-set.
 */

#define MAX_PARTITIONS         7  
#define NUM_DEFAULT_PARTITIONS 3

/* Default flash size is 2MB. CONFIG_ETRAX_PTABLE_SECTOR is most likely the
 * size of one flash block and "filesystem"-partition needs 5 blocks to be able
 * to use JFFS.
 */
static struct mtd_partition axis_default_partitions[NUM_DEFAULT_PARTITIONS] = {
	{
		name: "boot firmware",
		size: CONFIG_ETRAX_PTABLE_SECTOR,
		offset: 0
	},
	{
		name: "kernel",
		size: 0x200000 - (6 * CONFIG_ETRAX_PTABLE_SECTOR),
		offset: CONFIG_ETRAX_PTABLE_SECTOR
	},
	{
		name: "filesystem",
		size: 5 * CONFIG_ETRAX_PTABLE_SECTOR,
		offset: 0x200000 - (5 * CONFIG_ETRAX_PTABLE_SECTOR)
	}
};

static struct mtd_partition axis_partitions[MAX_PARTITIONS] = {
	{
		name: "part0",
		size: CONFIG_ETRAX_PTABLE_SECTOR,
		offset: 0
	},
	{
		name: "part1",
		size: 0,
		offset: 0
	},
	{
		name: "part2",
		size: 0,
		offset: 0
	},
	{
		name: "part3",
		size: 0,
		offset: 0
	},
	{
		name: "part4",
		size: 0,
		offset: 0
	},
	{
		name: "part5",
		size: 0,
		offset: 0
	},
	{
		name: "part6",
		size: 0,
		offset: 0
	},
};

/* CFI-scan the flash, and if there was a chip, read the partition-table
 * and register the partitions with MTD.
 */

static int __init
init_axis_flash(void)
{
	struct mtd_info *mymtd = NULL;
	int err = 0;
	int pidx = 0;
	struct partitiontable_head *ptable_head = NULL;
	struct partitiontable_entry *ptable;
	int use_default_ptable = 1; /* Until proven otherwise */
	const char *pmsg = "  /dev/flash%d at 0x%x, size 0x%x\n";

	printk(KERN_NOTICE "Axis flash mapping: %x at %lx\n",
	       WINDOW_SIZE, FLASH_CACHED_ADDR);

#ifdef CONFIG_MTD_CFI
	if (!mymtd) {
		// The cfi module is linked after this in the linker
		// so its init function has not been called yet.
		cfi_probe_init();
		printk("trying to cfi_probe...\n");
		mymtd = (struct mtd_info *)do_map_probe("cfi_probe", &axis_map);
	}
#endif

	if(mymtd) {
		printk(KERN_INFO "%s: 0x%08x bytes of flash memory.\n",
		       mymtd->name, mymtd->size);
		axisflash_mtd = mymtd;
		mymtd->owner = THIS_MODULE;
		ptable_head = (struct partitiontable_head *)
			(FLASH_CACHED_ADDR + CONFIG_ETRAX_PTABLE_SECTOR + 
			 PARTITION_TABLE_OFFSET);
	}
	pidx++;  /* first partition is always set to the default */

	if (ptable_head && (ptable_head->magic == PARTITION_TABLE_MAGIC)
	    && (ptable_head->size <
		(MAX_PARTITIONS * sizeof(struct partitiontable_entry) +
		PARTITIONTABLE_END_MARKER_SIZE))
	    && (*(unsigned long*)((void*)ptable_head + sizeof(*ptable_head) +
				  ptable_head->size -
				  PARTITIONTABLE_END_MARKER_SIZE)
		== PARTITIONTABLE_END_MARKER)) {
		/* Looks like a start, sane length and end of a
		 * partition table, lets check csum etc.
		 */
		int ptable_ok = 0;
		struct partitiontable_entry *max_addr =
			(struct partitiontable_entry *)
			((unsigned long)ptable_head + sizeof(*ptable_head) +
			 ptable_head->size);
		unsigned long offset = CONFIG_ETRAX_PTABLE_SECTOR;
		unsigned char *p;
		unsigned long csum = 0;
                
		ptable = (struct partitiontable_entry *)
			((unsigned long)ptable_head + sizeof(*ptable_head));

		/* Lets be PARANOID, and check the checksum. */
		p = (unsigned char*) ptable;

		while (p <= (unsigned char*)max_addr) {
			csum += *p++;
			csum += *p++;
			csum += *p++;
			csum += *p++;
		}
		/* printk("  total csum: 0x%08X 0x%08X\n",
		   csum, ptable_head->checksum); */
		ptable_ok = (csum == ptable_head->checksum);

		/* Read the entries and use/show the info.  */
		printk(" Found %s partition table at 0x%08lX-0x%08lX.\n",
		       (ptable_ok ? "valid" : "invalid"),
		       (unsigned long)ptable_head,
		       (unsigned long)max_addr);

		/* We have found a working bootblock.  Now read the
		   partition table.  Scan the table.  It ends when
		   there is 0xffffffff, that is, empty flash.  */
		
		while (ptable_ok
		       && ptable->offset != 0xffffffff
		       && ptable < max_addr
		       && pidx < MAX_PARTITIONS) {

			axis_partitions[pidx].offset = offset + ptable->offset;
			axis_partitions[pidx].size = ptable->size;

			printk(pmsg, pidx, axis_partitions[pidx].offset,
			       axis_partitions[pidx].size);
			pidx++;
			ptable++;
		}
		use_default_ptable = !ptable_ok;
	}

	if (romfs_in_flash) {
		/* Add an overlapping device for the root partition (romfs). */

		axis_partitions[pidx].name = "romfs";
		axis_partitions[pidx].size = romfs_length;
		axis_partitions[pidx].offset = romfs_start - 0x80000000;
		axis_partitions[pidx].mask_flags |= MTD_WRITEABLE;

		printk(KERN_INFO 
		       " Adding readonly flash partition for romfs image:\n");
		printk(pmsg, pidx, axis_partitions[pidx].offset,
		       axis_partitions[pidx].size);
		pidx++;
	}

	if(mymtd) {
		if (use_default_ptable) {
			printk(KERN_INFO " Using default partition table.\n");
			err = add_mtd_partitions(mymtd, axis_default_partitions,
						 NUM_DEFAULT_PARTITIONS);
		} else {
			err = add_mtd_partitions(mymtd, axis_partitions, pidx);
		}
		
		if (err) {
			panic("axisflashmap could not add MTD partitions!\n");
		}
	}

	if (!romfs_in_flash && romfs_start != 42) {
		/* Create an RAM device for the root partition (romfs). */

#if !defined(CONFIG_MTD_MTDRAM) || (CONFIG_MTDRAM_TOTAL_SIZE != 0) || (CONFIG_MTDRAM_ABS_POS != 0)
		/* No use trying to boot this kernel from RAM. Panic! */
		printk(KERN_EMERG "axisflashmap: Cannot create an MTD RAM "
		       "device due to kernel (mis)configuration!\n");
		panic("This kernel cannot boot from RAM!\n");
#else
		struct mtd_info *mtd_ram;

		mtd_ram = (struct mtd_info *)kmalloc(sizeof(struct mtd_info),
						     GFP_KERNEL);
		if (!mtd_ram) {
			panic("axisflashmap couldn't allocate memory for "
			      "mtd_info!\n");
		}

		printk(KERN_INFO " Adding RAM partition for romfs image (magic 0x%x):\n",
		       *(long*)(romfs_start));
		printk(pmsg, pidx, romfs_start, romfs_length);

		err = mtdram_init_device(mtd_ram, (void*)romfs_start, 
		                         romfs_length, "romfs");
		if (err) {
			panic("axisflashmap could not initialize MTD RAM "
			      "device!\n");
		}
#endif
	}
	return err;
}

/* This adds the above to the kernels init-call chain */

module_init(init_axis_flash);

EXPORT_SYMBOL(axisflash_mtd);
