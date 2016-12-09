/*
 * Copyright (C) 2016 Hisilicon Limited, All Rights Reserved.
 * Author: Zhichang Yuan <yuanzhichang@hisilicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_EXTIO_H
#define __LINUX_EXTIO_H

#include <linux/fwnode.h>
#include <linux/ioport.h>

/*
 * This macro is specific for some devices which have no cpu IO space.
 * But in bus/device domain, IO addresses are used to access the devices.
 * So, using this macro to register linux PIO for those devices, then kernel
 * can differentiate from normal MMIO registering.
 */
#define IO_RANGE_IOEXT (resource_size_t)(-1ull)

/* the total legacy IO size is 8K. */
#define LEGACY_IO_TSIZE	0x2000
/*
 * The maximal IO size for each leagcy bus.
 * As all the children IO ranges are consecutive in indirectIO bus virtual IO
 * window, 0x400 should be sufficient for all peripherals under one bus.
 * Which means that maximum 8 indirectIO buses are supported.
 */
#define LEGACY_BUS_IO_SIZE	0x400

struct extio_ops {
	u64 (*pfin)(void *devobj, unsigned long ptaddr,	size_t dlen);
	void (*pfout)(void *devobj, unsigned long ptaddr, u32 outval,
					size_t dlen);
	u64 (*pfins)(void *devobj, unsigned long ptaddr, void *inbuf,
				size_t dlen, unsigned int count);
	void (*pfouts)(void *devobj, unsigned long ptaddr,
				const void *outbuf, size_t dlen,
				unsigned int count);
	void *devpara;
};

/* only used when the devices are scanning */
struct extio_alloc_seg {
	struct list_head node;
	resource_size_t start; /* pio */
	resource_size_t end;
	resource_size_t offset;
	struct extio_bus_res *bus_res;
};
#define to_extio_alloc(x) container_of((x), struct extio_alloc_seg, node)

/* will be linked into resource tree after the device is created. */
struct extio_dev_res {
	struct resource res;
	resource_size_t offset; /* linux pio - dev start io */

	struct extio_ops *devops;
	/* point to the corresponding child for freeing resource */
	struct device *dev;
	struct extio_alloc_seg *pseg;
};
#define to_extio_dev_range(x) container_of((x), struct extio_dev_res, res)

struct extio_bus_res {
	struct fwnode_handle *fwnode; /* search key */

	struct resource iowin;
	struct extio_ops ops;
	/* list of linux pio segments allocated */
	spinlock_t child_lock;	
	struct list_head child_head;
};
#define to_extio_range(x) container_of((x), struct extio_bus_res, iowin)

struct extio_root_res {
	void *pio_node;
	struct resource root_res;
};

/*extern struct extio_range *extio_ent;*/
extern struct resource *extio_res_head;

/* add CONFIG_PCI to be compatible */
#ifdef CONFIG_PCI
extern int pci_register_io_range(phys_addr_t addr, resource_size_t size);
extern unsigned long pci_address_to_pio(phys_addr_t addr);
extern phys_addr_t pci_pio_to_address(unsigned long pio);
#else
static inline unsigned long pci_address_to_pio(phys_addr_t addr) { return -1; }
#endif

#if defined(PCI_IOBASE) && defined(CONFIG_INDIRECT_PIO)
extern int register_bus_extio_range(struct fwnode_handle *fwnode,
			struct extio_bus_res *range);
extern unsigned long extio_translate(struct fwnode_handle *node,
		unsigned long dev_io);

extern u8 extio_inb(unsigned long addr);
extern u16 extio_inw(unsigned long addr);
extern u32 extio_inl(unsigned long addr);
extern void extio_outb(u8 value, unsigned long addr);
extern void extio_outw(u16 value, unsigned long addr);
extern void extio_outl(u32 value, unsigned long addr);
extern void extio_insb(unsigned long addr, void *buffer, unsigned int count);
extern void extio_insw(unsigned long addr, void *buffer, unsigned int count);
extern void extio_insl(unsigned long addr, void *buffer, unsigned int count);
extern void extio_outsb(unsigned long addr, const void *buffer,
		unsigned int count);
extern void extio_outsw(unsigned long addr, const void *buffer,
		unsigned int count);
extern void extio_outsl(unsigned long addr, const void *buffer,
		unsigned int count);
#else
static inline int register_bus_extio_range(struct fwnode_handle *fwnode,
			struct extio_bus_res *range)
{
	return -1;
}
static inline unsigned long extio_translate(struct fwnode_handle *node,
		resource_size_t dev_io, resource_size_t size)
{
	return -1;
}
#endif

#endif /* __LINUX_EXTIO_H */
