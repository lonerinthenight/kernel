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

/*
 * This macro is specific for some devices which have no cpu IO space.
 * But in bus/device domain, IO addresses are used to access the devices.
 * So, using this macro to register linux PIO for those devices, then kernel
 * can differentiate from normal MMIO registering.
 */
#define IO_RANGE_IOEXT (resource_size_t)(-1ull)

/* the total legacy IO size is 16K. */
#define LEGACY_IO_TSIZE	0x4000
/* the maximal IO size for each leagcy bus. 12bits */
#define LEGACY_BUS_IO_SIZE	0x800

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

struct extio_range {
	struct fwnode_handle *fwnode; /* search key */

	struct resource iowin;
	resource_size_t offset; /* linux pio - dev start io */

	struct extio_ops ops;
};

struct extio_windows {
	void *pio_node;
	struct resource root_res;
};

extern struct extio_range *extio_ent;

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
			struct extio_range *range);
extern unsigned long extio_translate(struct fwnode_handle *node,
		unsigned long dev_io);
#else
static inline int register_bus_extio_range(struct fwnode_handle *fwnode,
			struct extio_range *range)
{
	return -1;
}
static inline unsigned long extio_translate(struct fwnode_handle *node,
		unsigned long dev_io)
{
	return -1;
}
#endif

#if defined(PCI_IOBASE) && defined(CONFIG_INDIRECT_PIO)
#define BUILD_EXTIO(bw, type)						\
type extio_in##bw(unsigned long addr)					\
{									\
	if (!extio_ent || extio_ent->iowin.start > addr ||		\
			extio_ent->iowin.end < addr)			\
		return read##bw(PCI_IOBASE + addr);			\
	return extio_ent->ops.pfin ?					\
		extio_ent->ops.pfin(extio_ent->ops.devpara,		\
			addr - extio_ent->offset, sizeof(type)) : -1;	\
}									\
									\
void extio_out##bw(type value, unsigned long addr)			\
{									\
	if (!extio_ent || extio_ent->iowin.start > addr ||		\
			extio_ent->iowin.end < addr)			\
		write##bw(value, PCI_IOBASE + addr);			\
	else								\
		if (extio_ent->ops.pfout)				\
			extio_ent->ops.pfout(extio_ent->ops.devpara,	\
				addr - extio_ent->offset, value, sizeof(type));	\
}									\
									\
void extio_ins##bw(unsigned long addr, void *buffer, unsigned int count)	\
{									\
	if (!extio_ent || extio_ent->iowin.start > addr ||		\
			extio_ent->iowin.end < addr)			\
		reads##bw(PCI_IOBASE + addr, buffer, count);		\
	else								\
		if (extio_ent->ops.pfins)				\
			extio_ent->ops.pfins(extio_ent->ops.devpara,	\
				addr - extio_ent->offset, buffer,	\
				sizeof(type), count);			\
}									\
									\
void extio_outs##bw(unsigned long addr, const void *buffer, unsigned int count)	\
{									\
	if (!extio_ent || extio_ent->iowin.start > addr ||		\
			extio_ent->iowin.end < addr)			\
		writes##bw(PCI_IOBASE + addr, buffer, count);		\
	else								\
		if (extio_ent->ops.pfouts)				\
			extio_ent->ops.pfouts(extio_ent->ops.devpara,	\
				addr - extio_ent->offset, buffer,	\
				sizeof(type), count);			\
}
#endif

#endif /* __LINUX_EXTIO_H */
