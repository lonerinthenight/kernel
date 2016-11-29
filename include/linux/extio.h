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

#include <linux/pci.h>

#if defined(CONFIG_PCI) && defined(CONFIG_INDIRECT_PIO)
#define EXTIO_LIMIT	PCIBIOS_MIN_IO
#elif defined(CONFIG_INDIRECT_PIO)
#define EXTIO_LIMIT	0x1000
#else
#define EXTIO_LIMIT	0x00
#endif

#define IO_RANGE_IOEXT (resource_size_t)(-1ull)


struct extio_ops {
	unsigned long start; /* inclusive, sys io addr */
	unsigned long end; /* inclusive, sys io addr */

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

extern struct extio_ops *extio_ops_node;

struct extio_win {
	struct resource *res;
	resource_size_t offset;
	resource_size_t iostart;
	struct resource __res;
};

struct extio_range {
	struct list_head list;
	struct fwnode_handle *fwnode; /* search key */

	struct extio_win io_host;
	struct extio_ops *ops;
};

#ifdef CONFIG_INDIRECT_PIO
extern unsigned long extio_translate(struct fwnode_handle *node,
		unsigned long dev_io);
extern struct extio_range *register_extio_ranges(struct fwnode_handle *fwnode,
		resource_size_t size, resource_size_t min_bus_io);

extern struct extio_range * extio_getrange_byaddr(unsigned long addr);
#endif

#define BUILD_EXTIO(bw, type)						\
type in##bw(unsigned long addr)						\
{									\
	if (addr < EXTIO_LIMIT) {					\
		struct extio_range *range;				\
									\
		range = extio_getrange_byaddr(addr);			\
		if (!range || !range->ops->pfin)			\
			return -1;					\
		return range->ops->pfin(range->ops->devpara,		\
				addr - range->io_host.offset,		\
				sizeof(type));				\
	}								\
	return readb(PCI_IOBASE + addr);				\
}									\
									\
void out##bw(type value, unsigned long addr)				\
{									\
	if (addr < EXTIO_LIMIT) {					\
		struct extio_range *range;				\
									\
		range = extio_getrange_byaddr(addr);			\
		if (range->ops->pfout)					\
			range->ops->pfout(range->ops->devpara,		\
				addr - range->io_host.offset,		\
				value, sizeof(type));			\
	}								\
	write##bw(value, PCI_IOBASE + addr);				\
}									\
									\
void ins##bw(unsigned long addr, void *buf, unsigned int count)		\
{									\
	if (addr < EXTIO_LIMIT) {					\
		struct extio_range *range;				\
									\
		range = extio_getrange_byaddr(addr);			\
		if (range->ops->pfins)					\
			range->ops->pfins(range->ops->devpara,		\
				addr - range->io_host.offset,		\
				buf, sizeof(type), count);		\
	}								\
	reads##bw(PCI_IOBASE + addr, buf, count);			\
}									\
									\
void outs##bw(unsigned long addr, const void *buf, unsigned int count)	\
{									\
	if (addr < EXTIO_LIMIT) {					\
		struct extio_range *range;				\
									\
		range = extio_getrange_byaddr(addr);			\
		if (range->ops->pfouts)					\
			range->ops->pfouts(range->ops->devpara, 	\
				addr - range->io_host.offset,		\
				buf, sizeof(type), count);		\
	}								\
	writes##bw(PCI_IOBASE + addr, buf, count);			\
}


#if 0
#define BUILD_EXTIO(bw, type)						\
type in##bw(unsigned long addr)						\
{									\
	if (!extio_ops_node || extio_ops_node->start > addr ||		\
			extio_ops_node->end < addr)			\
		return read##bw(PCI_IOBASE + addr);			\
	return extio_ops_node->pfin ?					\
		extio_ops_node->pfin(extio_ops_node->devpara,		\
			addr, sizeof(type)) : -1;			\
}									\
									\
void out##bw(type value, unsigned long addr)				\
{									\
	if (!extio_ops_node || extio_ops_node->start > addr ||		\
			extio_ops_node->end < addr)			\
		write##bw(value, PCI_IOBASE + addr);			\
	else								\
		if (extio_ops_node->pfout)				\
			extio_ops_node->pfout(extio_ops_node->devpara,	\
				addr, value, sizeof(type));		\
}									\
									\
void ins##bw(unsigned long addr, void *buffer, unsigned int count)	\
{									\
	if (!extio_ops_node || extio_ops_node->start > addr ||		\
			extio_ops_node->end < addr)			\
		reads##bw(PCI_IOBASE + addr, buffer, count);		\
	else								\
		if (extio_ops_node->pfins)				\
			extio_ops_node->pfins(extio_ops_node->devpara,	\
				addr, buffer, sizeof(type), count);	\
}									\
									\
void outs##bw(unsigned long addr, const void *buffer, unsigned int count)	\
{									\
	if (!extio_ops_node || extio_ops_node->start > addr ||		\
			extio_ops_node->end < addr)			\
		writes##bw(PCI_IOBASE + addr, buffer, count);		\
	else								\
		if (extio_ops_node->pfouts)				\
			extio_ops_node->pfouts(extio_ops_node->devpara,	\
				addr, buffer, sizeof(type), count);	\
}
#endif

#endif /* __LINUX_EXTIO_H */
