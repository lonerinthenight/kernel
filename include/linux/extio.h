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

#ifndef PCI_IOBASE
#define PCI_IOBASE ((void __iomem *)0)
#endif

#ifndef INDIRECT_MAX_IO
#define INDIRECT_MAX_IO		0
#endif

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

#endif /* __LINUX_EXTIO_H */
