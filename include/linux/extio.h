/*
 * Copyright (C) 2016 Hisilicon Limited, All Rights Reserved.
 * Author: Zhichang Yuan <yuanzhichang@hisilicon.com>
 * Author: Zou Rongrong <@huawei.com>
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


#include <linux/ioport.h>


typedef u64 (*inhook)(void *devobj, unsigned long ptaddr, void *inbuf,
				size_t dlen, unsigned int count);
typedef void (*outhook)(void *devobj, unsigned long ptaddr,
				const void *outbuf, size_t dlen,
				unsigned int count);

struct simio_ops {
	inhook	pfin;
	outhook	pfout;
	void *devpara;
};


/*
 * allow same range existing. For some IO range, probably is using for different  drivers
 * at different phrases. Such as the serial port addresses for earlycon and normal console.
 */
#define SIMIO_REDUNDANT		0x01


struct simio_range {
	phys_addr_t start;/*inclusive*/
	phys_addr_t end;/*inclusive*/
	unsigned long flags;
};


struct simio_node {
	struct list_head ranlink;/*keep this field at the offset 0*/

	struct simio_range iores;
	/*pointer to the device provided services*/
	struct simio_ops *regops;
};


extern struct list_head simio_list;


extern int simio_range_reg(phys_addr_t startadr, resource_size_t rngsz);
extern int simio_range_locate(phys_addr_t startadr, resource_size_t rngsz);
extern int simio_range_setops(phys_addr_t startadr, resource_size_t rngsz,
					struct simio_ops *ops);
extern int simio_range_getops(unsigned long ptaddr, struct simio_ops **ops);



static inline u64 extio_inx(unsigned long addr, size_t dlen)
{
	struct simio_ops *simop;

	if (!simio_range_getops(addr, &simop) && simop && simop->pfin)
		return simop->pfin(simop->devpara, addr, NULL, dlen, 1);

	return -1;
}


#define BUILD_EXTIO(bw, type)						\
static inline void extio_out##bw(type value, unsigned long addr,	\
					size_t dlen)			\
{									\
	struct simio_ops *simop;					\
									\
	if (!simio_range_getops(addr, &simop) && simop && simop->pfout)	\
		simop->pfout(simop->devpara, addr, &value, dlen, 1);	\
}

/*only define outb now. Please add others if need.*/
BUILD_EXTIO(b, u8)


static inline void extio_insx(unsigned long addr, void *buffer,
					size_t dlen, unsigned int count)
{
	struct simio_ops *simop;

	if (!simio_range_getops(addr, &simop) && simop && simop->pfin)
		simop->pfin(simop->devpara, addr, buffer, dlen, count);
}

static inline void extio_outsx(unsigned long addr, const void *buffer,
					size_t dlen, unsigned int count)
{
	struct simio_ops *simop;

	if (!simio_range_getops(addr, &simop) && simop && simop->pfout)
		simop->pfout(simop->devpara, addr, buffer, dlen, count);
}


#endif /* __LINUX_EXTIO_H*/
