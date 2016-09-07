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


typedef u64 (*inhook)(void *devobj, unsigned long ptaddr, void *inbuf,
				size_t dlen, unsigned int count);
typedef void (*outhook)(void *devobj, unsigned long ptaddr,
				const void *outbuf, size_t dlen,
				unsigned int count);

struct extio_ops {
	inhook	pfin;
	outhook	pfout;
	void *devpara;
};


#define EXTIO_OUT_CONTIAIN	0x01
#define EXTIO_EQUAL		0x02
#define EXTIO_IN_CONTIAIN	0x04
#define EXTIO_PARTIAL		0x08


/*
 * the node is defined as static variable rather than dynamically memory
 * allocation. This is available for those devices working in early boot,
 * such as earlycon.
 */
#define EXTIO_STATIC_MEM	0x01


/* critical area */
struct extio_range {
	unsigned long start;/* inclusive, sys io addr */
	unsigned long end;/* inclusive, sys io addr */
	unsigned long ptoffset;/* port Io - system Io */
	unsigned long flags;
};


struct extio_node {
	struct list_head ranlink;

	struct extio_range iores;

	/*pointer to the device provided services*/
	struct extio_ops *regops;
};


extern struct list_head extio_list;


extern int extio_range_reg(unsigned long syspt, unsigned long sysend,
					struct extio_node *pnew);
extern int extio_range_unreg(unsigned long syspt, unsigned long sysend);

extern struct extio_node * extio_range_getops(unsigned long syspt,
						unsigned long *offset);

#endif /* __LINUX_EXTIO_H*/
