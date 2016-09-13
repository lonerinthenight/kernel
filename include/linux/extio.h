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
	unsigned long start;/* inclusive, sys io addr */
	unsigned long end;/* inclusive, sys io addr */
	unsigned long ptoffset;/* port Io - system Io */

	inhook	pfin;
	outhook	pfout;
	void *devpara;
};


extern struct extio_ops *arm64_extio_ops;

extern u8 extio_inb(unsigned long addr);
extern void extio_outb(u8 value, unsigned long addr);
extern void extio_insb(unsigned long addr, void *buffer, unsigned int count);
extern void extio_outsb(unsigned long addr, const void *buffer,
				unsigned int count);


#endif /* __LINUX_EXTIO_H*/
