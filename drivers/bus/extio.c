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

#include <linux/io.h>


struct extio_ops *arm64_extio_ops;


u8 __weak extio_inb(unsigned long addr)
{
	return arm64_extio_ops->pfin ?
		arm64_extio_ops->pfin(arm64_extio_ops->devpara,
			addr + arm64_extio_ops->ptoffset, NULL,
			sizeof(u8), 1) : -1;
}

void __weak extio_outb(u8 value, unsigned long addr)
{
	if (!arm64_extio_ops->pfout)
		return;

	arm64_extio_ops->pfout(arm64_extio_ops->devpara,
			addr + arm64_extio_ops->ptoffset, &value,
			sizeof(u8), 1);
}


void __weak extio_insb(unsigned long addr, void *buffer,
				unsigned int count)
{
	if (!arm64_extio_ops->pfin)
		return;

	arm64_extio_ops->pfin(arm64_extio_ops->devpara,
			addr + arm64_extio_ops->ptoffset, buffer,
			sizeof(u8), count);
}

void __weak extio_outsb(unsigned long addr, const void *buffer,
			 unsigned int count)
{
	if (!arm64_extio_ops->pfout)
		return;

	arm64_extio_ops->pfout(arm64_extio_ops->devpara,
			addr + arm64_extio_ops->ptoffset, buffer,
			sizeof(u8), count);
}


