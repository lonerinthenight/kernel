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

#include <linux/io.h>

struct extio_ops *arm64_extio_ops;

/**
 * indirect_io_enabled - check whether indirectIO is enabled.
 *	arm64_extio_ops will be set only when indirectIO mechanism had been
 *	initialized.
 *
 * Returns true when indirectIO is enabled.
 */
bool indirect_io_enabled(void)
{
	return arm64_extio_ops ? true : false;
}

/**
 * addr_is_indirect_io - check whether the input taddr is for indirectIO.
 * @taddr: the io address to be checked.
 *
 * Returns 1 when taddr is in the range; otherwise return 0.
 */
int addr_is_indirect_io(u64 taddr)
{
	if (arm64_extio_ops->start > taddr || arm64_extio_ops->end < taddr)
		return 0;

	return 1;
}

BUILD_EXTIO(b, u8)

BUILD_EXTIO(w, u16)

BUILD_EXTIO(l, u32)
