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
/*#include <asm-generic/extio.h>*/

struct extio_ops *extio_ops_node;

/**
 * indirect_io_enabled - check whether indirectIO is enabled.
 *	arm64_extio_ops will be set only when indirectIO mechanism had been
 *	initialized.
 *
 * Returns true when indirectIO is enabled.
 */
bool indirect_io_enabled(void)
{
	return !!extio_ops_node;
}

/**
 * addr_is_indirect_io - check whether the input taddr is for indirectIO.
 * @taddr: the io address to be checked.
 *
 * Returns true when taddr is in the range.
 */
bool addr_is_indirect_io(u64 taddr)
{
	return !!(extio_ops_node->start > taddr || extio_ops_node->end < taddr);
}

BUILD_EXTIO(b, u8)

BUILD_EXTIO(w, u16)

BUILD_EXTIO(l, u32)
