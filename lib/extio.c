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
#include <linux/slab.h>
#include <linux/extio.h>

struct extio_ops *extio_ops_node;

static LIST_HEAD(extio_range_list);
static DEFINE_SPINLOCK(extio_range_lock);

struct extio_range *register_extio_ranges(struct fwnode_handle *fwnode,
		resource_size_t size, resource_size_t min_bus_io)
{
	struct extio_range *range;
	resource_size_t allocated_size = 0;

	if (!fwnode || !size)
		return NULL;

	/* check if the range hasn't been previously recorded */
	spin_lock(&extio_range_lock);
	list_for_each_entry(range, &extio_range_list, list) {
		if (range->fwnode == fwnode)
			/* range already registered, bail out */
			goto end_register;
		allocated_size += resource_size(range->io_host.res);
	}

	range = NULL;
	if (size == IO_RANGE_IOEXT) {
		pr_err("Can not find the match extio node!\n");
		goto end_register;
	}

	/* range not registed yet, check for available space */
	if (allocated_size + size - 1 >= EXTIO_LIMIT) {
		pr_warn("Requested IO range too big!"
			"EXTIO(0x%x) probably need extension!\n", EXTIO_LIMIT);
		goto end_register;
	}

	/* add the range to the list */
	range = kzalloc(sizeof(*range), GFP_ATOMIC);
	if (!range) {
		pr_err("alloc memory for IO range FAIL!\n");
		goto end_register;
	}

	range->fwnode = fwnode;
	range->io_host.res = &range->io_host.__res;
	range->io_host.res->start = allocated_size;
	range->io_host.res->end = range->io_host.res->start + size - 1;
	range->io_host.res->flags = IORESOURCE_IO;
	range->io_host.iostart = min_bus_io;
	range->io_host.offset = allocated_size - min_bus_io;

	list_add_tail(&range->list, &extio_range_list);

end_register:
	spin_unlock(&extio_range_lock);

	return range;
}


unsigned long extio_translate(struct fwnode_handle *node,
		unsigned long dev_io)
{
	resource_size_t io_end;
	struct extio_range *range;

	range = register_extio_ranges(node, IO_RANGE_IOEXT, dev_io);
	if (!range)
		return -1;

	io_end = range->io_host.iostart + resource_size(range->io_host.res);
	if (dev_io < range->io_host.iostart || dev_io >= io_end)
		return -1;

	return range->io_host.offset + dev_io;
}

struct extio_range * extio_getrange_byaddr(unsigned long addr)
{
	bool matched = false;
	struct extio_range *range;

	spin_lock(&extio_range_lock);
	list_for_each_entry(range, &extio_range_list, list) {
		if (range->io_host.res->start <= addr &&
			addr <= range->io_host.res->end && range->ops) {
			matched = true;
			break;
		}
	}
	spin_unlock(&extio_range_lock);

	return (matched) ? range : NULL;
}

#if 0
u8 inb(unsigned long addr)
{
	if (addr < EXTIO_LIMIT) {
		struct extio_range *range;

		list_for_each_entry(range, &extio_range_list, list) {
			if (range->io_host.res->start <= addr &&
				addr <= range->io_host.res->end) {
				return range->ops->pfin ?
					range->ops->pfin(range->ops->devpara,
						addr, sizeof(u8))) : -1;
		}
	}

	return readb(PCI_IOBASE + addr);
}
#endif

#ifdef PCI_IOBASE
BUILD_EXTIO(b, u8)

BUILD_EXTIO(w, u16)

BUILD_EXTIO(l, u32)
#endif
