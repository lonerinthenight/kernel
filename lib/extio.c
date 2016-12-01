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
#include <linux/ioport.h>
#include <linux/extio.h>
#include <linux/of.h>
#include <linux/acpi.h>
#include <linux/slab.h>
#include <linux/sizes.h>

#if defined(PCI_IOBASE) || defined(CONFIG_INDIRECT_PIO)
struct io_range {
	struct list_head list;
	phys_addr_t start;
	resource_size_t size;
};

static LIST_HEAD(io_range_list);
static DEFINE_SPINLOCK(io_range_lock);

/*
 * point to a io_range_list node which represents the linux PIO range
 * specially for legacy IO. Once this linux PIO range is allocated, will
 * not be released till system reboots.
 */
static struct extio_windows legacy_iospace;
#endif

/*
 * Record the PCI IO range (expressed as CPU physical address + size).
 * Return a negative value if an error has occured, zero otherwise
 */
int __weak pci_register_io_range(phys_addr_t addr, resource_size_t size)
{
	int err = 0;

#if defined(PCI_IOBASE) || defined(CONFIG_INDIRECT_PIO)
	struct io_range *range;
	resource_size_t allocated_size = 0;

	if (!size)
		return -EINVAL;
#ifdef CONFIG_INDIRECT_PIO
	if (addr != IO_RANGE_IOEXT)
		return -EINVAL;
#endif

	/* check if the range hasn't been previously recorded */
	if (addr == IO_RANGE_IOEXT && legacy_iospace.pio_node)
		goto end_register;
	spin_lock(&io_range_lock);
	list_for_each_entry(range, &io_range_list, list) {
		if (addr >= range->start && addr + size <= range->start + size) {
			/* range already registered, bail out */
			goto end_register;
		}
		allocated_size += range->size;
	}

	/* range not registed yet, check for available space */
	if (allocated_size + size - 1 > IO_SPACE_LIMIT) {
		if (addr == IO_RANGE_IOEXT) {
			pr_warn("Please check whether can increase the legacy IO(0x%lx from 0x%x)!\n",
				(unsigned long)size, IO_SPACE_LIMIT);
			err = -ENXIO;
			goto end_register;
		}

		/* if it's too big check if 64K space can be reserved */
		if (allocated_size + SZ_64K - 1 > IO_SPACE_LIMIT) {
			err = -E2BIG;
			goto end_register;
		}

		size = SZ_64K;
		pr_warn("Requested IO range too big, new size set to 64K\n");
	}

	/* add the range to the list */
	range = kzalloc(sizeof(*range), GFP_ATOMIC);
	if (!range) {
		err = -ENOMEM;
		goto end_register;
	}

	if (addr == IO_RANGE_IOEXT) {
		legacy_iospace.root_res.start = allocated_size;
		legacy_iospace.root_res.end = allocated_size + size - 1;
		legacy_iospace.root_res.flags = IORESOURCE_IO;

		err = request_resource(&ioport_resource,
				&legacy_iospace.root_res);
		if (err) {
			pr_err("request resource for legacy IO FAIL!\n");
			kfree(range);
			goto end_register;
		}

		legacy_iospace.pio_node = range;
	}

	range->start = (addr == IO_RANGE_IOEXT) ? allocated_size : addr;
	range->size = size;

	list_add_tail(&range->list, &io_range_list);

end_register:
	spin_unlock(&io_range_lock);
#endif

	return err;
}

#ifdef CONFIG_PCI
phys_addr_t pci_pio_to_address(unsigned long pio)
{
	phys_addr_t address = (phys_addr_t)OF_BAD_ADDR;

#ifdef PCI_IOBASE
	struct io_range *range;
	resource_size_t allocated_size = 0;

	if (pio > IO_SPACE_LIMIT)
		return address;

	spin_lock(&io_range_lock);
	list_for_each_entry(range, &io_range_list, list) {
		if (pio >= allocated_size &&
			pio < allocated_size + range->size &&
			range != legacy_iospace.pio_node) {
			address = range->start + pio - allocated_size;
			break;
		}
		allocated_size += range->size;
	}
	spin_unlock(&io_range_lock);
#endif

	return address;
}

unsigned long __weak pci_address_to_pio(phys_addr_t address)
{
#ifdef PCI_IOBASE
	struct io_range *res;
	resource_size_t offset = 0;
	unsigned long addr = -1;

	spin_lock(&io_range_lock);
	list_for_each_entry(res, &io_range_list, list) {
		if (res != legacy_iospace.pio_node &&
			address >= res->start &&
			address < res->start + res->size) {
			addr = address - res->start + offset;
			break;
		}
		offset += res->size;
	}
	spin_unlock(&io_range_lock);

	return addr;
#else
	if (address > IO_SPACE_LIMIT)
		return (unsigned long)-1;

	return (unsigned long) address;
#endif
}
#endif

#if defined(PCI_IOBASE) && defined(CONFIG_INDIRECT_PIO)
#define LEGACY_IO_DEFAULT_ALIGN	0x01

struct extio_range *extio_ent;

int register_bus_extio_range(struct fwnode_handle *fwnode,
			struct extio_range *range)
{
	int ret;

	if (extio_ent) {
		pr_err("Only support one indirect IO bus now!\n");
		return -EBUSY;
	}

	if (!range || (!is_of_node(fwnode) && !is_acpi_node(fwnode))) {
		pr_err("The device is not ACPI and OF!\n");
		return -EINVAL;
	}

	ret = allocate_resource(&legacy_iospace.root_res, &range->iowin,
			LEGACY_BUS_IO_SIZE, 0, ~0, LEGACY_IO_DEFAULT_ALIGN,
			NULL, NULL);
	if (ret) {
		pr_err("Allocate I/O from (%pR) FAIL(%d)\n",
				&legacy_iospace.root_res, -ret);
		return ret;
	}

	/*
	 * The IO can not start from ZERO, otherwise some devices will
	 * fail to initialization.
	 */
	if (!range->iowin.start)
		range->iowin.start = 1;
	/* suppose the start bus IO is ZERO... */
	range->offset = range->iowin.start - 0;
	range->fwnode = fwnode;

	extio_ent = range;

	return ret;
}

unsigned long extio_translate(struct fwnode_handle *node,
		unsigned long dev_io)
{
	if (!node || dev_io >= LEGACY_BUS_IO_SIZE) {
		pr_err("extio tanslate:: invalid parameters!\n");
		return -1;
	}

	if (extio_ent->fwnode != node)
		return -1;

	/* suppose lagacy device IO always starts from ZERO. */
	return extio_ent->offset + dev_io;
}

BUILD_EXTIO(b, u8)

BUILD_EXTIO(w, u16)

BUILD_EXTIO(l, u32)
#endif
