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
#include <linux/slab.h>



static DEFINE_RWLOCK(simio_lock);

/*
 * all the simio range nodes are linked this list.
 * the existing resource functions can be used for simio management. But worry about
 * the same rwlock will cause frequently conflicts for the in/out callings. for the better
 * performance and more easy to maintain, introducing this independant list.
 */
LIST_HEAD(simio_list);




#define SIMIO_OUT_CONTIAIN	0x01
#define SIMIO_EQUAL		0x02
#define SIMIO_IN_CONTIAIN	0x04
#define SIMIO_PARTIAL		0x08


/*
 * register a valid IO range into simio_list.
 * only support a complete new range;
 *
 * Returen 0 when successful, negative value for error, postive value means conflict.
 *
 */
int simio_range_reg(phys_addr_t startadr, resource_size_t rngsz)
{
	struct simio_node *pos;
	struct list_head *pre;
	phys_addr_t endadr;
	int ret;

	if (!rngsz || startadr >= PCIBIOS_MIN_IO ||
		(startadr + rngsz - 1) >= PCIBIOS_MIN_IO)
		return -EINVAL;

	endadr = startadr + rngsz - 1;
	ret = 0;
	pre = &simio_list;
	write_lock(&simio_lock);
	list_for_each_entry(pos, &simio_list, ranlink) {
		if (pos->iores.start > endadr) {
			pre = pos->ranlink.prev;
			break;
		}

		if (pos->iores.end < startadr)
			continue;

		/*conflict range...*/
		ret = SIMIO_PARTIAL | SIMIO_IN_CONTIAIN | SIMIO_EQUAL;
		goto endreg;
	}

	/*ok. create a new node and register it..*/
	pos = (struct simio_node *)kzalloc(sizeof(*pos), GFP_KERNEL);
	if (pos) {
	 	pos->iores.start = startadr;
		pos->iores.end = endadr;
		list_add(&pos->ranlink, pre);
	} else
		ret = -ENOMEM;

endreg:
	write_unlock(&simio_lock);

	return ret;
}


int simio_range_unreg(phys_addr_t startadr, resource_size_t rngsz)
{
	struct simio_node *pos;
	phys_addr_t endadr;
	int ret;

	if (!rngsz || startadr >= PCIBIOS_MIN_IO ||
		(startadr + rngsz - 1) >= PCIBIOS_MIN_IO)
		return -EINVAL;

	endadr = startadr + rngsz - 1;
	write_lock(&simio_lock);
	list_for_each_entry(pos, &simio_list, ranlink) {
		if (pos->iores.start == startadr &&
			pos->iores.end == endadr) {
			ret = 0;
			list_del(&pos->ranlink);
			goto endsetops;
		}
	}
	ret = -ENXIO;

endsetops:
	write_unlock(&simio_lock);

	if (!ret)
		kfree(pos);
	return ret;
}



/*
 * check whether the IO range required is registered.
 *
 * Returen 0 when successful, negative value for error, postive value means conflict.
 *
 */
int simio_range_locate(phys_addr_t startadr, resource_size_t rngsz)
{
	struct simio_node *pos;
	phys_addr_t endadr;
	int ret;

	if (!rngsz || startadr >= PCIBIOS_MIN_IO ||
		(startadr + rngsz - 1) >= PCIBIOS_MIN_IO)
		return -EINVAL;

	endadr = startadr + rngsz - 1;
	ret = SIMIO_OUT_CONTIAIN;
	read_lock(&simio_lock);
	list_for_each_entry(pos, &simio_list, ranlink) {
		if (pos->iores.start > endadr)
			break;

		if (pos->iores.end < startadr)
			continue;

		/*conflict range...*/
		if (pos->iores.start > startadr || pos->iores.end < endadr)
			ret = SIMIO_PARTIAL;
		else {
			ret = 0;
			break;
		}
	}

	read_unlock(&simio_lock);
	return ret;
}



/*
 * set a device specific method structure in a register IO range node.
 *
 * Returen 0 when successful, other are for failure.
 */
int simio_range_setops(phys_addr_t startadr, resource_size_t rngsz,
				struct simio_ops *ops)
{
	struct simio_node *pos;
	phys_addr_t endadr;
	int ret;

	if (!rngsz || startadr >= PCIBIOS_MIN_IO ||
		(startadr + rngsz - 1) >= PCIBIOS_MIN_IO)
		return -EINVAL;

	endadr = startadr + rngsz - 1;
	write_lock(&simio_lock);
	list_for_each_entry(pos, &simio_list, ranlink) {
		if (pos->iores.start == startadr &&
			pos->iores.end == endadr) {
			pos->regops = ops;
			ret = 0;
			goto endsetops;
		}
	}
	ret = -ENXIO;

endsetops:

	write_unlock(&simio_lock);
	return -ret;
}


/*
 * get a device specific method structure with a target port address.
 *
 * Returen 0 when successful, other are for failure.
 * the method structure pointer will be returned in **ops
 *
 */
int simio_range_getops(unsigned long ptaddr, struct simio_ops **ops)
{
	struct simio_node *pos;
	int ret;

	if (ptaddr >= PCIBIOS_MIN_IO)
		return -EINVAL;

	*ops = NULL;
	ret = -ENXIO;
	read_lock(&simio_lock);
	list_for_each_entry(pos, &simio_list, ranlink) {
		if (ptaddr >= pos->iores.start && ptaddr <= pos->iores.end) {
			*ops = pos->regops;
			ret = 0;
			break;
		}
	}

	read_unlock(&simio_lock);
	return ret;
}


