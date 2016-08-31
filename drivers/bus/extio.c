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


/*
 * register a valid IO range into simio_list.
 * pnew is allocated by caller, and should be freed by the caller too.
 * the range of pnew can be different from the one of ioadr.
 *
 * All these processings are done in critical area. Must not call printk or
 * anything relevant to in/out in the critical area, it will lead to deadlock.
 *
 * Returen 0 when successful, negative value for error, postive value
 * means conflict.
 */
int simio_range_reg(unsigned long syspt, unsigned long sysend,
			struct simio_node *pnew)
{
	struct simio_node *pos;
	struct list_head *pre;
	int ret;

	if (!pnew || syspt > IO_SPACE_LIMIT || sysend > IO_SPACE_LIMIT)
		return -EINVAL;

	pr_info("simio_reg:: 0x%lx - %lx\n", syspt, sysend);
	ret = 0;
	pre = &simio_list;
	write_lock(&simio_lock);
	list_for_each_entry(pos, &simio_list, ranlink) {
		/*pr_info("current start=0x%lx-%lx off=%lx\n", pos->iores.start,
				pos->iores.end, pos->iores.ptoffset);*/
		if (pos->iores.end < syspt) {
			pre = &pos->ranlink;
			continue;
		}

		if (pos->iores.start > sysend) {
			pre = pos->ranlink.prev;
			break;
		}

		/*matched a registered node...*/
		if (pos->iores.start == syspt &&
				pos->iores.end == sysend) {
			/*ret = SIMIO_EQUAL;*/
			/*update the old node*/
			pre = pos->ranlink.prev;
			list_del(&pos->ranlink);
			if (!(pos->iores.flags & SIMIO_STATIC_MEM))
				kfree(pos);
			break;
		} else {
			ret = SIMIO_PARTIAL;
			/*conflicting, exit directly...*/
			goto endreg;
		}
	}

	list_add(&pnew->ranlink, pre);

endreg:
	write_unlock(&simio_lock);

	return ret;
}


/*
 * unregister a valid range from simio_list.
 * only support the unregister for a full matched range.
 *
 * Returen 0 when successful, negative value for error, postive value means conflict.
 *
 */
int simio_range_unreg(unsigned long syspt, unsigned long sysend)
{
	struct simio_node *pos;
	int ret;

	if (syspt > IO_SPACE_LIMIT || sysend > IO_SPACE_LIMIT)
		return -EINVAL;

	ret = -ENXIO;
	write_lock(&simio_lock);
	list_for_each_entry(pos, &simio_list, ranlink) {
		if (pos->iores.end < syspt)
			continue;

		if (pos->iores.start == syspt && pos->iores.end == sysend) {
			ret = 0;
			list_del(&pos->ranlink);
		}
		break;
	}

	write_unlock(&simio_lock);

	if (!ret)
		kfree(pos);
	return ret;
}


/*
 * get a device specific method structure with a target port address.
 * @syspt: system port address which is probably not equal to physical
 *	port address.
 * @offset: cache the offset value from the match node.
 *
 * Returen NULL when can not find the matched node, otherwise will return
 * the matched one.
 *
 * the method structure pointer will be returned in **ops
 *
 */
struct simio_node * simio_range_getops(unsigned long syspt,
				unsigned long *offset)
{
	struct simio_node *pos;

	if (syspt > IO_SPACE_LIMIT)
		return NULL;

	read_lock(&simio_lock);
	list_for_each_entry(pos, &simio_list, ranlink) {
		if (pos->iores.end < syspt)
			continue;

		if (pos->iores.start <= syspt) {
			*offset = pos->iores.ptoffset;
			break;
		}
	}
	read_unlock(&simio_lock);

	if (&pos->ranlink == &simio_list)
		pos = NULL;
	return pos;
}

