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



static DEFINE_RWLOCK(extio_lock);



/*
 * all the extio range nodes are linked this list.
 */
LIST_HEAD(extio_list);


/*
 * register a valid IO range into extio_list.
 * pnew is allocated by caller, and should be freed by the caller too.
 * the range of pnew can be different from the one of ioadr.
 *
 * All these processings are done in critical area. Must not call printk or
 * anything relevant to in/out in the critical area, it will lead to deadlock.
 *
 * Returen 0 when successful, negative value for error, postive value
 * means conflict.
 */
int extio_range_reg(unsigned long syspt, unsigned long sysend,
			struct extio_node *pnew)
{
	struct extio_node *pos;
	struct list_head *pre;
	int ret;

	if (!pnew || syspt > IO_SPACE_LIMIT || sysend > IO_SPACE_LIMIT)
		return -EINVAL;

	pr_info("simio_reg:: 0x%lx - %lx\n", syspt, sysend);
	ret = 0;
	pre = &extio_list;
	write_lock(&extio_lock);
	list_for_each_entry(pos, &extio_list, ranlink) {
		if (pos->iores.end < syspt) {
			pre = &pos->ranlink;
			continue;
		}

		if (pos->iores.start > sysend) {
			pre = pos->ranlink.prev;
			break;
		}

		/* matched a registered node */
		if (pos->iores.start == syspt &&
				pos->iores.end == sysend) {
			/* update the old node */
			pre = pos->ranlink.prev;
			list_del(&pos->ranlink);
			if (!(pos->iores.flags & EXTIO_STATIC_MEM))
				kfree(pos);
			break;
		} else {
			ret = EXTIO_PARTIAL;
			/* conflicting, exit directly */
			goto endreg;
		}
	}

	list_add(&pnew->ranlink, pre);

endreg:
	write_unlock(&extio_lock);

	return ret;
}


/*
 * unregister a valid range from simio_list.
 * only support the unregister for a full matched range.
 *
 * Returen 0 when successful, negative value for error, postive value means
 * conflict.
 *
 */
int extio_range_unreg(unsigned long syspt, unsigned long sysend)
{
	struct extio_node *pos;
	int ret;

	if (syspt > IO_SPACE_LIMIT || sysend > IO_SPACE_LIMIT)
		return -EINVAL;
 
	ret = -ENXIO;
	write_lock(&extio_lock);
	list_for_each_entry(pos, &extio_list, ranlink) {
		if (pos->iores.end < syspt)
			continue;

		if (pos->iores.start == syspt && pos->iores.end == sysend) {
			ret = 0;
			list_del(&pos->ranlink);
		}
		break;
	}

	write_unlock(&extio_lock);

	if (!ret)
		kfree(pos);
	return ret;
}


/*
 * Get a device specific method structure with a target port address.
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
struct extio_node * extio_range_getops(unsigned long syspt,
				unsigned long *offset)
{
	struct extio_node *pos;

	if (syspt > IO_SPACE_LIMIT)
		return NULL;

	read_lock(&extio_lock);
	list_for_each_entry(pos, &extio_list, ranlink) {
		if (pos->iores.end < syspt)
			continue;

		if (pos->iores.start <= syspt) {
			*offset = pos->iores.ptoffset;
			break;
		}
	}
	read_unlock(&extio_lock);

	if (&pos->ranlink == &extio_list)
		pos = NULL;

	return pos;
}


/*
 * relock is re-entrant. So, here pr_info is safe.
 */
void extio_range_scan(void)
{
	struct extio_node *pos;

	pr_info("####SIMIO RANGE#####\n");
	read_lock(&extio_lock);
	list_for_each_entry(pos, &extio_list, ranlink) {
		pr_info("start =0x%lx\tend=0x%lx  offset=0x%lx\n",
			pos->iores.start, pos->iores.end, pos->iores.ptoffset);


		pr_info("regops=%p\n", pos->regops);
	}

	read_unlock(&extio_lock);
	return;
}

