/*
 * Copyright (C) 2016 Hisilicon Limited, All Rights Reserved.
 * Author: Zhichang Yuan <yuanzhichang@hisilicon.com>
 * Author: Zou Rongrong <zourongrong@huawei.com>
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

#include <linux/acpi.h>
#include <linux/console.h>
#include <linux/delay.h>
/* #include <linux/extio.h> */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/serial_8250.h>
#include <linux/slab.h>

/*
 * setting this bit means each IO operation will target to different port address;
 * 0 means repeatly IO operations will be sticked on the same port, such as BT;
 */
#define FG_INCRADDR_LPC		0x02

struct lpc_cycle_para {
	unsigned int opflags;
	unsigned int csize; /* the data length of each operation */
};

struct hisilpc_dev {
	spinlock_t cycle_lock;
	void __iomem  *membase;

	struct extio_bus_res *io_ent;
};


/* The maximum continous operations*/
#define LPC_MAX_OPCNT	16
/* only support IO data unit length is four at maximum */
#define LPC_MAX_DULEN	4
#if LPC_MAX_DULEN > LPC_MAX_OPCNT
#error "LPC.. MAX_DULEN must be not bigger than MAX_OPCNT!"
#endif

#define LPC_REG_START		0x00 /* start a new LPC cycle */
#define LPC_REG_OP_STATUS	0x04 /* the current LPC status */
#define LPC_REG_IRQ_ST		0x08 /* interrupt enable&status */
#define LPC_REG_OP_LEN		0x10 /* how many LPC cycles each start */
#define LPC_REG_CMD		0x14 /* command for the required LPC cycle */
#define LPC_REG_ADDR		0x20 /* LPC target address */
#define LPC_REG_WDATA		0x24 /* data to be written */
#define LPC_REG_RDATA		0x28 /* data coming from peer */


/* The command register fields*/
#define LPC_CMD_SAMEADDR	0x08
#define LPC_CMD_TYPE_IO		0x00
#define LPC_CMD_WRITE		0x01
#define LPC_CMD_READ		0x00
/* the bit attribute is W1C. 1 represents OK. */
#define LPC_STAT_BYIRQ		0x02

#define LPC_STATUS_IDLE		0x01
#define LPC_OP_FINISHED		0x02

#define START_WORK		0x01

/*
 * The minimal waiting interval... Suggest it is not less than 10.
 * Bigger value probably will lower the performance.
 */
#define LPC_NSEC_PERWAIT	100
/*
 * The maximum waiting time is about 128us.
 * The fastest IO cycle time is about 390ns, but the worst case will wait
 * for extra 256 lpc clocks, so (256 + 13) * 30ns = 8 us. The maximum
 * burst cycles is 16. So, the maximum waiting time is about 128us under
 * worst case.
 * choose 1300 as the maximum.
 */
#define LPC_MAX_WAITCNT		1300
/* About 10us. This is specfic for single IO operation, such as inb. */
#define LPC_PEROP_WAITCNT	100


static inline int wait_lpc_idle(unsigned char *mbase,
				unsigned int waitcnt) {
	u32 opstatus;

	while (waitcnt--) {
		ndelay(LPC_NSEC_PERWAIT);
		opstatus = readl(mbase + LPC_REG_OP_STATUS);
		if (opstatus & LPC_STATUS_IDLE)
			return (opstatus & LPC_OP_FINISHED) ? 0 : (-EIO);
	}
	return -ETIME;
}

/*
 * hisilpc_target_in - trigger a series of lpc cycles to read required data
 *		  from target peripheral.
 * @pdev: pointer to hisi lpc device
 * @para: some parameters used to control the lpc I/O operations
 * @ptaddr: the lpc I/O target port address
 * @buf: where the read back data is stored
 * @opcnt: how many I/O operations required in this calling
 *
 * only one byte data is read each I/O operation.
 *
 * Returns 0 on success, non-zero on fail.
 *
 */
static int hisilpc_target_in(struct hisilpc_dev *lpcdev,
				struct lpc_cycle_para *para,
				unsigned long ptaddr, unsigned char *buf,
				unsigned long opcnt)
{
	unsigned long cnt_per_trans;
	unsigned int cmd_word;
	unsigned int waitcnt;
	int ret;

	if (!buf || !opcnt || !para || !para->csize || !lpcdev)
		return -EINVAL;

	if (opcnt  > LPC_MAX_OPCNT)
		return -EINVAL;

	cmd_word = LPC_CMD_TYPE_IO | LPC_CMD_READ;
	waitcnt = (LPC_PEROP_WAITCNT);
	if (!(para->opflags & FG_INCRADDR_LPC)) {
		cmd_word |= LPC_CMD_SAMEADDR;
		waitcnt = LPC_MAX_WAITCNT;
	}

	ret = 0;
	cnt_per_trans = (para->csize == 1) ? opcnt : para->csize;
	for (; opcnt && !ret; cnt_per_trans = para->csize) {
		unsigned long flags;

		/* whole operation must be atomic */
		spin_lock_irqsave(&lpcdev->cycle_lock, flags);

		writel(cnt_per_trans, lpcdev->membase + LPC_REG_OP_LEN);

		writel(cmd_word, lpcdev->membase + LPC_REG_CMD);

		writel(ptaddr, lpcdev->membase + LPC_REG_ADDR);

		writel(START_WORK, lpcdev->membase + LPC_REG_START);

		/* whether the operation is finished */
		ret = wait_lpc_idle(lpcdev->membase, waitcnt);
		if (!ret) {
			opcnt -= cnt_per_trans;
			for (; cnt_per_trans--; buf++)
				*buf = readl(lpcdev->membase + LPC_REG_RDATA);
		}

		spin_unlock_irqrestore(&lpcdev->cycle_lock, flags);
	}

	return ret;
}

/*
 * hisilpc_target_out - trigger a series of lpc cycles to write required data
 *		  to target peripheral.
 * @pdev: pointer to hisi lpc device
 * @para: some parameters used to control the lpc I/O operations
 * @ptaddr: the lpc I/O target port address
 * @buf: where the data to be written is stored
 * @opcnt: how many I/O operations required
 *
 * only one byte data is read each I/O operation.
 *
 * Returns 0 on success, non-zero on fail.
 *
 */
static int hisilpc_target_out(struct hisilpc_dev *lpcdev,
				struct lpc_cycle_para *para,
				unsigned long ptaddr,
				const unsigned char *buf,
				unsigned long opcnt)
{
	unsigned long cnt_per_trans;
	unsigned int cmd_word;
	unsigned int waitcnt;
	int ret;

	if (!buf || !opcnt || !para || !lpcdev)
		return -EINVAL;

	if (opcnt > LPC_MAX_OPCNT)
		return -EINVAL;
	/* default is increasing address */
	cmd_word = LPC_CMD_TYPE_IO | LPC_CMD_WRITE;
	waitcnt = (LPC_PEROP_WAITCNT);
	if (!(para->opflags & FG_INCRADDR_LPC)) {
		cmd_word |= LPC_CMD_SAMEADDR;
		waitcnt = LPC_MAX_WAITCNT;
	}

	ret = 0;
	cnt_per_trans = (para->csize == 1) ? opcnt : para->csize;
	for (; opcnt && !ret; cnt_per_trans = para->csize) {
		unsigned long flags;

		spin_lock_irqsave(&lpcdev->cycle_lock, flags);

		writel(cnt_per_trans, lpcdev->membase + LPC_REG_OP_LEN);
		opcnt -= cnt_per_trans;
		for (; cnt_per_trans--; buf++)
			writel(*buf, lpcdev->membase + LPC_REG_WDATA);

		writel(cmd_word, lpcdev->membase + LPC_REG_CMD);

		writel(ptaddr, lpcdev->membase + LPC_REG_ADDR);

		writel(START_WORK, lpcdev->membase + LPC_REG_START);

		/* whether the operation is finished */
		ret = wait_lpc_idle(lpcdev->membase, waitcnt);

		spin_unlock_irqrestore(&lpcdev->cycle_lock, flags);
	}

	return ret;
}

/*
 * hisilpc_comm_in - read/input the data from the I/O peripheral through LPC.
 * @devobj: pointer to the device information relevant to LPC controller.
 * @ptaddr: the target I/O port address.
 * @dlen: the data length required to read from the target I/O port.
 *
 * when succeed, the data read back is stored in buffer pointed by inbuf.
 * For inb, return the data read from I/O or -1 when error occur.
 */
static u64 hisilpc_comm_in(void *devobj, unsigned long ptaddr, size_t dlen)
{
	struct hisilpc_dev *lpcdev;
	struct lpc_cycle_para iopara;
	u32 rd_data;
	unsigned char *newbuf;
	int ret = 0;

	if (!devobj || !dlen || dlen > LPC_MAX_DULEN ||	(dlen & (dlen - 1)))
		return -1;

	/* the local buffer must be enough for one data unit */
	if (sizeof(rd_data) < dlen)
		return -1;

	newbuf = (unsigned char *)&rd_data;

	lpcdev = (struct hisilpc_dev *)devobj;

	iopara.opflags = FG_INCRADDR_LPC;
	iopara.csize = dlen;

	ret = hisilpc_target_in(lpcdev, &iopara, ptaddr, newbuf, dlen);
	if (ret)
		return -1;

	return le32_to_cpu(rd_data);
}

/*
 * hisilpc_comm_out - write/output the data whose maximal length is four bytes to
 *		the I/O peripheral through LPC.
 * @devobj: pointer to the device information relevant to LPC controller.
 * @outval: a value to be outputed from caller, maximum is four bytes.
 * @ptaddr: the target I/O port address.
 * @dlen: the data length required writing to the target I/O port .
 *
 * This function is corresponding to out(b,w,l) only
 *
 */
static void hisilpc_comm_out(void *devobj, unsigned long ptaddr,
				u32 outval, size_t dlen)
{
	struct hisilpc_dev *lpcdev;
	struct lpc_cycle_para iopara;
	const unsigned char *newbuf;

	if (!devobj || !dlen || dlen > LPC_MAX_DULEN)
		return;

	if (sizeof(outval) < dlen)
		return;

	outval = cpu_to_le32(outval);

	newbuf = (const unsigned char *)&outval;
	lpcdev = (struct hisilpc_dev *)devobj;

	iopara.opflags = FG_INCRADDR_LPC;
	iopara.csize = dlen;

	hisilpc_target_out(lpcdev, &iopara, ptaddr, newbuf, dlen);
}

/*
 * hisilpc_comm_ins - read/input the data in buffer to the I/O peripheral
 *		    through LPC, it corresponds to ins(b,w,l)
 * @devobj: pointer to the device information relevant to LPC controller.
 * @ptaddr: the target I/O port address.
 * @inbuf: a buffer where read/input data bytes are stored.
 * @dlen: the data length required writing to the target I/O port.
 * @count: how many data units whose length is dlen will be read.
 *
 */
static u64 hisilpc_comm_ins(void *devobj, unsigned long ptaddr,
			void *inbuf, size_t dlen, unsigned int count)
{
	struct hisilpc_dev *lpcdev;
	struct lpc_cycle_para iopara;
	unsigned char *newbuf;
	unsigned int loopcnt, cntleft;
	unsigned int max_perburst;
	int ret = 0;

	if (!devobj || !inbuf || !count || !dlen ||
			dlen > LPC_MAX_DULEN || (dlen & (dlen - 1)))
		return -1;

	iopara.opflags = 0;
	if (dlen > 1)
		iopara.opflags |= FG_INCRADDR_LPC;
	iopara.csize = dlen;

	lpcdev = (struct hisilpc_dev *)devobj;
	newbuf = (unsigned char *)inbuf;
	/*
	 * ensure data stream whose length is multiple of dlen to be processed
	 * each IO input
	 */
	max_perburst = LPC_MAX_OPCNT & (~(dlen - 1));
	cntleft = count * dlen;
	do {
		loopcnt = (cntleft >= max_perburst) ? max_perburst : cntleft;
		ret = hisilpc_target_in(lpcdev, &iopara, ptaddr, newbuf,
						loopcnt);
		if (ret)
			break;
		newbuf += loopcnt;
		cntleft -= loopcnt;
	} while (cntleft);

	return ret;
}

/*
 * hisilpc_comm_outs - write/output the data in buffer to the I/O peripheral
 *		    through LPC, it corresponds to outs(b,w,l)
 * @devobj: pointer to the device information relevant to LPC controller.
 * @ptaddr: the target I/O port address.
 * @outbuf: a buffer where write/output data bytes are stored.
 * @dlen: the data length required writing to the target I/O port .
 * @count: how many data units whose length is dlen will be written.
 *
 */
static void hisilpc_comm_outs(void *devobj, unsigned long ptaddr,
			const void *outbuf, size_t dlen, unsigned int count)
{
	struct hisilpc_dev *lpcdev;
	struct lpc_cycle_para iopara;
	const unsigned char *newbuf;
	unsigned int loopcnt, cntleft;
	unsigned int max_perburst;
	int ret = 0;

	if (!devobj || !outbuf || !count || !dlen ||
			dlen > LPC_MAX_DULEN || (dlen & (dlen - 1)))
		return;

	iopara.opflags = 0;
	if (dlen > 1)
		iopara.opflags |= FG_INCRADDR_LPC;
	iopara.csize = dlen;

	lpcdev = (struct hisilpc_dev *)devobj;
	newbuf = (unsigned char *)outbuf;
	/*
	 * ensure data stream whose lenght is multiple of dlen to be processed
	 * each IO input
	 */
	max_perburst = LPC_MAX_OPCNT & (~(dlen - 1));
	cntleft = count * dlen;
	do {
		loopcnt = (cntleft >= max_perburst) ? max_perburst : cntleft;
		ret = hisilpc_target_out(lpcdev, &iopara, ptaddr, newbuf,
						loopcnt);
		if (ret)
			break;
		newbuf += loopcnt;
		cntleft -= loopcnt;
	} while (cntleft);
}


static int hisilpc_child_notifier(struct notifier_block *nb,
				   unsigned long action, void *data)
{

	return 0;
}


static struct notifier_block hisilpc_dev_nb = {
	.notifier_call = hisilpc_child_notifier,
};


/*
 * hisilpc_child_iorequest - request the linux IO resource of child from
 *		LPC bus IO window.
 *
 * @child: pointer to the child device.
 * @data: the IO window of LPC bus.
 *
 * return 0 for success, negative value for failure.
 */
static int hisilpc_child_io_insert(struct device *child, void *data)
{
	int i;
	int ret;
	struct resource *root;
	struct platform_device *pdev;
	struct extio_dev_res *dev_res;
	struct extio_alloc_seg *segnode;

	if (!child || !child->parent)
		return -EINVAL;

	pdev = to_platform_device(child);
	/* find the IO resource from device */
	for (i = 0; i < pdev->num_resources; i++) {
		if (&pdev->resource[i].flags & IORESOURCE_IO)
			break;
	}
	if (i == pdev->num_resources)
		return -ENXIO;
	root = pdev->resource + i;

	segnode = extio_find_registered_io(child->parent->fwnode, root->start,
			resource_size(root));
	if (!segnode)
		return -ENXIO;

	dev_res = kzalloc(child, sizeof(*dev_res), GFP_KERNEL);
	if (!dev_res)
		return -ENOMEM;

	dev_res->res = *root;
	dev_res->offset = segnode->offset;
	dev_res->devops = &segnode->bus_res->ops;
	dev_res->dev = child;
	dev_res->pseg = segnode;

	root = (struct resource *)data;
	ret = insert_resource(child, root, &dev_res->res);
	if (ret)
		dev_err(child, "request IO resource(%pR) FAIL(%d)!\n",
				&dev_res->res, -ret);

	/* register a callback to release this resource */
	ret = bus_register_notifier(pdev->dev.bus, &hisilpc_dev_nb);
	if (ret)
		dev_err(child, "Failed to register hisilpc notifier in'%s'\n");

	return ret;
}


struct platform_device *acpi_extio_reg_platform_dev(struct acpi_device *adev,
				struct property_entry *properties)
{
	struct platform_device *pdev = NULL;
	struct platform_device_info pdevinfo;
	struct resource_entry *rentry;
	struct list_head resource_list;
	struct resource *resources = NULL;
	int count;

	/* If the ACPI node already has a physical device attached, skip it. */
	if (adev->physical_node_count)
		return NULL;

	INIT_LIST_HEAD(&resource_list);
	count = acpi_dev_get_resources(adev, &resource_list, NULL, NULL);
	if (count < 0) {
		return NULL;
	} else if (count > 0) {
		resources = kzalloc(count * sizeof(struct resource),
				    GFP_KERNEL);
		if (!resources) {
			dev_err(&adev->dev, "No memory for resources\n");
			acpi_dev_free_resource_list(&resource_list);
			return ERR_PTR(-ENOMEM);
		}
		count = 0;
		list_for_each_entry(rentry, &resource_list, node) {
			if (rentry->res->flags & IORESOURCE_IO) {
				unsigned long sys_start;

				sys_start = extio_translate(&adev->parent->fwnode,
						rentry->res->start,
						resource_size(rentry->res));
				if (sys_start == -1) {
					dev_err(&adev->dev, "FAIL:convert resource(%pR) to system IO\n",
					        rentry->res);
					return ERR_PTR(-ENXIO);
				}
				rentry->res->start = sys_start;
				rentry->res->end = sys_start +
					resource_size(rentry->res) - 1;
			}
			acpi_platform_fill_resource(adev, rentry->res,
				&resources[count++]);
		}
		acpi_dev_free_resource_list(&resource_list);
	}

	memset(&pdevinfo, 0, sizeof(pdevinfo));
	/*
	 * If the ACPI node has a parent and that parent has a physical device
	 * attached to it, that physical device should be the parent of the
	 * platform device we are about to create.
	 */
	pdevinfo.parent = adev->parent ?
		acpi_get_first_physical_node(adev->parent) : NULL;
	pdevinfo.name = dev_name(&adev->dev);
	pdevinfo.id = -1;
	pdevinfo.res = resources;
	pdevinfo.num_res = count;
	pdevinfo.fwnode = acpi_fwnode_handle(adev);
	pdevinfo.properties = properties;

	if (acpi_dma_supported(adev))
		pdevinfo.dma_mask = DMA_BIT_MASK(32);
	else
		pdevinfo.dma_mask = 0;

	pdev = platform_device_register_full(&pdevinfo);
	if (IS_ERR(pdev))
		dev_err(&adev->dev, "platform device creation failed: %ld\n",
			PTR_ERR(pdev));
	else
		dev_dbg(&adev->dev, "created platform device %s\n",
			dev_name(&pdev->dev));

	kfree(resources);
	return pdev;
}



/*
 * hisilpc_probe - the probe callback function for hisi lpc device,
 *		will finish all the intialization.
 * @pdev: the platform device corresponding to hisi lpc
 *
 * Returns 0 on success, non-zero on fail.
 *
 */
static int hisilpc_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *iores;
	struct hisilpc_dev *lpcdev;

	dev_info(&pdev->dev, "probing hisilpc...\n");

	/* For ACPI mode, the alloc is redundant */
	lpcdev = devm_kzalloc(&pdev->dev,
		sizeof(*lpcdev) + sizeof(struct extio_bus_res),
		GFP_KERNEL);
	if (!lpcdev)
		return -ENOMEM;

	spin_lock_init(&lpcdev->cycle_lock);
	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lpcdev->membase = devm_ioremap_resource(&pdev->dev, iores);
	if (IS_ERR(lpcdev->membase)) {
		dev_err(&pdev->dev, "ioremap memory FAIL(%ld)!\n",
				(-PTR_ERR(lpcdev->membase)));
		return PTR_ERR(lpcdev->membase);
	}

	lpcdev->io_ent = (struct extio_bus_res *)((u8 *)lpcdev +
			sizeof(struct hisilpc_dev));
	/*
	 * During the bus/device scanning, only LPC bus will be created.
	 * But as indirectIO LPC bus is without 'ranges' or IO window, will
	 * not request to register any linux virtual IO. So the indirectIO
	 * root window can be registered here.
	 */
	ret = pci_register_io_range(IO_RANGE_IOEXT, LEGACY_IO_TSIZE);
	if (ret) {
		dev_err(&pdev->dev, "register whole legacyIO FAIL!\n");
		return ret;
	}
	/* register the bus IO window. */
	spin_lock_init(&lpcdev->io_ent->child_lock);
	INIT_LIST_HEAD(&lpcdev->io_ent->child_head);
	ret = register_bus_extio_range(pdev->dev.fwnode,
			lpcdev->io_ent);
	if (ret) {
		dev_err(&pdev->dev, "Allocate bus I/O FAIL(%d)\n",
				-ret);
		return ret;
	}

	/*
	 * For ACPI, all the children under indirectIO bus are delayed
	 * the scanning during the early booting. Start here...
	 */
	if (has_acpi_companion(&pdev->dev)) {
		struct acpi_device *root, *child;

		root = to_acpi_device_node(pdev->dev.fwnode);
		if (!root)
			return -ENODEV;
		list_for_each_entry(child, &root->children, node) {
			struct platform_device *tmpdev;

			tmpdev = acpi_extio_reg_platform_dev(child, NULL);
			if (!tmpdev)
				return -ENODEV;
		}
	} else {
	/*
	 * For OF children, the scanning hasn't started during the platform
	 * bus scanning. Here will start the scanning.
	 */
		ret = of_platform_populate(pdev->dev.of_node, NULL, NULL,
				&pdev->dev);
		if (ret) {
			dev_err(&pdev->dev, "scan children FAIL(%d)\n",
				-ret);
			return ret;
		}
	}

	/* must insert the child indirectIO resource to the tree */
	ret = device_for_each_child(&pdev->dev, &lpcdev->io_ent->iowin,
			hisilpc_child_io_insert);
	if (ret)
		return ret;


	lpcdev->io_ent->ops.devpara = lpcdev;
	lpcdev->io_ent->ops.pfin = hisilpc_comm_in;
	lpcdev->io_ent->ops.pfout = hisilpc_comm_out;
	lpcdev->io_ent->ops.pfins = hisilpc_comm_ins;
	lpcdev->io_ent->ops.pfouts = hisilpc_comm_outs;

	platform_set_drvdata(pdev, lpcdev);

	return ret;
}

static const struct of_device_id hisilpc_of_match[] = {
	{
		.compatible = "hisilicon,hip06-lpc",
	},
	{},
};

static const struct acpi_device_id hisilpc_acpi_match[] = {
	{"HISI0191",},
	{},
};

static struct platform_driver hisilpc_driver = {
	.driver = {
		.name           = "hisi_lpc",
		.of_match_table = hisilpc_of_match,
		.acpi_match_table = hisilpc_acpi_match,
	},
	.probe = hisilpc_probe,
};


builtin_platform_driver(hisilpc_driver);
