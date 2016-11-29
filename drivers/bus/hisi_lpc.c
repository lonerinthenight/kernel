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
#include <linux/extio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/serial_8250.h>
#include <linux/slab.h>

#define HISILPC_DEFAUL_ALIGN	0x01
#define HISILPC_IO_SIZE		0x400

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
	/*struct extio_node node_ent;*/
	struct extio_ops io_ops;
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

/**
 * hisilpc_target_in - trigger a series of lpc cycles to read required data
 *		  from target periperal.
 * @pdev: pointer to hisi lpc device
 * @para: some paramerters used to control the lpc I/O operations
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

/**
 * hisilpc_target_out - trigger a series of lpc cycles to write required data
 *		  to target periperal.
 * @pdev: pointer to hisi lpc device
 * @para: some paramerters used to control the lpc I/O operations
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

/**
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

/**
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

/**
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

/**
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

int hisilpc_cal_iosum(struct device *dev, resource_size_t *sum,
		resource_size_t *io_min)
{
	u64 start;
	resource_size_t io_max;

	if (!dev || !dev->fwnode || !sum || !io_min) {
		pr_err("hisilpc:: cal_iosum... invalid parameters!\n");
		return -EINVAL;
	}

	*sum = 0;
	*io_min = 0;
	io_max = 0;
	/* for OFT devices */
	if (is_of_node(dev->fwnode)) {
		struct device_node *parent, *child;

		parent = to_of_node(dev->fwnode);
		for_each_child_of_node(parent, child) {
			u64 size;
			const __be32	*addrp;
			unsigned int	flags;
			/*
			 * for hisilpc, only one 'reg' and without IO 'ranges'
			 * in the children.
			 */
			addrp = of_get_address(child, 0, &size, &flags);
			if (addrp == NULL)
				return -EINVAL;
			/*
			 * for hisilpc, the address stick to ISA binding
			 * spec.
			 */
			start = of_read_number(addrp + 1, 1);
			if (*io_min > (resource_size_t)start)
				*io_min = (resource_size_t)start;
			if (io_max <= start + size)
				io_max = start + size;
		}

		*sum = io_max - *io_min + 1;
		return 0;
	}

	/* for acpi devices */
	if (is_acpi_node(dev->fwnode)) {
		struct acpi_device *parent, *child;

		parent = to_acpi_device_node(dev->fwnode);
		list_for_each_entry(child, &parent->children, node) {
			int ret;
			struct resource_entry *entry;
			struct list_head resource_list;

			INIT_LIST_HEAD(&resource_list);
			ret = acpi_dev_get_resources(child, &resource_list,
						     acpi_dev_filter_resource_type_cb,
						     (void *)IORESOURCE_IO);
			if (ret < 0) {
				dev_warn(dev, "failed to parse IO _CRS method, error code %d\n",
					ret);
				return ret;
			}
			if (ret == 0) {
				dev_warn(dev, "No IO resources in _CRS\n");
				return -EFAULT;
			}

			resource_list_for_each_entry(entry, &resource_list) {
				start = entry->res->start;
				if (*io_min > start)
					*io_min = start;
				if (io_max < entry->res->end)
					io_max = entry->res->end;
			}
			acpi_dev_free_resource_list(&resource_list);
		}

		*sum = io_max - *io_min + 1;
		return 0;
	}

	/* don't support other device node */
	return -ENODEV;
}


int hisilpc_child_map_sysio(struct device *child, void *data)
{
	int ret;
	struct resource *res, *root;
	struct platform_device *pltdev;

	if (!child || !child->parent)
		return -EINVAL;

	pltdev = to_platform_device(child);
	/* Only one resource for hisi lpc */
	res = platform_get_resource(pltdev, IORESOURCE_IO, 0);
	if (!res) {
		dev_err(child, "No any IO!\n");
		return -ENXIO;
	}
#if 0
	/* For ACPI, need to convert device IO to system IO. */
	if (has_acpi_companion(child)) {
		unsigned long sys_start;

		sys_start = extio_translate(child->fwnode, res->start);
		if (sys_start == -1) {
			dev_err(child,
				"FAIL:convert resource(%pR) to system IO\n",
				res);
			return -ENXIO;
		}
		res->start = sys_start;
		res->end = sys_start + resource_size(res) - 1;
	}
#endif
	root = (struct resource *)data;
	ret = devm_request_resource(child, root, res);
	if (ret)
		dev_err(child, "request IO resource(%pR) FAIL(%d)!\n",
				res, -ret);

	return ret;
}

/**
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
	resource_size_t io_start;
	resource_size_t iosize;
	struct resource *iores;
	struct extio_range *range;
	struct hisilpc_dev *lpcdev;

	dev_info(&pdev->dev, "probing hslpc...\n");

	lpcdev = devm_kzalloc(&pdev->dev,
				sizeof(struct hisilpc_dev), GFP_KERNEL);
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

	/*
	 * For FDT mode, indirect IO registering must be done before
	 * scanning children since the 'reg' property parsing depends on
	 * this bus node in the extio list. For ACPI mode, the bus scanning
	 * doesn't depends on this node, but the linux PIO allocation for
	 * children need it.
	 */
	ret = hisilpc_cal_iosum(&pdev->dev, &iosize, &io_start);
	if (ret)
		return ret;

	if (!iosize) {
		dev_err(&pdev->dev, "No any I/O space requested!!\n");
		return -EFAULT;
	}

	range = register_extio_ranges(pdev->dev.fwnode, iosize, io_start);
	if (range) {
		dev_err(&pdev->dev, "register extio bus I/O FAIL!\n");
		return -EFAULT;
	}

	ret = devm_request_resource(&pdev->dev, &ioport_resource,
			range->io_host.res);
	if (ret) {
		dev_err(&pdev->dev, "request Root I/O(%pR) FAIL(%d)\n",
			range->io_host.res, -ret);
		return ret;
	}

	/*
	 * For OF children, the scanning hasn't started during the platform
	 * bus scanning. Here will start the scanning remained.
	 * For ACPI children, a special handler is added for our LPC
	 * controller to delay the children scanning. The real scanning
	 * will start here as it is easy to convert LPC I/O to linux PIO now.
	 */
	if (!has_acpi_companion(&pdev->dev)) {
#if 0
		struct device_node *parent, *child;

		parent = to_of_node(&pdev->dev.fwnode);
		for_each_child_of_node(parent, child) {
			if (!of_platform_device_create(child, NULL,
							&pdev->dev)) {
				dev_err(&child->dev, "OF:create child platform device FAIL!\n");
				return -EFAULT;
			}
		}
		of_node_set_flag(parent, OF_POPULATED_BUS);
#endif
		ret = of_platform_populate(pdev->dev.of_node, NULL, NULL,
				&pdev->dev);
		if (ret) {
			dev_err(&pdev->dev, "scan children FAIL(%d)\n",
				-ret);
			return ret;
		}

		/* request linux PIO ranges for children. probably is not needed*/
		ret = device_for_each_child(&pdev->dev, range->io_host.res,
				hisilpc_child_map_sysio);
		if (ret)
			return ret;
	} else {
		struct acpi_device *parent, *child;

		parent = to_acpi_device_node(pdev->dev.fwnode);
		list_for_each_entry(child, &parent->children, node) {
			if (acpi_legacyio_create(child, NULL)) {
				dev_err(&child->dev, "ACPI:create child platform device FAIL!\n");
				return -EFAULT;
			}
		}
	}

	lpcdev->io_ops.devpara = lpcdev;
	lpcdev->io_ops.pfin = hisilpc_comm_in;
	lpcdev->io_ops.pfout = hisilpc_comm_out;
	lpcdev->io_ops.pfins = hisilpc_comm_ins;
	lpcdev->io_ops.pfouts = hisilpc_comm_outs;

	range->ops = &lpcdev->io_ops;

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
	{"HISI0191", },
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
