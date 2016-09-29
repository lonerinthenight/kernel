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

#include <linux/acpi.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/serial_8250.h>
#include <linux/slab.h>


struct hisilpc_dev;

/* This flag is specific to differentiate earlycon operations and the others */
#define FG_EARLYCON_LPC		0x0001
/*
 * this bit set means each IO operation will target to different port address;
 * 0 means repeatly IO operations will be sticked on the same port, such as BT;
 */
#define FG_INCRADDR_LPC		0x0002

struct lpc_cycle_para {
	unsigned int opflags;
	unsigned int csize;/*the data length of each operation*/
};

struct hisilpc_dev {
	spinlock_t cycle_lock;
	void __iomem  *membase;
	struct platform_device *pltdev;
};


/* The maximum continous operations*/
#define LPC_MAX_OPCNT	16
/* only support IO data unit length is four at maximum */
#define LPC_MAX_DULEN	4
#if LPC_MAX_DULEN > LPC_MAX_OPCNT
#error "LPC.. MAX_DULEN must be not bigger than MAX_OPCNT!"
#endif

#define LPC_REG_START		(0x00)/*start a new LPC cycle*/
#define LPC_REG_OP_STATUS	(0x04)/*the current LPC status*/
#define LPC_REG_IRQ_ST		(0x08)/*interrupt enable&status*/
#define LPC_REG_OP_LEN		(0x10)/*how many LPC cycles each start*/
#define LPC_REG_CMD		(0x14)/*command for the required LPC cycle*/
#define LPC_REG_ADDR		(0x20)/*LPC target address*/
#define LPC_REG_WDATA		(0x24)/*data to be written*/
#define LPC_REG_RDATA		(0x28)/*data coming from peer*/


/* The command register fields*/
#define LPC_CMD_SAMEADDR_SING	(0x00000008)
#define LPC_CMD_SAMEADDR_INC	(0x00000000)
#define LPC_CMD_TYPE_IO		(0x00000000)
#define LPC_CMD_TYPE_MEM	(0x00000002)
#define LPC_CMD_TYPE_FWH	(0x00000004)
#define LPC_CMD_WRITE		(0x00000001)
#define LPC_CMD_READ		(0x00000000)

#define LPC_IRQ_CLEAR		(0x02)
#define LPC_IRQ_OCCURRED	(0x02)

#define LPC_STATUS_IDLE		(0x01)
#define LPC_OP_FINISHED		(0x02)

#define START_WORK		(0x01)

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
	u32 opstatus = 0;

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
static int hisilpc_target_in(struct hisilpc_dev *pdev,
				struct lpc_cycle_para *para,
				unsigned long ptaddr, unsigned char *buf,
				unsigned long opcnt)
{
	unsigned int cmd_word;
	unsigned int waitcnt;
	int retval;
	/*initialized as 0 to remove compile warning */
	unsigned long flags = 0;
	unsigned long cnt_per_trans;

	if (!buf || !opcnt || !para || !para->csize || !pdev)
		return -EINVAL;

	if (opcnt  > LPC_MAX_OPCNT)
		return -EINVAL;

	cmd_word = LPC_CMD_TYPE_IO | LPC_CMD_READ;
	waitcnt = (LPC_PEROP_WAITCNT);
	if (!(para->opflags & FG_INCRADDR_LPC)) {
		cmd_word |= LPC_CMD_SAMEADDR_SING;
		waitcnt = LPC_MAX_WAITCNT;
	}

	retval = 0;
	cnt_per_trans = (para->csize == 1) ? opcnt : para->csize;
	for (; opcnt && !retval; cnt_per_trans = para->csize) {
		/* whole operation must be atomic */
		if (!(para->opflags & FG_EARLYCON_LPC))
			spin_lock_irqsave(&pdev->cycle_lock, flags);

		writel(cnt_per_trans, pdev->membase + LPC_REG_OP_LEN);

		writel(cmd_word, pdev->membase + LPC_REG_CMD);

		writel(ptaddr, pdev->membase + LPC_REG_ADDR);

		writel(START_WORK, pdev->membase + LPC_REG_START);

		/* whether the operation is finished */
		retval = wait_lpc_idle(pdev->membase, waitcnt);
		if (!retval) {
			opcnt -= cnt_per_trans;
			for (; cnt_per_trans--; buf++)
				*buf = readl(pdev->membase + LPC_REG_RDATA);
		}

		if (!(para->opflags & FG_EARLYCON_LPC))
			spin_unlock_irqrestore(&pdev->cycle_lock, flags);
	}

	return retval;
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
static int hisilpc_target_out(struct hisilpc_dev *pdev,
				struct lpc_cycle_para *para,
				unsigned long ptaddr,
				const unsigned char *buf,
				unsigned long opcnt)
{
	unsigned int cmd_word;
	unsigned int waitcnt;
	int retval;
	/* initialized as 0 to remove compile warning */
	unsigned long flags = 0;
	unsigned long cnt_per_trans;

	if (!buf || !opcnt || !para || !pdev)
		return -EINVAL;

	if (opcnt > LPC_MAX_OPCNT)
		return -EINVAL;
	/* default is increasing address */
	cmd_word = LPC_CMD_TYPE_IO | LPC_CMD_WRITE;
	waitcnt = (LPC_PEROP_WAITCNT);
	if (!(para->opflags & FG_INCRADDR_LPC)) {
		cmd_word |= LPC_CMD_SAMEADDR_SING;
		waitcnt = LPC_MAX_WAITCNT;
	}

	retval = 0;
	cnt_per_trans = (para->csize == 1) ? opcnt : para->csize;
	for (; opcnt && !retval; cnt_per_trans = para->csize) {
		/* whole operation must be atomic */
		if (!(para->opflags & FG_EARLYCON_LPC))
			spin_lock_irqsave(&pdev->cycle_lock, flags);

		writel(cnt_per_trans, pdev->membase + LPC_REG_OP_LEN);
		opcnt -= cnt_per_trans;
		for (; cnt_per_trans--; buf++)
			writel(*buf, pdev->membase + LPC_REG_WDATA);

		writel(cmd_word, pdev->membase + LPC_REG_CMD);

		writel(ptaddr, pdev->membase + LPC_REG_ADDR);

		writel(START_WORK, pdev->membase + LPC_REG_START);

		/* whether the operation is finished */
		retval = wait_lpc_idle(pdev->membase, waitcnt);

		if (!(para->opflags & FG_EARLYCON_LPC))
			spin_unlock_irqrestore(&pdev->cycle_lock, flags);
	}

	return retval;
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
	BUG_ON(sizeof(rd_data) < dlen);
	newbuf = (unsigned char *)&rd_data;

	lpcdev = (struct hisilpc_dev *)devobj;

	iopara.opflags = FG_INCRADDR_LPC;
	iopara.csize = dlen;

	ret = hisilpc_target_in(lpcdev, &iopara, ptaddr, newbuf, dlen);
	if (ret)
		return -1;

	dev_dbg(&lpcdev->pltdev->dev, "In-IO(0x%lx), dlen=%zu retdata=0x%0x ~ %0x\n",
		ptaddr, dlen, rd_data, le32_to_cpu(rd_data));

	return le32_to_cpu(rd_data);
}

/**
 * hisilpc_comm_out - write/output the data whose maximal length is four bytes
 *			to the I/O peripheral through LPC.
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

	BUG_ON(sizeof(outval) < dlen);
	outval = cpu_to_le32(outval);

	newbuf = (const unsigned char *)&outval;
	lpcdev = (struct hisilpc_dev *)devobj;

	dev_dbg(&lpcdev->pltdev->dev, "Out-IO(0x%lx), dlen=%zu wr_val=0x%0x\n",
				ptaddr,	dlen, outval);

	iopara.opflags = FG_INCRADDR_LPC;
	iopara.csize = dlen;

	(void)hisilpc_target_out(lpcdev, &iopara, ptaddr, newbuf, dlen);
}


/**
 * hisilpc_comm_ins - read/input the data in buffer to the I/O peripheral
 *		    through LPC, it corresponds to ins(b,w,l)
 * @devobj: pointer to the device information relevant to LPC controller.
 * @ptaddr: the target I/O port address.
 * @inbuf: a buffer where read/input data bytes are stored.
 * @dlen: the data length required writing to the target I/O port .
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
	/* ensure data stream whose lenght is multiply of dlen each IO input */
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

/**
 * hisilpc_acpi_reg_pio - scan all hisi-lpc children to setup the extio range.
 *			It is only for ACPI devices.
 *
 * @ppdev: the platform device of hisi lpc
 *
 * Returns 0 on success, non-zero on fail.
 *
 */
static int hisilpc_acpi_reg_pio(struct device *ppdev)
{
	struct acpi_device *root;
	struct acpi_device *child;

	if (arm64_extio_ops->pfin) {
		dev_warn(ppdev, "had registered PIO before!\n");
		return 0;
	}

	root = ACPI_COMPANION(ppdev);
	if (!root)
		return -ENODEV;

	list_for_each_entry(child, &root->children, node) {
		struct list_head resource_list;
		struct resource_entry *rentry;

		INIT_LIST_HEAD(&resource_list);
		if (acpi_dev_get_resources(child, &resource_list, NULL, NULL)
				<= 0) {
			pr_err("child(%s):: No any resources!\n",
					dev_name(&child->dev));
			return -ENXIO;
		}

		list_for_each_entry(rentry, &resource_list, node) {
			struct resource *iores;

			iores = rentry->res;
			if ((iores->flags & IORESOURCE_TYPE_BITS) ==
				IORESOURCE_IO) {
				if (iores->start > iores->end ||
					iores->end > PCIBIOS_MIN_IO) {
					pr_err("%s:: IO resource %pR is out-range!\n",
						dev_name(&child->dev), iores);
					return -EFAULT;
				}

				if (!arm64_extio_ops->start ||
					arm64_extio_ops->start > iores->start)
					arm64_extio_ops->start = iores->start;
				if (!arm64_extio_ops->end ||
					arm64_extio_ops->end < iores->end)
					arm64_extio_ops->end = iores->end;
			}
		}
		acpi_dev_free_resource_list(&resource_list);
	}

	return 0;
}


/**
 * of_hisilpc_register_pio - get the IO range from the corresponding OF node
 *			of hisi-lpc, then initialize the extio range
 *
 * @dev: the child device of hisi lpc
 *
 * Returns 0 on success, non-zero on fail.
 *
 */
static int of_hisilpc_register_pio(struct device_node *dev)
{
	const __be32	*addrp;
	u64		size;
	unsigned int	flags;
	u64	taddr;
	int residx = 0;

	if (!dev || !dev->parent) {
		pr_err("%s:: register OF address with invalid parameter!\n",
			dev->name);
		return -EINVAL;
	}

	if (arm64_extio_ops->pfin) {
		pr_warn("%s:: had registered PIO before!\n", dev->name);
		return 0;
	}

	/* find the unique I/O resource of this child */
	do {
		addrp = of_get_address(dev, residx, &size, &flags);
		if (addrp == NULL) {
			pr_err("%s:: get OF address(%d) FAIL!\n",
				dev->name, residx);
			return -EINVAL;
		}
		residx++;
	} while (!(flags & IORESOURCE_IO));

	if (strcmp(dev->parent->name, "isa")) {
		pr_err("%s:: device type is not the mandatory ISA!\n",
				dev->name);
		return -EINVAL;
	}
	/* according to ISA binding spec, first 32bit is address flags */
	taddr = of_read_number(addrp + 1, 1);

	if (taddr >= PCIBIOS_MIN_IO || taddr + size > PCIBIOS_MIN_IO) {
		pr_err("%s:: range[0x%llx - %llx) is too big!\n", dev->name,
			taddr, taddr + size);
		return -EFAULT;
	}

	/* extend the IO range and store in the global range */
	if (!arm64_extio_ops->start || arm64_extio_ops->start > taddr)
		arm64_extio_ops->start = taddr;
	if (!arm64_extio_ops->end || arm64_extio_ops->end < (taddr + size - 1))
		arm64_extio_ops->end = taddr + size - 1;

	return 0;
}

/**
 * hisilpc_probe_child_ofdev - create the children devices and setup
 *			the IO range of	hisi-lpc bus. It is only for
 *			OF devices.
 *
 * @ppdev: the platform device corresponding to hisi lpc
 *
 * Returns 0 on success, non-zero on fail.
 *
 */
static int hisilpc_probe_child_ofdev(struct device *ppdev)
{
	int ret;
	struct device_node *root, *child;

	if (!ppdev || !ppdev->of_node) {
		dev_err(ppdev, "invalid OF node!\n");
		return -EINVAL;
	}

	root = ppdev->of_node;
	for_each_available_child_of_node(root, child) {
		ret = of_hisilpc_register_pio(child);
		if (ret) {
			dev_err(ppdev, "fail to register raw IO for child(%s)\n",
					child->name);
			return ret;
		}

		if (!of_platform_device_create(child, NULL, ppdev)) {
			dev_err(ppdev, "create platform device fail for %s\n",
				child->name);
			return -EFAULT;
		}
	}

	return 0;
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
	struct resource *iores;
	struct hisilpc_dev *lpcdev;
	int ret;

	dev_info(&pdev->dev, "hslpc start probing...\n");

	lpcdev = devm_kzalloc(&pdev->dev,
				sizeof(struct hisilpc_dev), GFP_KERNEL);
	if (!lpcdev)
		return -ENOMEM;

	spin_lock_init(&lpcdev->cycle_lock);

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lpcdev->membase = devm_ioremap_resource(&pdev->dev, iores);
	if (IS_ERR(lpcdev->membase)) {
		dev_err(&pdev->dev, "No mem resource for memory mapping!\n");
		return PTR_ERR(lpcdev->membase);
	}

	lpcdev->pltdev = pdev;
	platform_set_drvdata(pdev, lpcdev);

	if (arm64_extio_ops) {
		dev_err(&pdev->dev, "extio had been initialized!\n");
		return -EBUSY;
	}
	arm64_extio_ops = devm_kzalloc(&pdev->dev, sizeof(*arm64_extio_ops),
				GFP_KERNEL);
	if (!arm64_extio_ops)
		return -ENOMEM;

	if (!has_acpi_companion(&pdev->dev))
		ret = hisilpc_probe_child_ofdev(&pdev->dev);
	else
		ret = hisilpc_acpi_reg_pio(&pdev->dev);
	if (!ret) {
		arm64_extio_ops->devpara = lpcdev;
		arm64_extio_ops->pfin = hisilpc_comm_in;
		arm64_extio_ops->pfout = hisilpc_comm_out;
		arm64_extio_ops->pfins = hisilpc_comm_ins;
		arm64_extio_ops->pfouts = hisilpc_comm_outs;
		barrier();
		dev_info(&pdev->dev, "hslpc end probing. range[0x%lx - %lx]\n",
			arm64_extio_ops->start, arm64_extio_ops->end);
	} else
		dev_err(&pdev->dev, "hslpc probe got fail(%d)!\n", -ret);

	return ret;
}


static const struct of_device_id hisilpc_of_match[] = {
	{
		.compatible = "hisilicon,low-pin-count",
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
