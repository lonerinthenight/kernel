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
#include <linux/serial_8250.h>
#include <linux/slab.h>


struct hisilpc_dev;

/* This flag is specific to differentiate earlycon operations and the others */
#define FG_EARLYCON_LPC		(0x01U << 0)
/*
 * this bit set means each IO operation will target to different port address;
 * 0 means repeatly IO operations will be sticked on the same port, such as BT;
 */
#define FG_INCRADDR_LPC		(0x01U << 1)

struct lpc_cycle_para {
	unsigned int opflags;
	unsigned int csize;/*the data length of each operation*/
};

struct lpc_io_ops {
	unsigned int periosz;
	int (*lpc_iord)(struct hisilpc_dev *pdev, struct lpc_cycle_para *para,
				unsigned long ptaddr, unsigned char *buf,
				unsigned long dlen);
	int (*lpc_iowr)(struct hisilpc_dev *pdev, struct lpc_cycle_para *para,
				unsigned long ptaddr,
				const unsigned char *buf,
				unsigned long dlen);
};

struct hisilpc_dev {
	spinlock_t cycle_lock;
	void __iomem  *membase;
	struct extio_ops *io_ops;
	struct platform_device *pltdev;
	struct lpc_io_ops target_io;
};


/* The maximum continous operations*/
#define LPC_MAX_OPCNT	16

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


struct extio_ops *arm64_simops __refdata;


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

	if (!buf || !opcnt || !para || !pdev)
		return -EINVAL;

	if (para->csize != 1 || opcnt  > LPC_MAX_OPCNT)
		return -EINVAL;

	cmd_word = LPC_CMD_TYPE_IO | LPC_CMD_READ;
	waitcnt = (LPC_PEROP_WAITCNT);
	if (!(para->opflags & FG_INCRADDR_LPC)) {
		cmd_word |= LPC_CMD_SAMEADDR_SING;
		waitcnt = LPC_MAX_WAITCNT;
	}

	/* whole operation must be atomic */
	if (!(para->opflags & FG_EARLYCON_LPC))
		spin_lock_irqsave(&pdev->cycle_lock, flags);

	writel(opcnt, pdev->membase + LPC_REG_OP_LEN);

	writel(cmd_word, pdev->membase + LPC_REG_CMD);

	writel(ptaddr, pdev->membase + LPC_REG_ADDR);

	writel(START_WORK, pdev->membase + LPC_REG_START);

	/* whether the operation is finished */
	retval = wait_lpc_idle(pdev->membase, waitcnt);
	if (!retval) {
		for (; opcnt--; buf++)
			*buf = readl(pdev->membase + LPC_REG_RDATA);
	}

	if (!(para->opflags & FG_EARLYCON_LPC))
		spin_unlock_irqrestore(&pdev->cycle_lock, flags);

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


	if (!buf || !opcnt || !para || !pdev)
		return -EINVAL;

	if (para->csize != 1 || opcnt  > LPC_MAX_OPCNT)
		return -EINVAL;

	cmd_word = LPC_CMD_TYPE_IO | LPC_CMD_WRITE;
	waitcnt = (LPC_PEROP_WAITCNT);
	if (!(para->opflags & FG_INCRADDR_LPC)) {
		cmd_word |= LPC_CMD_SAMEADDR_SING;
		waitcnt = LPC_MAX_WAITCNT;
	}

	/* whole operation must be atomic */
	if (!(para->opflags & FG_EARLYCON_LPC))
		spin_lock_irqsave(&pdev->cycle_lock, flags);

	writel(opcnt, pdev->membase + LPC_REG_OP_LEN);
	for (; opcnt--; buf++)
		writel(*buf, pdev->membase + LPC_REG_WDATA);

	writel(cmd_word, pdev->membase + LPC_REG_CMD);

	writel(ptaddr, pdev->membase + LPC_REG_ADDR);

	writel(START_WORK, pdev->membase + LPC_REG_START);

	/* whether the operation is finished */
	retval = wait_lpc_idle(pdev->membase, waitcnt);

	if (!(para->opflags & FG_EARLYCON_LPC))
		spin_unlock_irqrestore(&pdev->cycle_lock, flags);

	return retval;
}

/**
 * hisilpc_comm_inb - read/input the data from the I/O peripheral through LPC.
 * @devobj: pointer to the device information relevant to LPC controller.
 * @outbuf: a buffer where the data read is stored at.
 * @ptaddr: the target I/O port address.
 * @dlen: the data length required to read from the target I/O port.
 * @count: how many I/O operations required in this calling.  >1 is for ins.
 *
 * For this lpc, only support inb/insb now.
 *
 * For inbs, returns 0 on success, -1 on fail.
 * when succeed, the data read back is stored in buffer pointed by inbuf.
 * For inb, return the data read from I/O or -1 when error occur.
 */
u64 hisilpc_comm_inb(void *devobj, unsigned long ptaddr,
				void *inbuf, size_t dlen,
				unsigned int count)
{
	struct hisilpc_dev *lpcdev;
	struct lpc_cycle_para iopara;
	unsigned int loopcnt, cntleft;
	unsigned int rd_data;
	unsigned char *newbuf;
	int ret = 0;
	/* only support data unit length is 1 now... */
	if (!count || (!inbuf && count != 1) || !devobj || dlen != 1)
		return -1;

	newbuf = (unsigned char *)inbuf;
	/*
	 * the operation data len is 4 bytes, need to ensure the buffer
	 * is big enough.
	 */
	if (!inbuf || count < sizeof(u32))
		newbuf = (unsigned char *)&rd_data;

	lpcdev = (struct hisilpc_dev *)devobj;
	dev_dbg(&lpcdev->pltdev->dev, "In-IO(0x%lx), count=%u\n", ptaddr,
			count);

	iopara.opflags = FG_INCRADDR_LPC;
	/*
	 * to improve performance,  support repeatly rd at same target
	 * address.
	 */
	if (count > 1)
		iopara.opflags &= ~FG_INCRADDR_LPC;

	iopara.csize = dlen;

	cntleft = count;
	do {
		loopcnt = (cntleft > LPC_MAX_OPCNT) ? LPC_MAX_OPCNT : cntleft;
		ret = lpcdev->target_io.lpc_iord(lpcdev,
				&iopara, ptaddr, newbuf, loopcnt);
		if (ret)
			return -1;
		newbuf += loopcnt;
		cntleft -= loopcnt;
	} while (cntleft);

	/* for inb */
	if (!inbuf)
		return rd_data;
	/* for insb, copy the data to the return variable */
	if (inbuf != newbuf)
		memcpy(inbuf, &rd_data, count);

	return 0;
}

/**
 * hisilpc_comm_outb - write/output the data in out buffer to the I/O peripheral
 *		    through LPC.
 * @devobj: pointer to the device information relevant to LPC controller.
 * @outbuf: a buffer where the data to be written is stored.
 * @ptaddr: the target I/O port address.
 * @dlen: the data length required writing to the target I/O port .
 * @count: how many I/O operations required in this calling. >1 is for outs.
 *
 * For this lpc, only support outb/outsb now.
 *
 */
void hisilpc_comm_outb(void *devobj, unsigned long ptaddr,
				const void *outbuf, size_t dlen,
				unsigned int count)
{
	struct hisilpc_dev *lpcdev;
	struct lpc_cycle_para iopara;
	unsigned int loopcnt;
	const unsigned char *newbuf;
	int ret = 0;

	if (!count || !outbuf || !devobj)
		return;

	newbuf = (const unsigned char *)outbuf;
	lpcdev = (struct hisilpc_dev *)devobj;

	dev_dbg(&lpcdev->pltdev->dev, "Out-IO(0x%lx), cnt=%u\n", ptaddr, count);

	iopara.opflags = FG_INCRADDR_LPC;
	/* to improve performance,  support repeatly wr same target address */
	if (count > 1)
		iopara.opflags &= ~FG_INCRADDR_LPC;

	iopara.csize = 1;

	do {
		loopcnt = (count > LPC_MAX_OPCNT) ? LPC_MAX_OPCNT : count;
		ret = lpcdev->target_io.lpc_iowr(lpcdev,
				&iopara, ptaddr, newbuf, loopcnt);
		if (ret)
			return;
		newbuf += loopcnt;
		count -= loopcnt;
	} while (count);
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

	dev_dbg(&pdev->dev, "hslpc start probing...\n");

	lpcdev = devm_kzalloc(&pdev->dev,
				sizeof(struct hisilpc_dev), GFP_KERNEL);
	if (!lpcdev)
		return -ENOMEM;

	spin_lock_init(&lpcdev->cycle_lock);

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lpcdev->membase = devm_ioremap_resource(&pdev->dev, iores);
	if (IS_ERR(lpcdev->membase)) {
		dev_err(&pdev->dev, "No mem resource\n");
		return PTR_ERR(lpcdev->membase);
	}

	lpcdev->target_io.periosz = 0x01;
	lpcdev->target_io.lpc_iord = hisilpc_target_in;
	lpcdev->target_io.lpc_iowr = hisilpc_target_out;


	lpcdev->pltdev = pdev;
	platform_set_drvdata(pdev, lpcdev);

	lpcdev->io_ops = devm_kzalloc(&pdev->dev,
					sizeof(*lpcdev->io_ops),
					GFP_KERNEL);
	if (!lpcdev->io_ops) {
		dev_err(&pdev->dev, "memory allocate fail!\n");
		return -ENOMEM;
	}

	lpcdev->io_ops->pfin = hisilpc_comm_inb;
	lpcdev->io_ops->pfout = hisilpc_comm_outb;
	lpcdev->io_ops->devpara = (void *)lpcdev;

	if (!has_acpi_companion(&pdev->dev)) {
		struct device_node *root, *child;

		root = pdev->dev.of_node;
		for_each_available_child_of_node(root, child) {
			if (!of_platform_device_create(child, NULL, &pdev->dev)) {
				dev_err(&pdev->dev, "create platform device fail for %s\n",
						child->name);
				return -EFAULT;
			}
			dev_info(&pdev->dev, "create platform device OK for %s\n",
					child->name);
		}
	}

	/*should lock the earlycon callings at first*/
	console_lock();
	arm64_set_simops(lpcdev->io_ops);
	console_unlock();

	dev_dbg(&pdev->dev, "hslpc finish probing... extio_ops = %p\n",
				arm64_simops);

	return 0;
}

/**
 * hisilpc_remove - the remove callback function for hisi lpc device.
 * @pdev: the platform device corresponding to hisi lpc that is to be removed.
 *
 * Returns 0 on success, non-zero on fail.
 *
 */
static int hisilpc_remove(struct platform_device *pdev)
{
	return 0;
}


static const struct of_device_id hisilpc_of_match[] = {
	{
		.compatible = "hisilicon,low-pin-count",
	},
	{},
};
MODULE_DEVICE_TABLE(of, hisilpc_of_match);

static const struct acpi_device_id hisilpc_acpi_match[] = {
	{"HISI0191", },
	{},
};

MODULE_DEVICE_TABLE(acpi, hisilpc_acpi_match);

static struct platform_driver hisilpc_driver = {
	.driver = {
		.name           = "hisi_lpc",
		.of_match_table = hisilpc_of_match,
		.acpi_match_table = hisilpc_acpi_match,
	},
	.probe = hisilpc_probe,
	.remove = hisilpc_remove,
};


module_platform_driver(hisilpc_driver);

MODULE_AUTHOR("Zhichang Yuan");
MODULE_DESCRIPTION("The LPC driver for Hip06 SoC based on indirect-IO");
MODULE_LICENSE("GPL");
MODULE_VERSION("v1.0");
