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

#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/serial_8250.h>
#include <asm-generic/serial.h>
#include <linux/of_address.h>


/**
 * hisilpc_serial_inb - read/input data from the designated serial port.
 * @p: the serial port where the data read from
 * @offset:  the target I/O port address where the read is from
 *
 * Returns the byte data from this serial port.
 * -1 means some failures.
 *
 */
static unsigned int hisilpc_serial_inb(struct uart_port *p, int offset)
{
	struct extio_ops *parentops;

	parentops = p->private_data;
	if (!parentops || !parentops->pfin)
		return -1;

	return parentops->pfin(parentops->devpara,
				p->iobase + (offset << p->regshift),
				NULL, sizeof(u8), 1);
}

/**
 * hisilpc_serial_outb - write/output data from the designated serial port.
 * @p: the serial port where the data is written to
 * @offset:  the target I/O port address where the write is from
 *
 */
static void hisilpc_serial_outb(struct uart_port *p, int offset, int value)
{
	struct extio_ops *parentops;

	parentops = p->private_data;
	if (!parentops || !parentops->pfout)
		return;

	parentops->pfout(parentops->devpara,
				p->iobase + (offset << p->regshift),
				&value, sizeof(u8), 1);
}


static int hisilpc8250_probe(struct platform_device *pdev)
{
	struct uart_8250_port uart = {};
	struct uart_port *port = &uart.port;
	int err = 0;
	struct resource *iores;
	struct extio_ops *platdata;

	if (!pdev->dev.parent)
		return -ENODEV;
	dev_info(&pdev->dev, "##probe entering\n");

	/* To support the earlycon in bootargs, the first reg must be MEM */
	iores = platform_get_resource_byname(pdev, IORESOURCE_IO,
						"dev_io");
	if (!iores) {
		dev_err(&pdev->dev, "can not find the IO0\n");
		return -ENXIO;
	}

	/*
	 * save the platform data from parent in uart_port for serial_in,
	 * serial_out
	 */
	platdata = dev_get_platdata(&pdev->dev);
	port->private_data = (void *)platdata;
	if (!port->private_data) {
		dev_err(&pdev->dev, "no platform data!\n");
		return -ENODEV;
	}

	if (platdata->start != iores->start || platdata->end != iores->end) {
		dev_err(&pdev->dev, "PIO range[0x%lx - %lx] isn't fit!\n",
			(unsigned long)iores->start,
			(unsigned long)iores->end);
		return -ENXIO;
	}
	port->iobase = (unsigned long)iores->start + platdata->ptoffset;
	dev_info(&pdev->dev, "real port start is 0x%lx\n", port->iobase);

	port->irq	= 0;
	port->flags	= UPF_BOOT_AUTOCONF | UPF_FIXED_PORT;
	port->dev	= &pdev->dev;
	port->iotype	= UPIO_PORT;
	port->regshift	= 0;
	port->uartclk	= BASE_BAUD * 16;

	spin_lock_init(&port->lock);

	port->serial_in = hisilpc_serial_inb;
	port->serial_out = hisilpc_serial_outb;

	err = serial8250_register_8250_port(&uart);
	if (err < 0) {
		dev_err(&pdev->dev, "register uart FAIL(%d)!\n", -err);
		return err;
	}

	platform_set_drvdata(pdev, (void *)&err);
	dev_info(&pdev->dev, "##probing OK(%d)\n", err);
	return 0;
}

static int hisilpc8250_remove(struct platform_device *pdev)
{
	int line = *((int *)platform_get_drvdata(pdev));

	serial8250_unregister_port(line);

	return 0;
}


static const struct of_device_id hs8250_of_match[] = {
	{ .compatible = "hisilicon,lpc-uart" },
	{ }
};
MODULE_DEVICE_TABLE(of, hs8250_of_match);

static const struct acpi_device_id hs8250_acpi_match[] = {
	/*{ "PNP0501", 0 },*/
	{ "HISI1031", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, hs8250_acpi_match);

static struct platform_driver hs_lpc8250_driver = {
	.driver = {
		.name		= "hisi-lpc-uart",
		.of_match_table	= hs8250_of_match,
		.acpi_match_table = ACPI_PTR(hs8250_acpi_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe			= hisilpc8250_probe,
	.remove			= hisilpc8250_remove,
};

module_platform_driver(hs_lpc8250_driver);


MODULE_AUTHOR("Rongrong Zou");
MODULE_DESCRIPTION("8250 serial probe module for Hisilicon LPC UART");
MODULE_LICENSE("GPL");
MODULE_VERSION("v1.0");
