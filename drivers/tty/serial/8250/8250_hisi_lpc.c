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


static int hslpc8250_probe(struct platform_device *pdev)
{
	struct uart_8250_port uart = {};
	struct uart_port *port = &uart.port;
	int err = 0;
	struct resource *iores;

	if (!pdev->dev.parent)
		return -ENODEV;
	dev_info(&pdev->dev, "##probe entering\n");

	iores = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!iores) {
		dev_err(&pdev->dev, "can not find the IO0\n");
		return -ENXIO;
	}
	port->iobase = iores->start;

	port->irq	= 0;
	port->flags	= UPF_BOOT_AUTOCONF | UPF_FIXED_PORT;
	port->dev	= &pdev->dev;
	port->iotype	= UPIO_PORT;
	port->regshift	= 0;
	port->uartclk	= BASE_BAUD * 16;

	spin_lock_init(&port->lock);

	err = serial8250_register_8250_port(&uart);
	if (err < 0) {
		dev_err(&pdev->dev, "register uart FAIL(%d)!\n", -err);
		return err;
	}

	platform_set_drvdata(pdev, (void *)&err);
	dev_info(&pdev->dev, "##probing OK(%d)\n", err);
	return 0;
}

static int hslpc8250_remove(struct platform_device *pdev)
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
	.probe			= hslpc8250_probe,
	.remove			= hslpc8250_remove,
};

module_platform_driver(hs_lpc8250_driver);


MODULE_AUTHOR("Rongrong Zou");
MODULE_DESCRIPTION("8250 serial probe module for Hisilicon LPC UART");
MODULE_LICENSE("GPL");
MODULE_VERSION("v1.0");
