/*
 * ACPI support for lpc bus based on indirect IO.
 *
 * Copyright (C) 2016 Hisilicon Limited, All Rights Reserved.
 * Author: Zhichang Yuan <yuanzhichang@hisilicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <linux/extio.h>

#include "internal.h"

ACPI_MODULE_NAME("leagcy IO");

/*
 * all the matched acpi devices will delay the creation of the physical
 * devices.
 */
static const struct acpi_device_id acpi_legacyio_id[] = {
	{"HISI0191",},
	{""},
};

static int acpi_legacyio_attach(struct acpi_device *adev,
				const struct acpi_device_id *id)
{
	int ret;
	struct extio_bus_res *range;
	struct platform_device *pdev;

	/* create bus device as platform device */
	pdev = acpi_create_platform_device(adev, NULL);
	if (!pdev) {
		pr_err(PREFIX "create legacy IO platform device FAIL!\n");
		return -EFAULT;
	}
	acpi_device_set_enumerated(adev);

	ret = pci_register_io_range(IO_RANGE_IOEXT, LEGACY_IO_TSIZE);
	if (ret) {
		pr_err("register whole legacy IO FAIL!\n");
		return ret;
	}
	/* create the bus IO window. */
	range = devm_kzalloc(&pdev->dev, sizeof(struct extio_bus_res),
			GFP_KERNEL);
	if (!range)
		return -ENOMEM;
	ret = register_bus_extio_range(&adev->fwnode, range);
	if (ret) {
		dev_err(&pdev->dev, "Allocate bus I/O FAIL(%d)\n", -ret);
		return ret;
	}
	adev->driver_data = range;

	return 1;
}


static struct acpi_scan_handler acpi_legacyio_handler = {
	.ids = acpi_legacyio_id,
	.attach = acpi_legacyio_attach,
};

bool acpi_is_legacyio_device(struct acpi_device *adev)
{
	return adev->handler == &acpi_legacyio_handler;
}
EXPORT_SYMBOL_GPL(acpi_is_legacyio_device);

void __init acpi_legacyio_scan_init(void)
{
	acpi_scan_add_handler(&acpi_legacyio_handler);
}
