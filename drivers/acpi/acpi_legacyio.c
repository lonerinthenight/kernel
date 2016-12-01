/*
 * ACPI support for lpc bus.
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
/*static extio_range *acpi_io_ent;*/
static const struct acpi_device_id acpi_legacyio_id[] = {
	/*{"HISI0191", ((unsigned long *)&acpi_io_ent),},*/
	{"HISI0191",},
	{""},
};

#if 0
static int acpi_legacy_io_cal(struct acpi_device *parent,
			resource_size_t *sum, resource_size_t *io_min)
{
	int ret;
	resource_size_t io_max;
	struct acpi_device *child;
	struct resource_entry *entry;
	struct list_head resource_list;

	list_for_each_entry(child, &parent->children, node) {
		INIT_LIST_HEAD(&resource_list);
		ret = acpi_dev_get_resources(child, &resource_list,
					     acpi_dev_filter_resource_type_cb,
					     (void *)IORESOURCE_IO);
		if (ret < 0) {
			dev_warn(&child->dev, "failed to parse IO _CRS method, error code %d\n",
				ret);
			return ret;
		}
		if (ret == 0) {
			dev_warn(&child->dev, "No IO resources in _CRS\n");
			return -EFAULT;
		}

		resource_list_for_each_entry(entry, &resource_list) {
			if (*io_min > entry->res->start)
				*io_min = entry->res->start;
			if (io_max < entry->res->end)
				io_max = entry->res->end;
		}
		acpi_dev_free_resource_list(&resource_list);
	}

	*sum = io_max - *io_min + 1;
	return 0;
}

struct platform_device *acpi_legacyio_create(struct acpi_device *adev,
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

				sys_start = extio_translate(&adev->parent->fwnode, rentry->res->start);
				if (sys_start == -1) {
					dev_err(&adev->dev, "FAIL:convert resource(%pR) to system IO\n",
						rentry->res);
					return ERR_PTR(-ENXIO);
				}
				rentry->res->start = sys_start;
				rentry->res->end = sys_start +
					resource_size(rentry->res) - 1;
			}
			resources[count++] = *(rentry->res);
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
#endif

static int acpi_legacyio_attach(struct acpi_device *adev,
				const struct acpi_device_id *id)
{
	int ret;
	/*struct acpi_device *child;
	struct extio_range **range = (void *)id->driver_data;*/
	struct extio_range *range;
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
	range = devm_kzalloc(&pdev->dev, sizeof(struct extio_range),
			GFP_KERNEL);
	if (!range)
		return -ENOMEM;
	ret = register_extio_range(&adev->fwnode, range);
	if (ret) {
		dev_err(&pdev->dev, "Allocate bus I/O FAIL(%d)\n", -ret);
		return ret;
	}
	adev->driver_data = range;

	/*
	list_for_each_entry(child, &adev->children, node)
		child->handler = adev->handler;
	*/
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
