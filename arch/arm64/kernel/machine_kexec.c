/*
 * kexec for arm64
 *
 * Copyright (C) Linaro.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/kexec.h>
#include <linux/of_fdt.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <asm/cacheflush.h>
#include <asm/system_misc.h>

/* TODO: Remove this include when KVM can support a kexec reboot. */
#include <asm/virt.h>

/* Global variables for the relocate_kernel routine. */
extern const unsigned char relocate_new_kernel[];
extern const unsigned long relocate_new_kernel_size;
extern unsigned long arm64_kexec_dtb_addr;
extern unsigned long arm64_kexec_kimage_head;
extern unsigned long arm64_kexec_kimage_start;

bool in_crash_kexec = false;

/**
 * kexec_is_dtb - Helper routine to check the device tree header signature.
 */
static bool kexec_is_dtb(const void *dtb)
{
	__be32 magic;

	return get_user(magic, (__be32 *)dtb) ? false :
		(be32_to_cpu(magic) == OF_DT_HEADER);
}

/**
 * kexec_image_info - For debugging output.
 */
#define kexec_image_info(_i) _kexec_image_info(__func__, __LINE__, _i)
static void _kexec_image_info(const char *func, int line,
	const struct kimage *image)
{
	unsigned long i;

#if !defined(DEBUG)
	return;
#endif
	pr_devel("%s:%d:\n", func, line);
	pr_devel("  kexec image info:\n");
	pr_devel("    type:        %d\n", image->type);
	pr_devel("    start:       %lx\n", image->start);
	pr_devel("    head:        %lx\n", image->head);
	pr_devel("    nr_segments: %lu\n", image->nr_segments);

	for (i = 0; i < image->nr_segments; i++) {
		pr_devel("      segment[%lu]: %016lx - %016lx, %lx bytes, %lu pages%s\n",
			i,
			image->segment[i].mem,
			image->segment[i].mem + image->segment[i].memsz,
			image->segment[i].memsz,
			image->segment[i].memsz /  PAGE_SIZE,
			(kexec_is_dtb(image->segment[i].buf) ?
				", dtb segment" : ""));
	}
}

/**
 * kexec_find_dtb_seg - Helper routine to find the dtb segment.
 */
static const struct kexec_segment *kexec_find_dtb_seg(
	const struct kimage *image)
{
	int i;

	for (i = 0; i < image->nr_segments; i++) {
		if (kexec_is_dtb(image->segment[i].buf))
			return &image->segment[i];
	}

	return NULL;
}

void machine_kexec_cleanup(struct kimage *image)
{
	/* Empty routine needed to avoid build errors. */
}

/**
 * machine_kexec_prepare - Prepare for a kexec reboot.
 *
 * Called from the core kexec code when a kernel image is loaded.
 */
int machine_kexec_prepare(struct kimage *image)
{
	const struct kexec_segment *dtb_seg = kexec_find_dtb_seg(image);

	arm64_kexec_dtb_addr = dtb_seg ? dtb_seg->mem : 0;
	arm64_kexec_kimage_start = image->start;

	kexec_image_info(image);

	/* TODO: Remove this message when KVM can support a kexec reboot. */
	if (IS_ENABLED(CONFIG_KVM) && is_hyp_mode_available()) {
		pr_err("%s: Your kernel is configured with KVM support (CONFIG_KVM=y) which currently does not allow for kexec re-boot.\n",
			__func__);
		return -ENOSYS;
	}

	return 0;
}

/**
 * kexec_list_flush - Helper to flush the kimage list to PoC.
 */
static void kexec_list_flush(unsigned long kimage_head)
{
	void *dest;
	unsigned long *entry;

	for (entry = &kimage_head, dest = NULL; ; entry++) {
		unsigned int flag = *entry &
			(IND_DESTINATION | IND_INDIRECTION | IND_DONE |
			IND_SOURCE);
		void *addr = phys_to_virt(*entry & PAGE_MASK);

		switch (flag) {
		case IND_INDIRECTION:
			entry = (unsigned long *)addr - 1;
			__flush_dcache_area(addr, PAGE_SIZE);
			break;
		case IND_DESTINATION:
			dest = addr;
			break;
		case IND_SOURCE:
			__flush_dcache_area(addr, PAGE_SIZE);
			dest += PAGE_SIZE;
			break;
		case IND_DONE:
			return;
		default:
			BUG();
		}
	}
}

/**
 * machine_kexec - Do the kexec reboot.
 *
 * Called from the core kexec code for a sys_reboot with LINUX_REBOOT_CMD_KEXEC.
 */
void machine_kexec(struct kimage *image)
{
	phys_addr_t reboot_code_buffer_phys;
	void *reboot_code_buffer;

	BUG_ON(num_online_cpus() > 1);

	arm64_kexec_kimage_head = image->head;

	reboot_code_buffer_phys = page_to_phys(image->control_code_page);
	reboot_code_buffer = phys_to_virt(reboot_code_buffer_phys);

	kexec_image_info(image);

	pr_devel("%s:%d: control_code_page:        %p\n", __func__, __LINE__,
		image->control_code_page);
	pr_devel("%s:%d: reboot_code_buffer_phys:  %pa\n", __func__, __LINE__,
		&reboot_code_buffer_phys);
	pr_devel("%s:%d: reboot_code_buffer:       %p\n", __func__, __LINE__,
		reboot_code_buffer);
	pr_devel("%s:%d: relocate_new_kernel:      %p\n", __func__, __LINE__,
		relocate_new_kernel);
	pr_devel("%s:%d: relocate_new_kernel_size: 0x%lx(%lu) bytes\n",
		__func__, __LINE__, relocate_new_kernel_size,
		relocate_new_kernel_size);

	pr_devel("%s:%d: kexec_dtb_addr:           %lx\n", __func__, __LINE__,
		arm64_kexec_dtb_addr);
	pr_devel("%s:%d: kexec_kimage_head:        %lx\n", __func__, __LINE__,
		arm64_kexec_kimage_head);
	pr_devel("%s:%d: kexec_kimage_start:       %lx\n", __func__, __LINE__,
		arm64_kexec_kimage_start);

	/*
	 * Copy relocate_new_kernel to the reboot_code_buffer for use
	 * after the kernel is shut down.
	 */
	memcpy(reboot_code_buffer, relocate_new_kernel,
		relocate_new_kernel_size);

	/* Flush the reboot_code_buffer in preparation for its execution. */
	__flush_dcache_area(reboot_code_buffer, relocate_new_kernel_size);

	/* Flush the kimage list. */
	kexec_list_flush(image->head);

	pr_info("Bye!\n");

	/* Disable all DAIF exceptions. */
	asm volatile ("msr daifset, #0xf" : : : "memory");

	/*
	 * soft_restart() will shutdown the MMU, disable data caches, then
	 * transfer control to the reboot_code_buffer which contains a copy of
	 * the relocate_new_kernel routine.  relocate_new_kernel will use
	 * physical addressing to relocate the new kernel to its final position
	 * and then will transfer control to the entry point of the new kernel.
	 */
	soft_restart(reboot_code_buffer_phys);
}

static void machine_kexec_mask_interrupts(void)
{
	unsigned int i;
	struct irq_desc *desc;

	for_each_irq_desc(i, desc) {
		struct irq_chip *chip;

		chip = irq_desc_get_chip(desc);
		if (!chip)
			continue;

		if (chip->irq_eoi && irqd_irq_inprogress(&desc->irq_data))
			chip->irq_eoi(&desc->irq_data);

		if (chip->irq_mask)
			chip->irq_mask(&desc->irq_data);

		if (chip->irq_disable && !irqd_irq_disabled(&desc->irq_data))
			chip->irq_disable(&desc->irq_data);
	}
}

/**
 * machine_crash_shutdown - shutdown non-boot cpus and save registers
 */
void machine_crash_shutdown(struct pt_regs *regs)
{
	struct pt_regs dummy_regs;
	int cpu;

	local_irq_disable();

	in_crash_kexec = true;

	/*
	 * clear and initialize the per-cpu info. This is necessary
	 * because, otherwise, slots for offline cpus would not be
	 * filled up. See smp_send_stop().
	 */
	memset(&dummy_regs, 0, sizeof(dummy_regs));
	for_each_possible_cpu(cpu)
		crash_save_cpu(&dummy_regs, cpu);

	/* shutdown non-boot cpus */
	smp_send_stop();

	/* for this cpu */
	crash_save_cpu(regs, smp_processor_id());
	machine_kexec_mask_interrupts();

	pr_info("Loading crashdump kernel...\n");
}
