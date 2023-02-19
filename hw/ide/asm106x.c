/*
 * QEMU ASM106x somewhat-emulation
 *
 * Copyright (c) 2023 Alex Badea <vamposdecampos@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/pci/msi.h"
#include "hw/pci/pci.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/isa/isa.h"
#include "hw/ssi/ssi.h"
#include "hw/block/flash.h"
#include "sysemu/dma.h"
#include "sysemu/blockdev.h"
#include "hw/ide/pci.h"
#include "ahci_internal.h"
#include "qapi/error.h"
#include "sysemu/block-backend-global-state.h"

#define TYPE_ASM106X "asm106x"
OBJECT_DECLARE_SIMPLE_TYPE(ASM106xState, ASM106X)

#define ASM106X_MSI_CAP_OFFSET	 0x80
#define ASM106X_SATA_CAP_OFFSET	0xA8

#define ASM106X_IDP_BAR			4
#define ASM106X_MEM_BAR			5

#define ASM106X_IDP_INDEX		  0x10
#define ASM106X_IDP_INDEX_LOG2	 0x04

struct ASM106xState {
	PCIDevice parent_obj;
	SSIBus *spi;
	qemu_irq irq;
	char *flash_chip;
	BlockBackend *blk;
	MemoryRegion rom_bar;

	uint32_t vendor_id;
	uint32_t device_id;
};

static const VMStateDescription vmstate_asm106x = {
	.name = "asm106x",
	.version_id = 1,
	.fields = (VMStateField[]) {
		VMSTATE_PCI_DEVICE(parent_obj, ASM106xState),
		VMSTATE_END_OF_LIST()
	},
};

static void asm106x_reset(DeviceState *dev)
{
}

static void asm106x_init(Object *obj)
{
}

static uint32_t asm106x_pci_config_read(PCIDevice *d, uint32_t address, int len)
{
	uint32_t res = pci_default_read_config(d, address, len);
//	if (address == 0xf4)
//		d->config[0xf4] &= ~0x20;
	return res;
}

static void asm106x_pci_config_write(PCIDevice *d, uint32_t addr, uint32_t val, int l)
{
	struct ASM106xState *ad = ASM106X(d);

	if (addr == 0xf4) {
		int cs = !!(val & 0x10);
//		printf("ZZZ cs=%d\n", cs);
		qemu_set_irq(ad->irq, cs);
		if (val & 0x20) {
			uint32_t nbytes = val & 0x07;
			bool do_read = !(val & 0x08);
			for (int k = 0; k < nbytes; k++) {
				uint32_t tx = d->config[0xf0 + k];
				uint32_t rx = ssi_transfer(ad->spi, tx);
//				printf("Z ssi %s %d/%d tx 0x%x rx 0x%x\n",
//					do_read ? "rd" : "wr",
//					k, nbytes,
//					tx, rx);
				if (do_read)
					d->config[0xf0 + k] = rx & 0xff;
			}
			val &= ~0x20;
		}
	}
//	printf("def write %02x %02x %02x %02x / %02x\n",
//		d->config[0xf0],
//		d->config[0xf1],
//		d->config[0xf2],
//		d->config[0xf3],
//		d->config[0xf4]);
	pci_default_write_config(d, addr, val, l);

}

static void asm106x_realize(PCIDevice *dev, Error **errp)
{
//	SysBusDevice *sbd = SYS_BUS_DEVICE(&dev->qdev);
	struct ASM106xState *d = ASM106X(dev);

//	sysbus_init_irq(sbd, &d->irq);

	d->spi = ssi_create_bus(&dev->qdev, "ssi");
	DeviceState *flash_dev = qdev_new(d->flash_chip ?: "en25f05");


	if (d->blk) {
		/* the drive is attached to _our_ device, so detach it first */
		BlockBackend *blk = d->blk;
		d->blk = NULL;
		blk_ref(blk);
		blk_detach_dev(blk, blk_get_attached_dev(blk));
		qdev_prop_set_drive_err(flash_dev, "drive", blk, &error_fatal);
		blk_unref(blk);
	}
	qdev_prop_set_bit(flash_dev, "write-enable", true);
	qdev_realize_and_unref(flash_dev, BUS(d->spi), &error_fatal);

	d->irq = qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0);
//	cs_line = qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0);
//	sysbus_connect_irq(sbd, 0, cs_line);

	unsigned rom_size = 65536;
	memory_region_init_rom(&d->rom_bar, OBJECT(d), "asm106x.rom", rom_size, &error_fatal);
	pci_register_bar(dev, PCI_ROM_SLOT, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->rom_bar);


	pci_config_set_prog_interface(dev->config, AHCI_PROGMODE_MAJOR_REV_1);
	pci_set_word(dev->config + PCI_VENDOR_ID, d->vendor_id);
	pci_set_word(dev->config + PCI_DEVICE_ID, d->device_id);
//	pci_set_word(dev->config + PCI_SUBSYSTEM_VENDOR_ID, 0x1b21);
//	pci_set_word(dev->config + PCI_SUBSYSTEM_ID, 0x1060);

	dev->config[PCI_CACHE_LINE_SIZE] = 0x08;  /* Cache line size */
	dev->config[PCI_LATENCY_TIMER]   = 0x00;  /* Latency timer */
	pci_config_set_interrupt_pin(dev->config, 1);

//	pci_set_byte(dev->config + 0xf4, 0x10);
	dev->config[0xf0] = 0x40;
	dev->config[0xf1] = 0x00;
	dev->config[0xf2] = 0x01;
	dev->config[0xf3] = 0x02;
	dev->config[0xf4] = 0x10;

	/* XXX Software should program this register */
	dev->config[0x90]   = 1 << 6; /* Address Map Register - AHCI mode */

#if 0
	d->ahci.irq = pci_allocate_irq(dev);

	pci_register_bar(dev, ASM106X_IDP_BAR, PCI_BASE_ADDRESS_SPACE_IO, &d->ahci.idp);
	pci_register_bar(dev, ASM106X_MEM_BAR, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->ahci.mem);

	sata_cap_offset = pci_add_capability(dev, PCI_CAP_ID_SATA, ASM106X_SATA_CAP_OFFSET, SATA_CAP_SIZE, errp);
	if (sata_cap_offset < 0) {
		return;
	}

	sata_cap = dev->config + sata_cap_offset;
	pci_set_word(sata_cap + SATA_CAP_REV, 0x10);
	pci_set_long(sata_cap + SATA_CAP_BAR,
				 (ASM106X_IDP_BAR + 0x4) | (ASM106X_IDP_INDEX_LOG2 << 4));
	d->ahci.idp_offset = ASM106X_IDP_INDEX;

	/* Although the AHCI 1.3 specification states that the first capability
	 * should be PMCAP, the Intel ASM106X data sheet specifies that the ASM106X
	 * AHCI device puts the MSI capability first, pointing to 0x80. */
	ret = msi_init(dev, ASM106X_MSI_CAP_OFFSET, 1, true, false, NULL);
	/* Any error other than -ENOTSUP(board's MSI support is broken)
	 * is a programming error.  Fall back to INTx silently on -ENOTSUP */
	assert(!ret || ret == -ENOTSUP);
#endif
}

static void asm106x_uninit(PCIDevice *dev)
{
}


static Property asm106x_properties[] = {
	DEFINE_PROP_UINT32("vendor-id", ASM106xState, vendor_id, 0x1b21),
	DEFINE_PROP_UINT32("device-id", ASM106xState, device_id, 0x0612),
	DEFINE_PROP_STRING("flash-chip", ASM106xState, flash_chip),
	DEFINE_PROP_DRIVE("drive", ASM106xState, blk),
	DEFINE_PROP_END_OF_LIST(),
};

static void asm106x_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

	k->realize = asm106x_realize;
	k->exit = asm106x_uninit;
	k->vendor_id = 0x1b21;
	k->device_id = 0x0612;
	k->revision = 0x02;
	k->class_id = PCI_CLASS_STORAGE_SATA;
	k->config_read = asm106x_pci_config_read;
	k->config_write = asm106x_pci_config_write;
	dc->vmsd = &vmstate_asm106x;
	dc->reset = asm106x_reset;
	device_class_set_props(dc, asm106x_properties);
	set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static const TypeInfo asm106x_info = {
	.name		= TYPE_ASM106X,
	.parent		= TYPE_PCI_DEVICE,
	.instance_size	= sizeof(ASM106xState),
	.instance_init	= asm106x_init,
	.class_init	= asm106x_class_init,
	.interfaces	= (InterfaceInfo[]) {
		{ INTERFACE_CONVENTIONAL_PCI_DEVICE },
		{ },
	},
};

static void asm106x_register_types(void)
{
	type_register_static(&asm106x_info);
}

type_init(asm106x_register_types)
