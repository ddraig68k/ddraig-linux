/*
 * MTD map for the Y Ddraig ROM region (0xF80000–0xFFFFFF, 512 KB).
 *
 * The full ROM window is exposed as a single read-only MTD device.
 * Place a ROMfs image at ROM_START and pass root=mtd0 rootfstype=romfs,
 * or adjust ROM_ROMFS_PHYS/SIZE below to match your ROM image layout.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mtd/map.h>
#include <linux/mtd/mtd.h>

#define DDRAIG_ROM_PHYS  0xF80000
#define DDRAIG_ROM_SIZE  0x080000	/* 512 KB */

static struct map_info ddraig_map = {
	.name      = "ddraig-rom",
	.phys      = DDRAIG_ROM_PHYS,
	.size      = DDRAIG_ROM_SIZE,
	.bankwidth = 1,
};

static struct mtd_info *ddraig_mtd;

static int __init ddraig_map_init(void)
{
	ddraig_map.virt = ioremap(ddraig_map.phys, ddraig_map.size);
	if (!ddraig_map.virt)
		return -EIO;

	simple_map_init(&ddraig_map);

	ddraig_mtd = do_map_probe("map_rom", &ddraig_map);
	if (!ddraig_mtd) {
		iounmap(ddraig_map.virt);
		return -ENXIO;
	}

	ddraig_mtd->owner = THIS_MODULE;
	mtd_device_register(ddraig_mtd, NULL, 0);

	pr_info("ddraig: ROM MTD at 0x%06x, %u KB\n",
		DDRAIG_ROM_PHYS, DDRAIG_ROM_SIZE >> 10);
	return 0;
}
device_initcall(ddraig_map_init);
