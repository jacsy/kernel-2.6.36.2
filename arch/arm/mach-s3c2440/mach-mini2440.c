/* linux/arch/arm/mach-s3c2440/mach-mini2440.c
 *
 * Copyright (c) 2004-2005 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * http://www.fluff.org/ben/mini2440/
 *
 * Thanks to Dimity Andric and TomTom for the loan of an SMDK2440.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dm9000.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>

#include <mach/idle.h>
#include <mach/fb.h>
#include <plat/iic.h>

#include <plat/s3c2410.h>
#include <plat/s3c244x.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <plat/common-smdk.h>
#include <plat/nand.h>

/* DM9000AEP 10/100 ethernet controller */
#define MACH_MINI2440_DM90000_BASE	(S3C2410_CS4+0x300)

static struct resource mini2440_dm9000_resource[] ={
	[0] = {
		.start	= MACH_MINI2440_DM90000_BASE,
		.end	= MACH_MINI2440_DM90000_BASE + 3,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MACH_MINI2440_DM90000_BASE + 4,
		.end	= MACH_MINI2440_DM90000_BASE + 7,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= IRQ_EINT7,
		.end	= IRQ_EINT7,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

/*
* the DM9000 has not eeprom, and it's MAC address have been set by
* the bootloader before starting the kernel
*/
static struct dm9000_plat_data mini2440_dm9000_pdata = {
	.flags	= (DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM),
};

static struct platform_device mini2440_device_ethernet = {
	.name			= "dm9000",
	.id				= -1,
	.num_resources	= ARRAY_SIZE(mini2440_dm9000_resource),
	.resource		= mini2440_dm9000_resource,
	.dev			= {
		.platform_data	= &mini2440_dm9000_pdata,
	},
};

static struct map_desc mini2440_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}
};

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg mini2440_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x43,
		.ufcon	     = 0x51,
	}
};

/* LCD driver info */

static struct s3c2410fb_display mini2440_lcd_cfg __initdata = {

	.lcdcon5	= S3C2410_LCDCON5_FRM565 |
			  S3C2410_LCDCON5_INVVLINE |
			  S3C2410_LCDCON5_INVVFRAME |
			  S3C2410_LCDCON5_PWREN |
			  S3C2410_LCDCON5_HWSWP,

	.type		= S3C2410_LCDCON1_TFT,

	.width		= 240,
	.height		= 320,

	.pixclock	= 166667, /* HCLK 60 MHz, divisor 10 */
	.xres		= 240,
	.yres		= 320,
	.bpp		= 16,
	.left_margin	= 20,
	.right_margin	= 8,
	.hsync_len	= 4,
	.upper_margin	= 8,
	.lower_margin	= 7,
	.vsync_len	= 4,
};

static struct s3c2410fb_mach_info mini2440_fb_info __initdata = {
	.displays	= &mini2440_lcd_cfg,
	.num_displays	= 1,
	.default_display = 0,

#if 0
	/* currently setup by downloader */
	.gpccon		= 0xaa940659,
	.gpccon_mask	= 0xffffffff,
	.gpcup		= 0x0000ffff,
	.gpcup_mask	= 0xffffffff,
	.gpdcon		= 0xaa84aaa0,
	.gpdcon_mask	= 0xffffffff,
	.gpdup		= 0x0000faff,
	.gpdup_mask	= 0xffffffff,
#endif

	.lpcsel		= ((0xCE6) & ~7) | 1<<4,
};

static struct mtd_partition mini2440_default_nand_part[] = {
	[0] = {
		.name	= "bootloader",
		.offset = 0x0,
		.size	= 0x00040000,
	},
	[1] = {
		.name 	= "parameters",
		.offset = 0x00040000,
		.size	= 0x00020000,
	},
	[2] = {
		.name 	= "kernel",
		.offset	= 0x00060000,
		.size	= 0x00500000,
	},
	[3] = {
		.name	= "root",
		.offset	= 0x00560000,
		.size 	= 1024 * 1024 * 1024,
	},
	[4] = {
		.name 	= "nand flash",
		.offset	= 0x0,
		.size	= 1024 * 1024 * 1024,
	},
};

static struct s3c2410_nand_set mini2440_nand_sets[] = {
	/* only one chip */
	[0] = {
		.name 			= "NAND",
		.nr_chips		= 1,
		.nr_partitions	= ARRAY_SIZE(mini2440_default_nand_part),
		.partitions		= mini2440_default_nand_part,
	},
};

static struct s3c2410_platform_nand mini2440_nand_info = {
	.tacls				= 20,
	.twrph0				= 60,
	.twrph1				= 20,
	.nr_sets			= ARRAY_SIZE(mini2440_nand_sets),
	.sets				= mini2440_nand_sets,
	.ignore_unset_ecc	= 1,
};
static struct platform_device *mini2440_devices[] __initdata = {
	&s3c_device_ohci,
	&s3c_device_lcd,
	&s3c_device_wdt,
	&s3c_device_i2c0,
	&s3c_device_iis,
	&mini2440_device_ethernet,
	&s3c_device_nand,
};

static void __init mini2440_map_io(void)
{
	s3c24xx_init_io(mini2440_iodesc, ARRAY_SIZE(mini2440_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(mini2440_uartcfgs, ARRAY_SIZE(mini2440_uartcfgs));
}

static void __init mini2440_machine_init(void)
{
	s3c24xx_fb_set_platdata(&mini2440_fb_info);
	s3c_i2c0_set_platdata(NULL);

	platform_add_devices(mini2440_devices, ARRAY_SIZE(mini2440_devices));
	//smdk_machine_init();
}

MACHINE_START(S3C2440, "MINI2440")
	/* Maintainer: Ben Dooks <ben-linux@fluff.org> */
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= mini2440_map_io,
	.init_machine	= mini2440_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
