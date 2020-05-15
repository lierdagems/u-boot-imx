/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the lierda IMX6ULL EVK board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __IMX6ULLEVK_CONFIG_H
#define __IMX6ULLEVK_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/imx-common/gpio.h>

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

#ifdef CONFIG_SECURE_BOOT
#ifndef CONFIG_CSF_SIZE
#define CONFIG_CSF_SIZE 0x4000
#endif
#endif

#define PHYS_SDRAM_SIZE		SZ_128M
#define CONFIG_BOOTARGS_CMA_SIZE   "cma=96M "

/* DCDC used, no PMIC */
#undef CONFIG_LDO_BYPASS_CHECK

/* SPL options */
/* We default not support SPL
 * #define CONFIG_SPL_LIBCOMMON_SUPPORT
 * #define CONFIG_SPL_MMC_SUPPORT
 * #include "imx6_spl.h"
*/

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* MMC Configs */
#ifdef CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR

/* NAND pin conflicts with usdhc1 */
#ifdef CONFIG_SYS_BOOT_NAND
#define CONFIG_SYS_FSL_USDHC_NUM	1
#else
#define CONFIG_SYS_FSL_USDHC_NUM	2
#endif
#endif

/* I2C configs */
#define CONFIG_CMD_I2C
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED		100000
#endif

#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

#ifdef CONFIG_SYS_BOOT_NAND
#define CONFIG_MFG_NAND_PARTITION "mtdparts=gpmi-nand:4m(uboot),10m(kernel),1m(dtb),-(rootfs) " 
#else
#define CONFIG_MFG_NAND_PARTITION ""
#endif

#ifdef CONFIG_SYS_BOOT_NAND
#define MTDIDS_DEFAULT		"nand0=gpmi-nand"
#define CONFIG_BOOT_MEDIA	"bootmedia=echo Booting from NAND ...;\0"
#define CONFIG_BOOT_ARGS        "bootargs=console=ttymxc0,115200 ubi.mtd=3 "  \
					"root=ubi0:imx6ull128-rootfs rootfstype=ubifs " \
					CONFIG_BOOTARGS_CMA_SIZE \
					"mtdparts=gpmi-nand:4m(uboot),10m(kernel),1m(dtb),-(rootfs)\0"
#else
#define CONFIG_BOOT_MEDIA	"bootmedia=echo Booting from MMC ...;\0"
#define CONFIG_BOOT_ARGS	"bootargs=console=${console},115200 " \
					CONFIG_BOOTARGS_CMA_SIZE \
					"root=${mmcroot}\0"
#endif

#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
	    CONFIG_BOOTARGS_CMA_SIZE \
		"rdinit=/linuxrc " \
		"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
		"g_mass_storage.file=/fat g_mass_storage.ro=1 " \
		"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
		"g_mass_storage.iSerialNumber=\"\" "\
		CONFIG_MFG_NAND_PARTITION \
		"clk_ignore_unused "\
		"\0" \
	"initrd_addr=0x83800000\0" \
	"initrd_high=0xffffffff\0" \
	"bootcmd_mfg=run mfgtool_args; bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0" \

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	"panel=LSD1RM-A7C077LI\0" \
	"loadaddr=0x80800000\0" \
	"fdt_addr=0x83000000\0" \
	"fdt_high=0xffffffff\0" \
	"console=ttymxc0\0" \
	"uboot=u-boot-padding.imx\0" \
	"image=zImage\0" \
	"fdt_file=imx6ull-evk-unvs.dtb\0" \
	"ubifs=rootfs.ubi\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"loadbootscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} boot.scr\0" \
	"loadimagenand=nand read ${loadaddr} 0x400000 0x800000\0" \
	"loadfdtnand=nand read ${fdt_addr} 0xe00000 0x100000\0" \
	"loadubootemmc=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${uboot}\0" \
	"loadimagemmc=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdtmmc=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"loadubimmc=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${ubifs}\0" \
	CONFIG_BOOT_ARGS \
	CONFIG_BOOT_MEDIA \

/**

-------------------------------------------------
|    4M   |   10M    |  1M   |        113M      |
|  uboot  |  kernel  |  dtb  |      fsImage     |
-------------------------------------------------
*/

/**
 * 
 * 0x00000000-0x00400000 : "uboot"                                         
 * 0x00400000-0x00e00000 : "kernel"                                        
 * 0x00e00000-0x00f00000 : "dtb"                                           
 * 0x00f00000-0x08000000 : "rootfs"

 * #: name                size            offset          mask_flags              
 * 0: uboot               0x00400000      0x00000000      0
 * 1: kernel              0x00a00000      0x00400000      0
 * 2: dtb                 0x00100000      0x00e00000      0
 * 3: rootfs              0x07100000      0x00f00000      0
 */

#define CONFIG_BOOTCOMMAND \
	"if mmc dev 0; then " \
		"if test -e mmc ${mmcdev}:${mmcpart} boot.scr; then " \
			"run loadbootscript; " \
			"source ${loadaddr}; " \
		"else " \
			"if run loadubootemmc; then " \
				"nand erase 0x00000000 0x00400000; " \
				"kobsng ${loadaddr} 0x00000000 $filesize; " \
			"fi; " \
			"if run loadimagemmc; then " \
				"nand erase 0x00400000 0x00a00000; " \
				"nand write ${loadaddr} 0x00400000 $filesize; " \
			"fi; " \
			"if run loadfdtmmc; then " \
				"nand erase 0x00e00000 0x00100000; " \
				"nand write ${fdt_addr} 0x00e00000 $filesize; " \
			"fi; " \
			"if run loadubimmc; then " \
				"nand erase 0x00f00000 0x07100000;" \
				"nand write ${fdt_addr} 0x00f00000 $filesize; " \
			"fi; " \
			"led 0 off; " \
		"fi; " \
	"else " \
		"if run loadimagenand; then " \
			"run bootmedia; " \
		"fi; " \
		"if run loadfdtnand; then " \
			"bootz ${loadaddr} - ${fdt_addr}; " \
		"fi; " \
	"fi; " \


/* Miscellaneous configurable options */
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x4000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#ifdef CONFIG_SYS_BOOT_NAND
#define CONFIG_ENV_IS_IN_NAND
#else
#define CONFIG_ENV_IS_IN_MMC
#endif

#ifdef CONFIG_SYS_BOOT_NAND
#define CONFIG_SYS_MMC_ENV_DEV		0   			/* USDHC1 */
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"	/* USDHC1 */
#else
#define CONFIG_SYS_MMC_ENV_DEV		1   			/* USDHC2 */
#define CONFIG_MMCROOT			"/dev/mmcblk1p2"	/* USDHC2 */
#endif

#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */


#define CONFIG_CMD_BMODE

/* NAND stuff */
#ifdef CONFIG_SYS_BOOT_NAND
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS

#define CONFIG_CMD_KOBSNG
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_CMD_MTDPARTS
#define CONFIG_RBTREE
#define CONFIG_LZO
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS

#define CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* LED */
#define CONFIG_CMD_LED
#define CONFIG_GPIO_LED
#define CONFIG_STATUS_LED
#define CONFIG_BOARD_SPECIFIC_LED
#define STATUS_LED_BIT IMX_GPIO_NR(5, 4)
#define STATUS_LED_STATE STATUS_LED_ON
#define STATUS_LED_PERIOD (CONFIG_SYS_HZ / 2)

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(12 * SZ_64K)
#define CONFIG_ENV_SIZE			SZ_8K
#elif defined(CONFIG_ENV_IS_IN_NAND)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(128 << 10)
#define CONFIG_ENV_SIZE			CONFIG_ENV_SECT_SIZE
#endif


/* USB Configs */
#define CONFIG_CMD_USB
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#endif

/** Net Configs */
#ifdef CONFIG_CMD_NET
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII

#define CONFIG_FEC_ENET_DEV		0
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x0
#define CONFIG_FEC_XCV_TYPE             RMII
#define CONFIG_ETHPRIME			"FEC"

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#endif

#define CONFIG_IMX_THERMAL

#define CONFIG_VIDEO
#ifdef CONFIG_VIDEO
#define CONFIG_CFB_CONSOLE
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_SW_CURSOR
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#endif

#define CONFIG_IOMUX_LPSR

#if defined(CONFIG_ANDROID_SUPPORT)
#include "mx6ullevk_android.h"
#endif

#endif
