/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the novasomp boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <linux/sizes.h>

#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#define CONFIG_SPL_FAT_SUPPORT
#include "imx6_spl.h"

#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define MACH_TYPE_NOVASOMP		5095
#define CONFIG_MACH_TYPE		MACH_TYPE_NOVASOMP

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_SYS_GENERIC_BOARD
#undef CONFIG_LDO_BYPASS_CHECK

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART3_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_SETEXPR

#define CONFIG_BOOTDELAY		2

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)
#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

/* MMC Configuration */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

/* SATA Configs */
#define CONFIG_CMD_SATA
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE      1
#define CONFIG_DWC_AHSATA_PORT_ID       0
#define CONFIG_DWC_AHSATA_BASE_ADDR     SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0

/* Ethernet Configuration */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR          0
#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC

/* Framebuffer */
#define CONFIG_CMD_BMP
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE (1 << 20)
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

/* PWMs */
#define CONFIG_PWM_IMX
#define CONFIG_IMX6_PWM_PER_CLK 66000000

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_DEFAULT_FDT_FILE         "novasomp.dtb"

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
        "setenv splashimage_mmc_dev 0\0"                              \
        "setenv splashimage_mmc_part 1\0"                              \
        "bootenv=NOVAsomParams\0" \
        "splashimage=0x10800000\0"                              \
        "setenv splashpos m,m\0"                                        \
        "splashimage_file_name=splash.bmp.gz\0"   \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"initrd=uInitrd\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"console=ttymxc2\0" \
	"splashpos=m,m\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_addr=0x18000000\0" \
        "fsaddr=0x12800000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"ethaddr=5c:b8:b2:91:9f:29\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"usbdev=0\0" \
	"usbpart=1\0" \
	"mmcroot=/dev/mmcblk2p2 rootwait rw\0" \
	"ramroot=/dev/ram rootwait rw\0" \
	"bootscript=echo Running bootscript from mmc ...; source\0" \
	"mmcloadbootscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"mmcloadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"mmcloadinitrd=fatload mmc ${mmcdev}:${mmcpart} ${fsaddr} ${initrd}\0" \
	"mmcloadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
        "mmcloadbootenv=fatload mmc ${mmcdev}:${mmcpart}  ${loadaddr} ${bootenv}\0" \
        "importbootenv=echo Importing environment ...; env import -t ${loadaddr} ${filesize}\0" \
	"usbloadbootscript=fatload usb ${usbdev}:${usbpart} ${loadaddr} ${script};\0" \
	"usbloadimage=fatload usb ${usbdev}:${usbpart} ${loadaddr} ${image}\0" \
	"usbloadinitrd=fatload usb ${usbdev}:${usbpart} ${fsaddr} ${initrd}\0" \
	"usbloadfdt=fatload usb ${usbdev}:${usbpart} ${fdt_addr} ${fdtfile}\0" \
	"videomode=video=mxcfb0:dev=hdmi,1280x720M@60,if=RGB24 fbmem=28M\0" \
	"ramdisk_size=96000\0" \
	"boardargs=setenv bootargs console=${console},${baudrate} root=${ramroot} ${videomode} ramdisk_size=${ramdisk_size};\0" \
	"board_boot=echo Booting ...; " \
		"run boardargs; " \
		"bootz ${loadaddr} ${fsaddr} ${fdt_addr};\0" \

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev}; "\
		"if mmc rescan; then " \
			"if run mmcloadbootenv; then " \
				"echo Loaded environment ${bootenv};" \
				"run importbootenv;" \
				"run uenvcmd;" \
			"fi;" \
			"if run mmcloadimage; then " \
				"if run mmcloadfdt; then " \
					"if run mmcloadinitrd; then " \
						"run board_boot; " \
					"fi; " \
				"fi; " \
			"fi; " \
		"fi; " \
	   "usb start;" \
		   "if run usbloadbootscript; then " \
			   "run bootscript; " \
		    "else " \
			   "if run usbloadimage; then " \
			   	"if run usbloadfdt; then " \
			   		"if run usbloadinitrd; then " \
					   "run board_boot; " \
			   		"fi; " \
				 "fi; " \
			   "fi; " \
		   "fi; " \
		"fi; " \
	   "run board_boot;"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256
#define CONFIG_SYS_MAXARGS	       16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

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

#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#endif			       /* __CONFIG_H * */
