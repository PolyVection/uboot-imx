/*
 * Copyright (C) 2014-2016 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6SX ARM2 board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __VSM1_CONFIG_H
#define __VSM1_CONFIG_H


#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/imx-common/gpio.h>


#define is_mx6ull_9x9_evk()    CONFIG_IS_ENABLED(TARGET_MX6ULL_9X9_EVK)

#ifdef CONFIG_TARGET_MX6ULL_9X9_EVK
#define PHYS_SDRAM_SIZE        SZ_256M
#define BOOTARGS_CMA_SIZE   "cma=96M "
#else
#define PHYS_SDRAM_SIZE        SZ_512M
#define BOOTARGS_CMA_SIZE   ""
/* DCDC used on 14x14 EVK, no PMIC */
#undef CONFIG_LDO_BYPASS_CHECK
#endif

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN        (16 * SZ_1M)

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE        UART1_BASE

/* MMC Configs */
#ifdef CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR    USDHC2_BASE_ADDR


#define CONFIG_SYS_FSL_USDHC_NUM    1
#endif
/* I2C configs */
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1        /* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2        /* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED        100000
#endif

/* Only use DM I2C driver for 14x14 EVK. Because the PFUZE3000 driver does not support DM */
#ifndef CONFIG_DM_I2C
#define CONFIG_SYS_I2C

/* PMIC only for 9X9 EVK */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE3000
#define CONFIG_POWER_PFUZE3000_I2C_ADDR  0x08
#endif

#define CONFIG_SYS_MMC_IMG_LOAD_PART    1

#define CONFIG_EXTRA_ENV_SETTINGS \
"fdt_high=0xffffffff\0" \
"initrd_high=0xffffffff\0" \
"mmcautodetect=yes\0" \
"swbank=1\0" \
"switch= if test ${swbank} -eq 1; then " \
"setenv swbank 2; saveenv; " \
"else " \
"setenv swbank 1; saveenv; " \
"fi; " \
"echo swbank=${swbank};\0" \
"update_required=0;\0" \
"update_loader=sf probe; sf erase 0x0 0xF0000; mw.b 0x90800000 0xFF 0xF0000; " \
"ext4load mmc 0:3 0x90800000 /system/bootloader/u-boot-dtb.imx; sf write 0x90800000 0x400 0xC0000\0" \
"mmcargs=setenv bootargs console=ttymxc0,115200 " \
"root=/dev/mmcblk1p${swbank} coherent_pool=4M net.ifnames=0 rootwait rw\0" \
"loadimage=ext4load mmc 0:${swbank} 0x80800000 /boot/zImage\0" \
"loadfdt=ext4load mmc 0:${swbank} 0x83000000 ${fdt_file}\0" \
"mmcboot=echo Booting from swbank=${swbank} ...; " \
"run mmcargs; " \
"if run loadfdt; then " \
"bootz 0x80800000 - 0x83000000; " \
"else " \
"echo WARN: Cannot load the DT - ABORTING; " \
"fi; " \

#define CONFIG_BOOTCOMMAND \
"mmc dev 0;" \
"mmc dev 0;" \
"if mmc rescan; then " \
"ext4load mmc 0:3 0x83080000 /system/bootloader/uEnv.txt; env import -t 0x83080000 $filesize; " \
"ext4load mmc 0:3 0x84080000 /system/bootloader/uEnv_update.txt; env import -t 0x84080000 $filesize; " \
"if test ${update_required} -eq 1; then " \
"run update_loader; setenv update_required 0; ext4write mmc 0:3 0x40000000 /system/bootloader/uEnv_update.txt 10; reset; " \
"fi; " \
"if run loadimage; then " \
"run mmcboot; " \
"else " \
"echo No kernel found on try 1- switching bank!; " \
"run switch; " \
"fi; " \
"fi;" \
"mmc dev 0;" \
"mmc dev 0;" \
"if mmc rescan; then " \
"if run loadimage; then " \
"run mmcboot; " \
"else " \
"echo No kernel found on try 2 - switching bank!; " \
"run switch; " \
"fi; " \
"fi;" \
"mmc dev 0;" \
"mmc dev 0;" \
"if mmc rescan; then " \
"if run loadimage; then " \
"run mmcboot; " \
"else " \
"echo No kernel found on try 3 - switching bank!; " \
"run switch; " \
"fi; " \
"fi;" \
"mmc dev 0;" \
"mmc dev 0;" \
"if mmc rescan; then " \
"if run loadimage; then " \
"run mmcboot; " \
"else " \
"echo No kernel found on try 4 - ROOTFS CORRUPTED!; " \
"fi; " \
"fi;"

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START    0x80000000
#define CONFIG_SYS_MEMTEST_END        (CONFIG_SYS_MEMTEST_START + 0x8000000)

#define CONFIG_SYS_LOAD_ADDR        CONFIG_LOADADDR
#define CONFIG_SYS_HZ            1000

#define CONFIG_STACKSIZE        SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS        1
#define PHYS_SDRAM            MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE        PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR    IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE    IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#ifdef CONFIG_QSPI_BOOT
#define CONFIG_ENV_IS_IN_SPI_FLASH
#elif defined CONFIG_NAND_BOOT
#define CONFIG_CMD_NAND
#define CONFIG_ENV_IS_IN_NAND
#else
#define CONFIG_ENV_IS_IN_MMC
#endif

/* environment organization */
#define CONFIG_SYS_MMC_ENV_DEV        1    /* USDHC2 */
#define CONFIG_SYS_MMC_ENV_PART        0    /* user area */
#define CONFIG_MMCROOT            "/dev/mmcblk1p2"  /* USDHC2 */

#define CONFIG_CMD_BMODE

#define CONFIG_IMX_THERMAL

#define CONFIG_IOMUX_LPSR
/*
#define CONFIG_SOFT_SPI
*/
#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_FSL_QSPI_AHB
#define CONFIG_SF_DEFAULT_BUS        0
#define CONFIG_SF_DEFAULT_CS        0
#define CONFIG_SF_DEFAULT_SPEED    40000000
#define CONFIG_SF_DEFAULT_MODE        SPI_MODE_0
#define FSL_QSPI_FLASH_NUM        1
#define FSL_QSPI_FLASH_SIZE        SZ_32M
#endif

#ifdef CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS

#define CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE    1
#define CONFIG_SYS_NAND_BASE        0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8
#endif

#define CONFIG_ENV_SIZE            SZ_8K
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET        (12 * SZ_64K)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET        (896 * 1024)
#define CONFIG_ENV_SECT_SIZE        (64 * 1024)
#define CONFIG_ENV_SPI_BUS        CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS        CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE        CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ        CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET        (60 << 20)
#define CONFIG_ENV_SECT_SIZE        (128 << 10)
#define CONFIG_ENV_SIZE            CONFIG_ENV_SECT_SIZE
#endif

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC        (PORT_PTS_UTMI | PORT_PTS_PTW)
#endif

#ifdef CONFIG_CMD_NET
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_ENET_DEV        1

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE            ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x2
#define CONFIG_FEC_XCV_TYPE             RMII
#ifdef CONFIG_DM_ETH
#define CONFIG_ETHPRIME            "eth0"
#else
#define CONFIG_ETHPRIME            "FEC0"
#endif
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE            ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR        0x1
#define CONFIG_FEC_XCV_TYPE        RMII
#ifdef CONFIG_DM_ETH
#define CONFIG_ETHPRIME            "eth1"
#else
#define CONFIG_ETHPRIME            "FEC1"
#endif
#endif

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_FEC_MXC_MDIO_BASE ENET2_BASE_ADDR
#endif

#define CONFIG_MODULE_FUSE
#define CONFIG_OF_SYSTEM_SETUP

#define CONFIG_MXC_SPI

#ifdef CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  2
#define CONFIG_SF_DEFAULT_SPEED 20000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#define CONFIG_SF_DEFAULT_CS   0
#endif

#endif



