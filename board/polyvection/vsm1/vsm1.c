/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <i2c.h>
#include <miiphy.h>
#include <fsl_esdhc.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <mxsfb.h>
#include <asm/imx-common/video.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include "../common/pfuze.h"
#include <fuse.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |        \
PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |        \
PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
PAD_CTL_DSE_40ohm | PAD_CTL_HYS |            \
PAD_CTL_ODE)

#define SPI_PAD_CTRL (PAD_CTL_HYS |                \
PAD_CTL_SPEED_MED |        \
PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#ifdef CONFIG_SYS_I2C
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC and EEPROM */
static struct i2c_pads_info i2c_pad_info1 = {
    .scl = {
        .i2c_mode =  MX6_PAD_UART4_TX_DATA__I2C1_SCL | PC,
        .gpio_mode = MX6_PAD_UART4_TX_DATA__GPIO1_IO28 | PC,
        .gp = IMX_GPIO_NR(1, 28),
    },
    .sda = {
        .i2c_mode = MX6_PAD_UART4_RX_DATA__I2C1_SDA | PC,
        .gpio_mode = MX6_PAD_UART4_RX_DATA__GPIO1_IO29 | PC,
        .gp = IMX_GPIO_NR(1, 29),
    },
};

#ifdef CONFIG_POWER
#define I2C_PMIC       0
int power_init_board(void)
{
    if (is_mx6ull_9x9_evk()) {
        struct pmic *pfuze;
        int ret;
        unsigned int reg, rev_id;
        
        ret = power_pfuze3000_init(I2C_PMIC);
        if (ret)
        return ret;
        
        pfuze = pmic_get("PFUZE3000");
        ret = pmic_probe(pfuze);
        if (ret)
        return ret;
        
        pmic_reg_read(pfuze, PFUZE3000_DEVICEID, &reg);
        pmic_reg_read(pfuze, PFUZE3000_REVID, &rev_id);
        printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n",
               reg, rev_id);
        
        /* disable Low Power Mode during standby mode */
        pmic_reg_read(pfuze, PFUZE3000_LDOGCTL, &reg);
        reg |= 0x1;
        pmic_reg_write(pfuze, PFUZE3000_LDOGCTL, reg);
        
        /* SW1B step ramp up time from 2us to 4us/25mV */
        reg = 0x40;
        pmic_reg_write(pfuze, PFUZE3000_SW1BCONF, reg);
        
        /* SW1B mode to APS/PFM */
        reg = 0xc;
        pmic_reg_write(pfuze, PFUZE3000_SW1BMODE, reg);
        
        /* SW1B standby voltage set to 0.975V */
        reg = 0xb;
        pmic_reg_write(pfuze, PFUZE3000_SW1BSTBY, reg);
    }
    
    return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
    unsigned int value;
    u32 vddarm;
    
    struct pmic *p = pmic_get("PFUZE3000");
    
    if (!p) {
        printf("No PMIC found!\n");
        return;
    }
    
    /* switch to ldo_bypass mode */
    if (ldo_bypass) {
        prep_anatop_bypass();
        /* decrease VDDARM to 1.275V */
        pmic_reg_read(p, PFUZE3000_SW1BVOLT, &value);
        value &= ~0x1f;
        value |= PFUZE3000_SW1AB_SETP(12750);
        pmic_reg_write(p, PFUZE3000_SW1BVOLT, value);
        
        set_anatop_bypass(1);
        vddarm = PFUZE3000_SW1AB_SETP(11750);
        
        pmic_reg_read(p, PFUZE3000_SW1BVOLT, &value);
        value &= ~0x1f;
        value |= vddarm;
        pmic_reg_write(p, PFUZE3000_SW1BVOLT, value);
        
        finish_anatop_bypass();
        
        printf("switch to ldo_bypass mode!\n");
    }
}
#endif
#endif
#endif

#ifdef CONFIG_DM_PMIC
int power_init_board(void)
{
    struct udevice *dev;
    int ret, dev_id, rev_id;
    unsigned int reg;
    
    ret = pmic_get("pfuze3000", &dev);
    if (ret == -ENODEV)
    return 0;
    if (ret != 0)
    return ret;
    
    dev_id = pmic_reg_read(dev, PFUZE3000_DEVICEID);
    rev_id = pmic_reg_read(dev, PFUZE3000_REVID);
    printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", dev_id, rev_id);
    
    /* disable Low Power Mode during standby mode */
    reg = pmic_reg_read(dev, PFUZE3000_LDOGCTL);
    reg |= 0x1;
    pmic_reg_write(dev, PFUZE3000_LDOGCTL, reg);
    
    /* SW1B step ramp up time from 2us to 4us/25mV */
    reg = 0x40;
    pmic_reg_write(dev, PFUZE3000_SW1BCONF, reg);
    
    /* SW1B mode to APS/PFM */
    reg = 0xc;
    pmic_reg_write(dev, PFUZE3000_SW1BMODE, reg);
    
    /* SW1B standby voltage set to 0.975V */
    reg = 0xb;
    pmic_reg_write(dev, PFUZE3000_SW1BSTBY, reg);
    
    return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
    unsigned int value;
    u32 vddarm;
    struct udevice *dev;
    int ret;
    
    ret = pmic_get("pfuze3000", &dev);
    if (ret == -ENODEV) {
        printf("No PMIC found!\n");
        return;
    }
    
    /* switch to ldo_bypass mode */
    if (ldo_bypass) {
        prep_anatop_bypass();
        /* decrease VDDARM to 1.275V */
        value = pmic_reg_read(dev, PFUZE3000_SW1BVOLT);
        value &= ~0x1f;
        value |= PFUZE3000_SW1AB_SETP(12750);
        pmic_reg_write(dev, PFUZE3000_SW1BVOLT, value);
        
        set_anatop_bypass(1);
        vddarm = PFUZE3000_SW1AB_SETP(11750);
        
        value = pmic_reg_read(dev, PFUZE3000_SW1BVOLT);
        value &= ~0x1f;
        value |= vddarm;
        pmic_reg_write(dev, PFUZE3000_SW1BVOLT, value);
        
        finish_anatop_bypass();
        
        printf("switch to ldo_bypass mode!\n");
    }
}
#endif
#endif

int dram_init(void)
{
    gd->ram_size = imx_ddr_size();
    
    return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
    MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
    imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FEC_MXC
static int setup_fec(int fec_id)
{
    struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
    int ret;
    
    if (fec_id == 0) {
        if (check_module_fused(MX6_MODULE_ENET1))
        return -1;
        
        /*
         * Use 50M anatop loopback REF_CLK1 for ENET1,
         * clear gpr1[13], set gpr1[17].
         */
        clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
                        IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
    } else {
        if (check_module_fused(MX6_MODULE_ENET2))
        return -1;
        
        /*
         * Use 50M anatop loopback REF_CLK2 for ENET2,
         * clear gpr1[14], set gpr1[18].
         */
        clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
                        IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);
    }
    
    ret = enable_fec_anatop_clock(fec_id, ENET_50MHZ);
    if (ret)
    return ret;
    
    enable_enet_clk(1);
    
    return 0;
}

int board_phy_config(struct phy_device *phydev)
{
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8190);
    
    if (phydev->drv->config)
    phydev->drv->config(phydev);
    
    return 0;
}
#endif

int board_mmc_get_env_dev(int devno)
{
    return devno;
}

int mmc_map_to_kernel_blk(int devno)
{
    return devno;
}

iomux_v3_cfg_t const ecspi4_pads[] = {
    MX6_PAD_UART2_RX_DATA__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_UART2_RTS_B__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_UART2_CTS_B__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_UART2_TX_DATA__GPIO1_IO20  | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void setup_spinor(void)
{
    imx_iomux_v3_setup_multiple_pads(ecspi4_pads,
                                     ARRAY_SIZE(ecspi4_pads));
    gpio_request(IMX_GPIO_NR(1, 20), "ecspi cs");
    gpio_direction_output(IMX_GPIO_NR(1, 20), 0);
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
    return (bus == 2 && cs == 0) ? (IMX_GPIO_NR(1, 20)) : -1;
}

int board_early_init_f(void)
{
    setup_iomux_uart();
    
    return 0;
}

int board_init(void)
{
    /* Address of boot parameters */
    gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
    
#ifdef CONFIG_SYS_I2C
    setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
#endif
    
    
    setup_spinor();


    
    return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
    /* 4 bit bus width */
    {"sd1", MAKE_CFGVAL(0x42, 0x20, 0x00, 0x00)},
    {"sd2", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
    {"qspi1", MAKE_CFGVAL(0x10, 0x00, 0x00, 0x00)},
    {NULL,     0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
    add_board_boot_modes(board_boot_modes);
#endif
    
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
    setenv("board_name", "VS");
    setenv("board_rev", "A1");
    setenv("uboot_ver", "3");
    
    u32 bank, word, val;
    int ret;
    bank=4;
    word=7;
    ret = fuse_read(bank, word, &val);
    if (ret) goto err;
    
    if (val == 00000005){
        setenv("fdt_file", "/boot/imx6ull-voltastream-amp1.dtb");}
    else if (val == 00000004){
        setenv("fdt_file", "/boot/imx6ull-voltastream0.dtb");}
    else if (val == 00000003){
        setenv("fdt_file", "/boot/imx6ull-voltastream0.dtb");}
    else if (val == 00000002){
        setenv("fdt_file", "/boot/imx6ull-voltastream0.dtb");}
    else if (val == 00000001){
        setenv("fdt_file", "/boot/imx6ull-voltastream0.dtb");}
    else if (val == 00000006){
        setenv("fdt_file", "/boot/imx6ull-vsm1-0006.dtb");}
    else if (val == 00000007){
        setenv("fdt_file", "/boot/imx6ull-vsm1-0007.dtb");}
    else {
        setenv("fdt_file", "/boot/imx6ull-voltastream0.dtb");}
    
    return 0;
    
err:
    puts("ERROR while reading fuses!\n");
    return CMD_RET_FAILURE;
    
#endif
    
#ifdef CONFIG_ENV_IS_IN_MMC
    board_late_mmc_env_init();
#endif
    
    set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);
    
    return 0;
}

int checkboard(void)
{
    u32 bank, word, val;
    int ret;
    bank=4;
    word=7;
    ret = fuse_read(bank, word, &val);
    if (ret) goto err;
    
    if (val == 00000005)
    puts("Board: VS AMP1\n");
    else if (val == 00000004)
    puts("Board: VS ZERO - RAM:8G DAC:42\n");
    else if (val == 00000003)
    puts("Board: VS ZERO - RAM:8G DAC:21\n");
    else if (val == 00000002)
    puts("Board: VS ZERO - RAM:4G DAC:42\n");
    else if (val == 00000001)
    puts("Board: VS ZERO - RAM:4G DAC:21\n");
    else if (val == 00000006)
    puts("Board: VS M1 - ID: 0006\n");
    else if (val == 00000007)
    puts("Board: VS M1 - ID: 0007\n");
    else
    puts("Board: Unknown\n");
    
    return 0;
    
err:
    puts("ERROR while reading fuses!\n");
    return CMD_RET_FAILURE;
}
