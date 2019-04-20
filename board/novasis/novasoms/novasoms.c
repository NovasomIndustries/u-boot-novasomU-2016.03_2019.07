/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2014 O.S. Systems Software LTDA.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <phy.h>
#include <input.h>
#include <i2c.h>
#include <splash.h>
#include <pwm.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define USDHC3_CD_GPIO		IMX_GPIO_NR(1, 7)
#define ETH_PHY_RESET		IMX_GPIO_NR(1, 25)
#define USB_HUB_RESET		IMX_GPIO_NR(2, 19)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD    | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	/* SOM MicroSD Card Detect */
	IOMUX_PADS(PAD_EIM_DA9__GPIO3_IO09  | MUX_PAD_CTRL(NO_PAD_CTRL)),
};
static iomux_v3_cfg_t const usdhc4_pads[] = {
        /* eMMC */
        IOMUX_PADS(PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_NANDF_ALE__SD4_RESET   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
        IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};


static iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC    | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* KSZ9031 PHY Reset */
	IOMUX_PADS(PAD_ENET_CRS_DV__GPIO1_IO25    | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const usb_pads[] = {
	IOMUX_PADS(PAD_EIM_A19__GPIO2_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};
static iomux_v3_cfg_t const backlight_pads[] = {
        IOMUX_PADS(PAD_GPIO_9__PWM1_OUT | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}


static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);
	/* Reset KSZ9031 PHY */
	gpio_direction_output(ETH_PHY_RESET, 0);
	udelay(500);
	gpio_set_value(ETH_PHY_RESET, 1);
}

static void setup_iomux_usb(void)
{
	SETUP_IOMUX_PADS(usb_pads);
	/* Reset USB2514B HUB */
	gpio_direction_output(USB_HUB_RESET, 0);
	udelay(2000);
	gpio_set_value(USB_HUB_RESET, 1);
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
int ret = 0;

	switch (cfg->esdhc_base) 
	{
		case USDHC3_BASE_ADDR:
			ret = !gpio_get_value(USDHC3_CD_GPIO);
			break;
		default : ret = 1;
			break;
	}
	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;

	/*
	 * Following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SOM MicroSD
	 * mmc1                    Carrier board MicroSD
	 */
	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			SETUP_IOMUX_PADS(usdhc3_pads);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			usdhc_cfg[0].max_bus_width = 4;
			gpio_direction_input(USDHC3_CD_GPIO);
			break;
		case 1:
			SETUP_IOMUX_PADS(usdhc4_pads);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			usdhc_cfg[1].max_bus_width = 8;
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (ret)
			return ret;
	}

	return 0;
}

static int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
struct i2c_pads_info mx6q_i2c2_pad_info = {
	.scl = {
		.i2c_mode = MX6Q_PAD_KEY_COL3__I2C2_SCL
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO4_IO12
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_KEY_ROW3__I2C2_SDA
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO4_IO13
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

struct i2c_pads_info mx6dl_i2c2_pad_info = {
	.scl = {
		.i2c_mode = MX6DL_PAD_KEY_COL3__I2C2_SCL
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6DL_PAD_KEY_COL3__GPIO4_IO12
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6DL_PAD_KEY_ROW3__I2C2_SDA
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6DL_PAD_KEY_ROW3__GPIO4_IO13
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

static void enable_lvds(struct display_info_t const *dev)
{
        imx_iomux_v3_setup_multiple_pads( backlight_pads, ARRAY_SIZE(backlight_pads));

        /* enable backlight PWM 1 */
        if (pwm_init(0, 0, 0))
                goto error;
        /* duty cycle 500ns, period: 3000ns */
        if (pwm_config(0, 500, 3000))
                goto error;
        if (pwm_enable(0))
                goto error;
        return;

error:
        puts("error init pwm for backlight\n");
        return;
}

static void enable_hdmi(struct display_info_t const *dev)
{
	imx_enable_hdmi_phy();
}

struct display_info_t const displays[] = {
	{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
		} 
	}, 
        {
                .bus    = -1,
                .addr   = 0,
                .pixfmt = IPU_PIX_FMT_RGB24,
                .detect = NULL,
                .enable = enable_lvds,
                .mode   = {
                        .name           = "am800x480",
                        .refresh        = 60,
                        .xres           = 800,
                        .yres           = 480,
                        .pixclock       = 33246,
                        .left_margin    = 88,
                        .right_margin   = 88,
                        .upper_margin   = 10,
                        .lower_margin   = 10,
                        .hsync_len      = 80,
                        .vsync_len      = 25,
                        .sync           = 0,
                        .vmode          = FB_VMODE_NONINTERLACED
                }
        }

};
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
int reg;

        enable_ipu_clock();
	/* FIL uncomment this for 3.0 kernel ?? */
	// imx_setup_hdmi();
	/* FIL END */
        /* Turn on LDB0,IPU,IPU DI0 clocks */
        reg = __raw_readl(&mxc_ccm->CCGR3);
        reg |= MXC_CCM_CCGR3_LDB_DI0_MASK;
        writel(reg, &mxc_ccm->CCGR3);

        /* set LDB0, LDB1 clk select to 011/011 */
        reg = readl(&mxc_ccm->cs2cdr);
        reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
        reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET) |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
        writel(reg, &mxc_ccm->cs2cdr);

        reg = readl(&mxc_ccm->cscmr2);
        reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
        writel(reg, &mxc_ccm->cscmr2);

        reg = readl(&mxc_ccm->chsccdr);
        reg |= (CHSCCDR_CLK_SEL_LDB_DI0 <<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
        writel(reg, &mxc_ccm->chsccdr);

        reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
             |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
             |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
             |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
             |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
             |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
             |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
             |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
             |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
        writel(reg, &iomux->gpr[2]);

        reg = readl(&iomux->gpr[3]);
        reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK) | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
        writel(reg, &iomux->gpr[3]);

        /* Backlight CABEN on LVDS connector */
/*
        SETUP_IOMUX_PAD(PAD_SD2_CLK__GPIO1_IO10 | DIO_PAD_CFG);
        gpio_direction_output(IMX_GPIO_NR(1, 10), 0);
*/
}

#endif /* CONFIG_VIDEO_IPUV3 */

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();

	return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
	setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	  MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	  MAKE_CFGVAL(0x40, 0x20, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D))
	{
		setenv("board_rev", "MX6Q");
		setenv("fdt_file", "imx6q-novasomS.dtb");
	}
	else
	{
		setenv("board_rev", "MX6DL");
		setenv("fdt_file", "imx6sdl-novasomS.dtb");
	}
#endif
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c2_pad_info);
	if (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D))
		setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c2_pad_info);
	else
		setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c2_pad_info);
	setup_iomux_usb();
	return 0;
}

int checkboard(void)
{
	if (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D))
		puts("Board: NOVAsom S, QUAD version\n");
	if (is_cpu_type(MXC_CPU_MX6SOLO))
		puts("Board: NOVAsom S, SOLO version\n");
	if (is_cpu_type(MXC_CPU_MX6DL))
		puts("Board: NOVAsom S, D/L version\n");
	return 0;
}
