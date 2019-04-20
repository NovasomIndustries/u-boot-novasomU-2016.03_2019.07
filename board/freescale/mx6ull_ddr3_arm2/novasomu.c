/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <linux/sizes.h>
#include <linux/fb.h>
#include <miiphy.h>
#include <mmc.h>
#include <mxsfb.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../common/pfuze.h"
#include <usb.h>
#include <usb/ehci-fsl.h>
#if defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#include <asm/imx-common/video.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL_WP (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                   \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_120ohm   | PAD_CTL_SRE_FAST)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |          \
	PAD_CTL_SPEED_HIGH   | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
			PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

#define WEIM_NOR_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | \
		PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
		PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define EPDC_PAD_CTRL	0x010b1

#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC and EEPROM */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		/* conflict with usb_otg2_pwr */
		.i2c_mode = MX6_PAD_GPIO1_IO02__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO02__GPIO1_IO02 | PC,
		.gp = IMX_GPIO_NR(1, 2),
	},
	.sda = {
		/* conflict with usb_otg2_oc */
		.i2c_mode = MX6_PAD_GPIO1_IO03__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO03__GPIO1_IO03 | PC,
		.gp = IMX_GPIO_NR(1, 3),
	},
};
#endif

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_UART1_CTS_B__USDHC1_WP | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

#if !defined(CONFIG_SYS_USE_NAND) && !defined(CONFIG_MX6ULL_DDR3_ARM2_QSPIB_REWORK)
static iomux_v3_cfg_t const usdhc2_pads[] = {
	/* usdhc2_clk, nand_re_b, qspi1b_clk */
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* usdhc2_cmd, nand_we_b, qspi1b_cs0_b */
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* usdhc2_data0, nand_data0, qspi1b_cs1_b */
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* usdhc2_data1, nand_data1 */
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* usdhc2_data2, nand_data2, qspi1b_dat0 */
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* usdhc2_data3, nand_data3, qspi1b_dat1 */
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/*
	 * VSELECT
	 * Conflicts with WDOG1, so default disabled.
	 * MX6_PAD_GPIO1_IO08__USDHC2_VSELECT | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	 */
	/*
	 * CD
	 * Share with sdhc1
	 * MX6_PAD_CSI_MCLK__GPIO4_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL),
	 */
	/*
	 * RST_B
	 * Pin conflicts with NAND ALE, if want to test nand,
	 * Connect R169(B), disconnect R169(A).
	 */
	MX6_PAD_NAND_ALE__GPIO4_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#endif

#ifdef CONFIG_SYS_USE_NAND
#endif

#ifdef CONFIG_SYS_USE_SPINOR
#endif

#ifdef CONFIG_FEC_MXC
/*
 * pin conflicts for fec1 and fec2, GPIO1_IO06 and GPIO1_IO07 can only
 * be used for ENET1 or ENET2, cannot be used for both.
 */
static iomux_v3_cfg_t const fec1_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	/* Pin conflicts with LCD PWM1 */
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/*
	 * ALT5 mode is only valid when TAMPER pin is used for GPIO.
	 * This depends on FUSE settings, TAMPER_PIN_DISABLE[1:0].
	 */
};

static void setup_iomux_fec(int fec_id)
{
	imx_iomux_v3_setup_multiple_pads(fec1_pads, ARRAY_SIZE(fec1_pads));
}
#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_QSPI

#define QSPI_PAD_CTRL1	\
	(PAD_CTL_SRE_FAST | PAD_CTL_SPEED_MED | \
	 PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_47K_UP | PAD_CTL_DSE_120ohm)

static iomux_v3_cfg_t const quadspi_pads[] = {
	MX6_PAD_NAND_WP_B__QSPI_A_SCLK	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_READY_B__QSPI_A_DATA00	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CE0_B__QSPI_A_DATA01	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CE1_B__QSPI_A_DATA02	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CLE__QSPI_A_DATA03	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DQS__QSPI_A_SS0_B	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA07__QSPI_A_SS1_B	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),

#ifdef CONFIG_MX6ULL_DDR3_ARM2_QSPIB_REWORK
	MX6_PAD_NAND_RE_B__QSPI_B_SCLK	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_WE_B__QSPI_B_SS0_B	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA00__QSPI_B_SS1_B	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA02__QSPI_B_DATA00	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA03__QSPI_B_DATA01	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA04__QSPI_B_DATA02	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA05__QSPI_B_DATA03	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
#endif
};

int board_qspi_init(void)
{
	/* Set the iomux */
	imx_iomux_v3_setup_multiple_pads(quadspi_pads, ARRAY_SIZE(quadspi_pads));

	/* Set the clock */
	enable_qspi_clk(0);

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC1_BASE_ADDR, 0, 4},
};

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int mmc_map_to_kernel_blk(int devno)
{
	return devno;
}

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads( usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers (%d) than supported by the board\n", i + 1);
			return 0;
			}

			if (fsl_esdhc_initialize(bis, &usdhc_cfg[i]))
				printf("Warning: failed to initialize mmc dev %d\n", i);
	}
	return 0;
}
#endif

#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_LCD_CLK__LCDIF_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_ENABLE__LCDIF_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_HSYNC__LCDIF_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_VSYNC__LCDIF_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA00__LCDIF_DATA00 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA01__LCDIF_DATA01 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA02__LCDIF_DATA02 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA03__LCDIF_DATA03 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA04__LCDIF_DATA04 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA05__LCDIF_DATA05 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA06__LCDIF_DATA06 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA07__LCDIF_DATA07 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA08__LCDIF_DATA08 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA09__LCDIF_DATA09 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA10__LCDIF_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA11__LCDIF_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA12__LCDIF_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA13__LCDIF_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA14__LCDIF_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA15__LCDIF_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA16__LCDIF_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA17__LCDIF_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA18__LCDIF_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA19__LCDIF_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA20__LCDIF_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA21__LCDIF_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA22__LCDIF_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA23__LCDIF_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_RESET__GPIO3_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/*
	 * PWM1, pin conflicts with ENET1_RX_DATA0
	 * Use GPIO for Brightness adjustment, duty cycle = period.
	 */
	/* MX6_PAD_ENET1_RX_DATA0__GPIO2_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),*/
};

struct lcd_panel_info_t {
	unsigned int lcdif_base_addr;
	int depth;
	void (*enable)(struct lcd_panel_info_t const *dev);
	struct fb_videomode mode;
};

void do_enable_parallel_lcd(struct display_info_t const *dev)
{
	enable_lcdif_clock(dev->bus);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	/* Power up the LCD */
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 1);

	/* Set Brightness to high */
	/* gpio_direction_output(IMX_GPIO_NR(2, 0) , 1); */
}

struct display_info_t const displays[] = {{
	.bus = MX6ULL_LCDIF1_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
	.enable	= do_enable_parallel_lcd,
	.mode	= {
		.name		= "MCIMX28LCD",
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 29850,
		.left_margin    = 89,
		.right_margin   = 164,
		.upper_margin   = 23,
		.lower_margin   = 10,
		.hsync_len      = 10,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);
#endif


#ifdef CONFIG_FEC_MXC
int board_eth_init(bd_t *bis)
{
	int ret;

	setup_iomux_fec(CONFIG_FEC_ENET_DEV);

	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV, CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
	if (ret)
		printf("FEC%d MXC: %s:failed\n", CONFIG_FEC_ENET_DEV, __func__);

	return 0;
}

static int setup_fec(int fec_id)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;
	int ret;

	if (0 == fec_id) {
		if (check_module_fused(MX6_MODULE_ENET1))
			return -1;
		/*
		 * Use 50M anatop loopback REF_CLK1 for ENET1,
		 * clear gpr1[13], set gpr1[17]
		 */
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
				IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
		ret = enable_fec_anatop_clock(fec_id, ENET_50MHZ);
		if (ret)
			return ret;

	} else {
		if (check_module_fused(MX6_MODULE_ENET2))
			return -1;

		/* clk from phy, set gpr1[14], clear gpr1[18]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
				IOMUX_GPR1_FEC2_CLOCK_MUX2_SEL_MASK);
	}

	enable_enet_clk(1);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	if (CONFIG_FEC_ENET_DEV == 0) {
		phy_write(phydev, MDIO_DEVAD_NONE, 0x16, 0x202);
		phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8190);
	} else if (CONFIG_FEC_ENET_DEV == 1) {
		phy_write(phydev, MDIO_DEVAD_NONE, 0x16, 0x201);
		phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8110);
	}

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif

#ifdef CONFIG_POWER
#define I2C_PMIC	0
static struct pmic *pfuze;
int power_init_board(void)
{
#if 0
	int ret;
	u32 rev_id, value;

	ret = power_pfuze100_init(I2C_PMIC);
	if (ret)
		return ret;

	pfuze = pmic_get("PFUZE100");
	if (!pfuze)
		return -ENODEV;

	ret = pmic_probe(pfuze);
	if (ret)
		return ret;

	ret = pfuze_mode_init(pfuze, APS_PFM);
	if (ret < 0)
		return ret;

	pmic_reg_read(pfuze, PFUZE100_DEVICEID, &value);
	pmic_reg_read(pfuze, PFUZE100_REVID, &rev_id);
	printf("PMIC: PFUZE200! DEV_ID=0x%x REV_ID=0x%x\n", value, rev_id);

	/*
	 * Our PFUZE0200 is PMPF0200X0AEP, the Pre-programmed OTP
	 * Configuration is F0.
	 * Default VOLT:
	 * VSNVS_VOLT	|	3.0V
	 * SW1AB	|	1.375V
	 * SW2		|	3.3V
	 * SW3A		|	1.5V
	 * SW3B		|	1.5V
	 * VGEN1	|	1.5V
	 * VGEN2	|	1.5V
	 * VGEN3	|	2.5V
	 * VGEN4	|	1.8V
	 * VGEN5	|	2.8V
	 * VGEN6	|	3.3V
	 *
	 * According to schematic, we need SW3A 1.35V, SW3B 3.3V,
	 * VGEN1 1.2V, VGEN2 1.5V, VGEN3 2.8V, VGEN4 1.8V,
	 * VGEN5 3.3V, VGEN6 3.0V.
	 *
	 * Here we just use the default VOLT, but not configure
	 * them, when needed, configure them to our requested voltage.
	 */

	/* set SW1AB standby volatage 0.975V */
	pmic_reg_read(pfuze, PFUZE100_SW1ABSTBY, &value);
	value &= ~0x3f;
	value |= PFUZE100_SW1ABC_SETP(9750);
	pmic_reg_write(pfuze, PFUZE100_SW1ABSTBY, value);

	/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
	pmic_reg_read(pfuze, PFUZE100_SW1ABCONF, &value);
	value &= ~0xc0;
	value |= 0x40;
	pmic_reg_write(pfuze, PFUZE100_SW1ABCONF, value);

	/* Enable power of VGEN5 3V3 */
	pmic_reg_read(pfuze, PFUZE100_VGEN5VOL, &value);
	value &= ~0x1F;
	value |= 0x1F;
	pmic_reg_write(pfuze, PFUZE100_VGEN5VOL, value);
#endif

	return 0;
}

#if 0
#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	int is_400M;
	u32 vddarm;

	struct pmic *p = pfuze;

	if (!p) {
		printf("No PMIC found!\n");
		return;
	}

	/* switch to ldo_bypass mode */
	if (ldo_bypass) {
		prep_anatop_bypass();
		/* decrease VDDARM to 1.275V */
		pmic_reg_read(pfuze, PFUZE100_SW1ABVOL, &value);
		value &= ~0x3f;
		value |= PFUZE100_SW1ABC_SETP(12750);
		pmic_reg_write(pfuze, PFUZE100_SW1ABVOL, value);

		is_400M = set_anatop_bypass(1);
		if (is_400M)
			vddarm = PFUZE100_SW1ABC_SETP(10750);
		else
			vddarm = PFUZE100_SW1ABC_SETP(11750);

		pmic_reg_read(pfuze, PFUZE100_SW1ABVOL, &value);
		value &= ~0x3f;
		value |= vddarm;
		pmic_reg_write(pfuze, PFUZE100_SW1ABVOL, value);

		finish_anatop_bypass();

		printf("switch to ldo_bypass mode!\n");
	}
}
#endif
#else
void ldo_mode_set(int ldo_bypass)
{
}
#endif
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

	setup_fec(CONFIG_FEC_ENET_DEV);

#ifdef CONFIG_SYS_USE_QSPI
	board_qspi_init();
#endif

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1", MAKE_CFGVAL(0x42, 0x20, 0x00, 0x00)},
	{"qspi1", MAKE_CFGVAL(0x10, 0x00, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

int checkboard(void)
{
	puts("Board: NOVAsomU\n");

	return 0;
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)
iomux_v3_cfg_t const usb_otg1_pads[] = {
	MX6_PAD_GPIO1_IO04__USB_OTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO1_IO00__ANATOP_OTG1_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL),
};

/*
 * Leave it here, but default configuration only supports 1 port now,
 * because we need sd1 and i2c1
 */
iomux_v3_cfg_t const usb_otg2_pads[] = {
	/* conflict with i2c1_scl */
	MX6_PAD_GPIO1_IO02__USB_OTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* conflict with sd1_vselect */
	MX6_PAD_GPIO1_IO05__ANATOP_OTG2_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL),
};

int board_usb_phy_mode(int port)
{
	return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	switch (port) {
	case 0:
		imx_iomux_v3_setup_multiple_pads(usb_otg1_pads,
						 ARRAY_SIZE(usb_otg1_pads));
		break;
	case 1:
		imx_iomux_v3_setup_multiple_pads(usb_otg2_pads,
						 ARRAY_SIZE(usb_otg2_pads));
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}
#endif
