// SPDX-License-Identifier: GPL-2.0

/* Copyright (c) 2018 Rockchip Electronics Co. Ltd. */

#include <asm/cacheflush.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif

#include "sfc.h"
#include "rkflash_api.h"
#include "rkflash_blk.h"

#define RKSFC_VERSION_AND_DATE		"rksfc_base v1.1 2016-01-08"
#define RKSFC_CLK_MAX_RATE		(150 * 1000 * 1000)
#define RKSFC_DLL_THRESHOLD_RATE	(50 * 1000 * 1000)

struct rksfc_info {
	void __iomem	*reg_base;
	int	irq;
	unsigned long	clk_rate;
	struct clk	*clk;		/* sfc clk*/
	struct clk	*ahb_clk;	/* ahb clk gate*/
	u16	dll_cells[SFC_MAX_CHIP_SELECT_NUM];
	struct gpio_desc **cs_gpiods;
	int num_cs_gpios;
	bool sclk_x2_bypass;
};

static struct rksfc_info g_sfc_info;
static struct device *g_sfc_dev;
static struct completion sfc_irq_complete;

static int rksfc_clk_set_rate(struct rksfc_info *sfc, unsigned long speed)
{
	if (sfc_get_version() < SFC_VER_8 || sfc->sclk_x2_bypass)
		return clk_set_rate(sfc->clk, speed);
	else
		return clk_set_rate(sfc->clk, speed * 2);
}

static unsigned long rksfc_clk_get_rate(struct rksfc_info *sfc)
{
	if (sfc_get_version() < SFC_VER_8 || sfc->sclk_x2_bypass)
		return clk_get_rate(sfc->clk);
	else
		return clk_get_rate(sfc->clk) / 2;
}

void rksfc_set_cs_gpio(u8 cs, bool enable)
{
	if (g_sfc_info.cs_gpiods && cs < g_sfc_info.num_cs_gpios) {
		if (g_sfc_info.cs_gpiods[cs])
			gpiod_set_value_cansleep(g_sfc_info.cs_gpiods[cs], enable);
	}
}

static int rksfc_get_gpio_descs(struct device *dev)
{
	int nb, i;
	struct gpio_desc **cs;

	nb = gpiod_count(dev, "sfc-cs");
	if (nb == 0 || nb == -ENOENT)
		return 0;
	else if (nb < 0)
		return nb;

	cs = devm_kcalloc(dev, nb, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;
	g_sfc_info.cs_gpiods = cs;
	g_sfc_info.num_cs_gpios = nb;

	for (i = 0; i < nb; i++) {
		cs[i] = devm_gpiod_get_index_optional(dev, "sfc-cs", i, GPIOD_OUT_HIGH);
		if (IS_ERR(cs[i]))
			return PTR_ERR(cs[i]);

		if (cs[i]) {
			char *gpioname;

			gpioname = devm_kasprintf(dev, GFP_KERNEL, "%s CS%d",
						  dev_name(dev), i);
			if (!gpioname)
				return -ENOMEM;
			gpiod_set_consumer_name(cs[i], gpioname);
			rksfc_set_cs_gpio(i, false);
		}
	}

	return 0;
}

unsigned long rksfc_dma_map_single(unsigned long ptr, int size, int dir)
{
	return dma_map_single(g_sfc_dev, (void *)ptr, size
		, dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
}

void rksfc_dma_unmap_single(unsigned long ptr, int size, int dir)
{
	dma_unmap_single(g_sfc_dev, (dma_addr_t)ptr, size
		, dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
}

static irqreturn_t rksfc_interrupt(int irq, void *dev_id)
{
	sfc_clean_irq();
	complete(&sfc_irq_complete);
	return IRQ_HANDLED;
}

void rksfc_irq_flag_init(void)
{
	init_completion(&sfc_irq_complete);
}

void rksfc_wait_for_irq_completed(void)
{
	wait_for_completion_timeout(&sfc_irq_complete,
				    msecs_to_jiffies(10));
}

static int rksfc_irq_config(int mode, void *pfun)
{
	int ret = 0;
	int irq = g_sfc_info.irq;

	if (mode)
		ret = request_irq(irq, pfun, 0, "rksfc",
				  g_sfc_info.reg_base);
	else
		free_irq(irq,  NULL);
	return ret;
}

static int rksfc_irq_init(void)
{
	init_completion(&sfc_irq_complete);
	rksfc_irq_config(1, rksfc_interrupt);
	return 0;
}

static int rksfc_irq_deinit(void)
{
	rksfc_irq_config(0, rksfc_interrupt);
	return 0;
}

static void rksfc_delay_lines_tuning(u8 cs)
{
	u8 id[3], id_temp[3];
	struct rk_sfc_op op;
	u16 cell_max = (u16)sfc_get_max_dll_cells();
	u16 right, left = 0;
	u16 step = SFC_DLL_TRANING_STEP;
	bool dll_valid = false;

	op.sfcmd.d32 = 0;
	op.sfcmd.b.cmd = 0x9F;
	op.sfcmd.b.cs = cs;
	op.sfctrl.d32 = 0;

	rksfc_clk_set_rate(&g_sfc_info, RKSFC_DLL_THRESHOLD_RATE);
	sfc_request(&op, 0, id, 3);
	if ((0xFF == id[0] && 0xFF == id[1]) ||
	    (0x00 == id[0] && 0x00 == id[1])) {
		dev_dbg(g_sfc_dev, "no dev, dll by pass\n");

		return;
	}

	rksfc_clk_set_rate(&g_sfc_info, g_sfc_info.clk_rate);
	for (right = 0; right <= cell_max; right += step) {
		int ret;

		sfc_set_delay_lines(right, cs);
		sfc_request(&op, 0, id_temp, 3);
		dev_dbg(g_sfc_dev, "cs%d dll read flash id:%x %x %x\n",
			cs, id_temp[0], id_temp[1], id_temp[2]);

		ret = memcmp(&id, &id_temp, 3);
		if (dll_valid && ret) {
			right -= step;

			break;
		}
		if (!dll_valid && !ret)
			left = right;

		if (!ret)
			dll_valid = true;

		/* Add cell_max to loop */
		if (right == cell_max)
			break;
		if (right + step > cell_max)
			right = cell_max - step;
	}

	if (dll_valid && (right - left) >= SFC_DLL_TRANING_VALID_WINDOW) {
		if (left == 0 && right < cell_max)
			g_sfc_info.dll_cells[cs] = left + (right - left) * 2 / 5;
		else
			g_sfc_info.dll_cells[cs] = left + (right - left) / 2;
	} else {
		g_sfc_info.dll_cells[cs] = 0;
	}

	if (g_sfc_info.dll_cells[cs]) {
		dev_info(g_sfc_dev, "cs%d: %d %d %d dll training success in %luMHz max_cells=%u sfc_ver=%d\n",
			 cs, left, right, g_sfc_info.dll_cells[cs], g_sfc_info.clk_rate / 1000000,
			 sfc_get_max_dll_cells(), sfc_get_version());
		sfc_set_delay_lines((u16)g_sfc_info.dll_cells[cs], cs);
	} else {
		dev_err(g_sfc_dev, "cs%d: %d %d dll training failed in %luMHz, reduce the frequency\n",
			cs, left, right, g_sfc_info.clk_rate / 1000000);
		sfc_set_delay_lines(0, cs);
		rksfc_clk_set_rate(&g_sfc_info, RKSFC_DLL_THRESHOLD_RATE);
		g_sfc_info.clk_rate = rksfc_clk_get_rate(&g_sfc_info);
	}
}

static int rksfc_probe(struct platform_device *pdev)
{
	int irq;
	int ret;
	struct resource	*mem;
	void __iomem	*membase;
	int dev_result = -1;
	u32 val;
#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
	u32 status;
#endif

	g_sfc_dev = &pdev->dev;
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	membase = devm_ioremap_resource(&pdev->dev, mem);
	if (!membase) {
		dev_err(&pdev->dev, "no reg resource?\n");
		return -1;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}

	g_sfc_info.irq = irq;
	g_sfc_info.reg_base = membase;
	g_sfc_info.ahb_clk = devm_clk_get(&pdev->dev, "hclk_sfc");
	g_sfc_info.clk = devm_clk_get(&pdev->dev, "clk_sfc");
	if (unlikely(IS_ERR(g_sfc_info.clk)) ||
	    unlikely(IS_ERR(g_sfc_info.ahb_clk))) {
		dev_err(&pdev->dev, "%s get clk error\n", __func__);
		return -1;
	}

	g_sfc_info.sclk_x2_bypass = of_property_read_bool(pdev->dev.of_node, "rockchip,sclk-x2-bypass");

	if (!of_property_read_u32(pdev->dev.of_node, "spi-max-frequency", &val))
		g_sfc_info.clk_rate = val;

	clk_prepare_enable(g_sfc_info.ahb_clk);
	clk_prepare_enable(g_sfc_info.clk);

#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
	if (readl_poll_timeout(membase + SFC_SR, status,
			       !(status & SFC_BUSY), 10,
			       500 * USEC_PER_MSEC))
		dev_err(g_sfc_dev, "Wait for SFC idle timeout!\n");
#endif

	ret = rksfc_get_gpio_descs(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get gpio descs\n");
		return ret;
	}

	sfc_init(g_sfc_info.reg_base);
	rksfc_irq_init();

	if (!g_sfc_info.clk_rate)
		g_sfc_info.clk_rate = rksfc_clk_get_rate(&g_sfc_info);
	else if (g_sfc_info.clk_rate > RKSFC_CLK_MAX_RATE)
		g_sfc_info.clk_rate = RKSFC_CLK_MAX_RATE;
	ret = rksfc_clk_set_rate(&g_sfc_info, g_sfc_info.clk_rate);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set clk rate\n");
		return ret;
	}
	g_sfc_info.clk_rate = rksfc_clk_get_rate(&g_sfc_info);
	dev_info(&pdev->dev, "clk rate = %luHz\n", g_sfc_info.clk_rate);
	if (sfc_get_version() >= SFC_VER_4 && g_sfc_info.clk_rate > RKSFC_DLL_THRESHOLD_RATE)
		rksfc_delay_lines_tuning(0);
	else if (sfc_get_version() >= SFC_VER_4)
		sfc_set_delay_lines(0, 0);

#ifdef CONFIG_RK_SFC_NOR
	dev_result = rkflash_dev_init(g_sfc_info.reg_base, FLASH_TYPE_SFC_NOR, &sfc_nor_ops);
#endif
#ifdef CONFIG_RK_SFC_NAND
	if (dev_result)
		dev_result = rkflash_dev_init(g_sfc_info.reg_base, FLASH_TYPE_SFC_NAND, &sfc_nand_ops);
#endif

	if (dev_result)
		return dev_result;

	return dma_set_mask(g_sfc_dev, DMA_BIT_MASK(32));
}

static int __maybe_unused rksfc_suspend(struct device *dev)
{
	return rkflash_dev_suspend();
}

static int __maybe_unused rksfc_resume(struct device *dev)
{
	if (g_sfc_info.dll_cells[0])
		sfc_set_delay_lines(g_sfc_info.dll_cells[0], 0);
	return rkflash_dev_resume(g_sfc_info.reg_base);
}

static SIMPLE_DEV_PM_OPS(rksfc_pmops,
			 rksfc_suspend,
			 rksfc_resume);

static void rksfc_shutdown(struct platform_device *pdev)
{
	rkflash_dev_shutdown();
}

#ifdef CONFIG_OF
static const struct of_device_id of_rksfc_match[] = {
	{.compatible = "rockchip,sfc"},
	{}
};
#endif

static struct platform_driver rksfc_driver = {
	.probe		= rksfc_probe,
	.shutdown	= rksfc_shutdown,
	.driver		= {
		.name	= "rksfc",
#ifdef CONFIG_OF
		.of_match_table	= of_rksfc_match,
#endif
		.pm		= &rksfc_pmops,
	},
};

static void __exit rksfc_driver_exit(void)
{
	rkflash_dev_exit();
	rksfc_irq_deinit();
	platform_driver_unregister(&rksfc_driver);
}

static int __init rksfc_driver_init(void)
{
	int ret = 0;

	pr_err("%s\n", RKSFC_VERSION_AND_DATE);
	ret = platform_driver_register(&rksfc_driver);
	return ret;
}

module_init(rksfc_driver_init);
module_exit(rksfc_driver_exit);
MODULE_ALIAS("rksfc");
