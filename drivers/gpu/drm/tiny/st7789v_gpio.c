+// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for Sitronix ST7789b panels with parallel bus interface
 *
 * Based on ili9341_gpio.c:
 * Copyright 2020 Mikhail Durnev <mikhail_dur...@mentor.com>
 *
 * Based on ili9341.c:
 * Copyright 2018 David Lechner <da...@lechnology.com>
 *
 * Based on mi0283qt.c:
 * Copyright 2016 Noralf Trønnes
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modeset_helper.h>
#include <video/mipi_display.h>

#define	ST7789V_RAMCTRL	0xB0
#define	ST7789V_PORCTRL	0xB2
#define	ST7789V_GCTRL	0xB7
#define	ST7789V_VCOMS	0xBB
#define	ST7789V_LCMCTRL	0xC0
#define	ST7789V_VDVVRHEN	0xC2
#define	ST7789V_VRHS	0xC3
#define	ST7789V_VDVS	0xC4
#define	ST7789V_PWCTRL1	0xD0
#define	ST7789V_PVGAMCTRL	0xE0
#define	ST7789V_NVGAMCTRL	0xE1

#define MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
#define MADCTL_MV BIT(5) /* bitmask for page/column order */
#define MADCTL_MX BIT(6) /* bitmask for column address order */
#define MADCTL_MY BIT(7) /* bitmask for page address order */

static void t-display_enable(struct drm_simple_display_pipe *pipe,
                            struct drm_crtc_state *crtc_state,
                            struct drm_plane_state *plane_state)
{
       struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
       struct mipi_dbi *dbi = &dbidev->dbi;
       u8 addr_mode;
       int ret, idx;

       if (!drm_dev_enter(pipe->crtc.dev, &idx))
               return;

       DRM_DEBUG_KMS("\n");

       ret = mipi_dbi_poweron_conditional_reset(dbidev);
       if (ret < 0)
               goto out_exit;
       if (ret == 1)
               goto out_enable;

       mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_OFF);
	   
	   /* Sleep Out */
	   mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
       msleep(120);
	   /* Interface Pixel Format */
	   mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);
	   /* RAM Control */
	   mipi_dbi_command(dbi, ST7789V_RAMCTRL, 0x00, 0xf0);
	   /* Porch Setting */
	   mipi_dbi_command(dbi, ST7789V_PORCTRL, 0x0c, 0x0c, 0x00, 0x33, 0x33);
	   /* Gate Control */
	   mipi_dbi_command(dbi, ST7789V_GCTRL, 0x35);
	   /* VCOMS Setting */
	   mipi_dbi_command(dbi, ST7789V_VCOMS, 0x19);
	   /* LCM Control */
	   mipi_dbi_command(dbi, ST7789V_LCMCTRL, 0x2c);
	   /* VDV and VRH Command Enable */
	   mipi_dbi_command(dbi, ST7789V_VDVVRHEN, 0x01);
	   /* VRH Set */
	   mipi_dbi_command(dbi, ST7789V_VRHS, 0x12);
	   /* VDV Set */
	   mipi_dbi_command(dbi, ST7789V_VDVS, 0x20);
	   /* Power Control 1 */
	   mipi_dbi_command(dbi, ST7789V_PWCTRL1, 0xa4, 0xa1);
	   /* Positive Voltage Gamma Control */
	   mipi_dbi_command(dbi, ST7789V_PVGAMCTRL,
						0xf0, 0x09, 0x13, 0x12, 0x12, 0x2b, 0x3c,
						0x44, 0x4b, 0x1b, 0x18, 0x17, 0x1d, 0x21);
	   /* Negative Voltage Gamma Control */
	   mipi_dbi_command(dbi, ST7789V_NVGAMCTRL, 
						0xf0, 0x09, 0x13, 0x0c, 0x0d, 0x27, 0x3b,
						0x44, 0x4d, 0x0b, 0x17, 0x17, 0x1d, 0x21);
	   /* Normal Display Mode On */
	   mipi_dbi_command(dbi, MIPI_DCS_ENTER_NORMAL_MODE);
	   msleep(10);
	   /* Display On */
	   mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);
	   msleep(100);

out_enable:
       switch (dbidev->rotation) {
       default:
               addr_mode = MADCTL_MX;
               break;
       case 90:
               addr_mode = MADCTL_MV;
               break;
       case 180:
               addr_mode = MADCTL_MY;
               break;
       case 270:
               addr_mode = MADCTL_MV | MADCTL_MY | MADCTL_MX;
               break;
       }
       addr_mode |= MADCTL_BGR;
	   /* Memory Data Access Control */
       mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
       mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
       drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs st7789vgpio_pipe_funcs = {
       .enable = t-display_enable,
       .disable = mipi_dbi_pipe_disable,
       .update = mipi_dbi_pipe_update,
       .prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode t-display_mode = {
       DRM_SIMPLE_MODE(170, 320, 25, 46),
};

DEFINE_DRM_GEM_DMA_FOPS(st7789vgpio_fops);

static struct drm_driver st7789vgpio_driver = {
       .driver_features        = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
       .fops                   = &st7789vgpio_fops,
DRM_GEM_DMA_DRIVER_OPS_VMAP,
       .debugfs_init           = mipi_dbi_debugfs_init,
       .name                   = "st7789vgpio",
       .desc                   = "Sitronix ST7789v",
       .date                   = "20240221",
       .major                  = 1,
       .minor                  = 0,
};

static const struct of_device_id st7789vgpio_of_match[] = {
       { .compatible = "sitronix,st7789v" },
       { }
};
MODULE_DEVICE_TABLE(of, st7789vgpio_of_match);

static int st7789vgpio_probe(struct platform_device *pdev)
{
       struct device *dev = &pdev->dev;
       struct mipi_dbi_dev *dbidev;
       struct drm_device *drm;
       struct mipi_dbi *dbi;
       struct gpio_desc *dc;
       struct gpio_desc *wr;
       struct gpio_descs *db;
       u32 rotation = 0;
       u32 wr_delays[2] = {15, 60};
       int ret;

       dbidev = devm_drm_dev_alloc(dev, &st7789vgpio_driver,
                                   struct mipi_dbi_dev, drm);
       if (IS_ERR(dbidev))
               return PTR_ERR(dbidev);

       dbi = &dbidev->dbi;
       drm = &dbidev->drm;

       dbi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
       if (IS_ERR(dbi->reset)) {
               DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
               return PTR_ERR(dbi->reset);
       }

       dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_HIGH);
       if (IS_ERR(dc)) {
               DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
               return PTR_ERR(dc);
       }

       wr = devm_gpiod_get(dev, "wr", GPIOD_OUT_HIGH);
       if (IS_ERR(wr)) {
               DRM_DEV_ERROR(dev, "Failed to get gpio 'wr'\n");
               return PTR_ERR(wr);
       }

       db = devm_gpiod_get_array(dev, "db", GPIOD_OUT_LOW);
       if (IS_ERR(db)) {
               DRM_DEV_ERROR(dev, "Failed to get gpio 'db'\n");
               return PTR_ERR(db);
       }
       if (db->ndescs != 16 && db->ndescs != 8) {
               /*
                * The data bus can be either 8 or 16 bits wide.
                * ST7789c can work with 8, 9, and 16-bit parallel interfaces.
                */
               DRM_DEV_ERROR(dev, "Wrong number of bits in gpio 'db': %u\n", db->ndescs);
               return PTR_ERR(db);
       }

       dbidev->backlight = devm_of_find_backlight(dev);
       if (IS_ERR(dbidev->backlight))
               return PTR_ERR(dbidev->backlight);

       device_property_read_u32(dev, "rotation", &rotation);

       device_property_read_u32_array(dev, "wr-up-down-delays", wr_delays, 2);

       ret = mipi_dbi_gpio_init(dbi, dc, wr, db, wr_delays[0], wr_delays[1]);
       if (ret)
               return ret;

       ret = mipi_dbi_dev_init(dbidev, &st7789vgpio_pipe_funcs, &t-display_mode, rotation);
       if (ret)
               return ret;

       drm_mode_config_reset(drm);

       ret = drm_dev_register(drm, 0);
       if (ret)
               return ret;

       platform_set_drvdata(pdev, drm);

       drm_fbdev_generic_setup(drm, 0);

       return 0;
}

static int st7789vgpio_remove(struct platform_device *pdev)
{
       struct drm_device *drm = platform_get_drvdata(pdev);

       drm_dev_unplug(drm);
       drm_atomic_helper_shutdown(drm);

       return 0;
}

static void st7789vgpio_shutdown(struct platform_device *pdev)
{
       drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static struct platform_driver st7789vgpio_platform_driver = {
       .driver = {
               .name = "st7789vgpio",
               .of_match_table = st7789vgpio_of_match,
       },
       .probe = st7789vgpio_probe,
       .remove = st7789vgpio_remove,
       .shutdown = st7789vgpio_shutdown,
};
module_platform_driver(st7789vgpio_platform_driver);

MODULE_DESCRIPTION("Sitronix ST7789b 8/16-bit DRM driver");
MODULE_AUTHOR("Trace Guy <jo...@gmail.com>");
MODULE_LICENSE("GPL");
