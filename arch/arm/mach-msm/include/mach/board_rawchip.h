#ifndef __ASM_ARCH_MSM_BOARD_RAWCHIP_H
#define __ASM_ARCH_MSM_BOARD_RAWCHIP_H

#include <linux/clk.h>

struct msm_camera_rawchip_info {
	int rawchip_reset;
	int rawchip_intr0;
	int rawchip_intr1;
	uint8_t rawchip_spi_freq;
	uint8_t rawchip_mclk_freq;
	int (*camera_rawchip_power_on)(void);
	int (*camera_rawchip_power_off)(void);
	int (*rawchip_use_ext_1v2)(void);
        struct device *dev;
        struct clk *cam_clk[1];
};

enum rawchip_enable_type {
	RAWCHIP_DISABLE,
	RAWCHIP_ENABLE,
	RAWCHIP_DXO_BYPASS,
	RAWCHIP_MIPI_BYPASS,
};

enum sensor_flip_mirror_info {
	CAMERA_SENSOR_NONE,
	CAMERA_SENSOR_MIRROR,
	CAMERA_SENSOR_FLIP,
	CAMERA_SENSOR_MIRROR_FLIP,
};

enum msm_camera_pixel_order_default {
	MSM_CAMERA_PIXEL_ORDER_GR,
	MSM_CAMERA_PIXEL_ORDER_RG,
	MSM_CAMERA_PIXEL_ORDER_BG,
	MSM_CAMERA_PIXEL_ORDER_GB,
};

enum htc_camera_image_type_board {
	HTC_CAMERA_IMAGE_NONE_BOARD,
	HTC_CAMERA_IMAGE_YUSHANII_BOARD,
	HTC_CAMERA_IMAGE_MAX_BOARD,
};

#endif /* !__ASM_ARCH_MSM_BOARD_RAWCHIP_H */
