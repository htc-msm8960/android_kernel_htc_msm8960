/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/camera.h>
#include <mach/msm_bus_board.h>
#include <mach/socinfo.h>
#include <mach/gpiomux.h>
#include "devices.h"
#include "board-ville.h"

#ifdef CONFIG_MSM_CAMERA

int camera_sensor_power_enable(char *power, unsigned volt, struct regulator **sensor_power)
{
	int rc;

	if (power == NULL)
		return -ENODEV;

	*sensor_power = regulator_get(NULL, power);

	if (IS_ERR(*sensor_power)) {
		pr_err("[CAM] %s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}

	if (volt != 1800000) {
		rc = regulator_set_voltage(*sensor_power, volt, volt);
		if (rc < 0) {
			pr_err("[CAM] %s: unable to set %s voltage to %d rc:%d\n",
					__func__, power, volt, rc);
			regulator_put(*sensor_power);
			*sensor_power = NULL;
			return -ENODEV;
		}
	}

	rc = regulator_enable(*sensor_power);
	if (rc < 0) {
		pr_err("[CAM] %s: Enable regulator %s failed\n", __func__, power);
		regulator_put(*sensor_power);
		*sensor_power = NULL;
		return -ENODEV;
	}

	return rc;
}

int camera_sensor_power_disable(struct regulator *sensor_power)
{

	int rc;
	if (sensor_power == NULL)
		return -ENODEV;

	if (IS_ERR(sensor_power)) {
		pr_err("[CAM] %s: Invalid requlator ptr\n", __func__);
		return -ENODEV;
	}

	rc = regulator_disable(sensor_power);
	if (rc < 0)
		pr_err("[CAM] %s: disable regulator failed\n", __func__);

	regulator_put(sensor_power);
	sensor_power = NULL;
	return rc;
}

static struct gpiomux_setting cam_settings[16] = {
	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},

	{
		.func = GPIOMUX_FUNC_1, /*active 1 - FUNC1 2MA*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 2*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_1, /*active 3 - FUNC1 8MA*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_2, /*active 4 - FUNC2 2MA*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 5 - I(L) 4MA*/
		.drv = GPIOMUX_DRV_4MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_2, /*active 6 - A FUNC2 4MA*/
		.drv = GPIOMUX_DRV_4MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 7 - I(NP) 2MA*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 8 - I(PD) 2MA*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 9 - O(H) 2MA*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_HIGH,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 10 - O(L) 8MA*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 11 - I(PU) 2MA*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 12 - O(L) 2MA*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},

	{
		.func = GPIOMUX_FUNC_2, /*active 13 - A FUNC2 8MA*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 14 - I(NP) 8MA*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 15 - I(PD) 8MA*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},
};

static struct msm_gpiomux_config msm8960_cam_common_configs[] = {
	{
		.gpio = VILLE_GPIO_CAM_MCLK1,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[13], /*A FUNC2 8MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[10], /*O(L) 8MA*/
		},
	},
	{
		.gpio = VILLE_GPIO_CAM_MCLK0,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],  /*Fun1 8MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[10], /*O(L) 8MA*/
		},
	},

	{
		.gpio = VILLE_GPIO_CAM_PWDN,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[9],       /*O(H) 2MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[12],  /*O(L) 2MA*/
		},
	},

	{
		.gpio = VILLE_GPIO_CAM_I2C_DAT,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3], /*FUNC1 8MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[15], /*I(PD) 8MA*/
		},
	},
	{
		.gpio = VILLE_GPIO_CAM_I2C_CLK,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3], /*FUNC1 8MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[15], /*I(PD) 8MA*/
		},
	},
	{
		.gpio = VILLE_GPIO_RAW_INTR0,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[7], /*I(NP) 2MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[8], /*I(PD) 2MA*/
		},
	},
	{
		.gpio = VILLE_GPIO_RAW_INTR1,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[7], /*I(NP) 2MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[8], /*I(PD) 2MA*/
		},
	},
	/* gpio config for Rawchip SPI - gsbi10 */
	{
		.gpio      = VILLE_GPIO_MCAM_SPI_CLK,
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_settings[13], /*A FUNC2 8MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[15], /* I(PD) 8MA */
		},
	},
	{
		.gpio      = VILLE_GPIO_MCAM_SPI_CS0,
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_settings[13], /*A FUNC2 8MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[15], /* I(PD) 8MA */
		},
	},
	{
		.gpio      = VILLE_GPIO_MCAM_SPI_DI,
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_settings[13], /*A FUNC2 8MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[15], /* I(PD) 8MA */
		},
	},
	{
		.gpio      = VILLE_GPIO_MCAM_SPI_DO,
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_settings[13], /*A FUNC2 8MA*/
			[GPIOMUX_SUSPENDED] = &cam_settings[15], /* I(PD) 8MA */
		},
	},
};

static struct msm_bus_vectors cam_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_preview_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 27648000,
		.ib  = 2656000000UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 600000000,
		.ib  = 2656000000UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 600000000,
		.ib  = 2656000000UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
};

static struct msm_bus_vectors cam_zsl_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 600000000,
		.ib  = 2656000000UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
};

static struct msm_bus_vectors cam_video_ls_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 348192000,
		.ib  = 617103360,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_dual_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 600000000,
		.ib  = 2656000000UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
};



static struct msm_bus_paths cam_bus_client_config[] = {
	{
		ARRAY_SIZE(cam_init_vectors),
		cam_init_vectors,
	},
	{
		ARRAY_SIZE(cam_preview_vectors),
		cam_preview_vectors,
	},
	{
		ARRAY_SIZE(cam_video_vectors),
		cam_video_vectors,
	},
	{
		ARRAY_SIZE(cam_snapshot_vectors),
		cam_snapshot_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_video_ls_vectors),
		cam_video_ls_vectors,
	},
	{
		ARRAY_SIZE(cam_dual_vectors),
		cam_dual_vectors,
	},
};

static struct msm_bus_scale_pdata cam_bus_client_pdata = {
		cam_bus_client_config,
		ARRAY_SIZE(cam_bus_client_config),
		.name = "msm_camera",
};

static struct msm_camera_device_platform_data msm_camera_csi_device_data[] = {
	{
		.csid_core = 0,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
	{
		.csid_core = 1,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
	{
		.csid_core = 2,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
};

static struct camera_vreg_t msm_8960_cam_vreg[] = {
  //	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000},
  //	{"cam_vio", REG_VS, 0, 0, 0},
  //	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
  //	{"cam_vaf", REG_LDO, 2800000, 2800000, 300000},
};

static struct gpio msm8960_common_cam_gpio[] = {
  {VILLE_GPIO_CAM_MCLK0, 0, "CAM_GPIO"}, /*CAMIF_MCLK*/
  {VILLE_GPIO_CAM_MCLK1, 0, "CAM_GPIO"},
  {VILLE_GPIO_RAW_INTR0, 0, "CAM_GPIO"},
  {VILLE_GPIO_RAW_INTR1, 0, "CAM_GPIO"},
  {VILLE_GPIO_MCAM_SPI_CLK, 0, "CAM_GPIO"},
  {VILLE_GPIO_MCAM_SPI_CS0, 0, "CAM_GPIO"},
  {VILLE_GPIO_MCAM_SPI_DI, 0, "CAM_GPIO"},
  {VILLE_GPIO_MCAM_SPI_DO, 0, "CAM_GPIO"},
};

static struct gpio msm8960_front_cam_gpio[] = {
  {VILLE_GPIO_V_LCMIO_1V8_EN, GPIOF_DIR_OUT, "CAM2_IO"},
  {VILLE_GPIO_CAM2_RSTz, GPIOF_DIR_OUT, "CAM2_RSTz"},
};

static struct gpio msm8960_back_cam_gpio[] = {
  {VILLE_GPIO_CAM_PWDN, GPIOF_DIR_OUT, "CAM_PWDN"},
};

static struct msm_gpio_set_tbl msm8960_front_cam_gpio_set_tbl[] = {
  {VILLE_GPIO_V_LCMIO_1V8_EN, GPIOF_OUT_INIT_LOW, 1000},
  {VILLE_GPIO_V_LCMIO_1V8_EN, GPIOF_OUT_INIT_HIGH, 4000},
  {VILLE_GPIO_CAM2_RSTz, GPIOF_OUT_INIT_LOW, 1000},
  {VILLE_GPIO_CAM2_RSTz, GPIOF_OUT_INIT_HIGH, 4000},
};

static struct msm_gpio_set_tbl msm8960_back_cam_gpio_set_tbl[] = {
  {VILLE_GPIO_CAM_PWDN, GPIOF_OUT_INIT_LOW, 1000},
  {VILLE_GPIO_CAM_PWDN, GPIOF_OUT_INIT_HIGH, 4000},
};

static struct msm_camera_gpio_conf msm_8960_front_cam_gpio_conf = {
	.cam_gpio_common_tbl = msm8960_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(msm8960_common_cam_gpio),
	.cam_gpio_req_tbl = msm8960_front_cam_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(msm8960_front_cam_gpio),
	.cam_gpio_set_tbl = msm8960_front_cam_gpio_set_tbl,
	.cam_gpio_set_tbl_size = ARRAY_SIZE(msm8960_front_cam_gpio_set_tbl),
};

static struct msm_camera_gpio_conf msm_8960_back_cam_gpio_conf = {
  	.cam_gpio_common_tbl = msm8960_common_cam_gpio,
  	.cam_gpio_common_tbl_size = ARRAY_SIZE(msm8960_common_cam_gpio),
  	.cam_gpio_req_tbl = msm8960_back_cam_gpio,
  	.cam_gpio_req_tbl_size = ARRAY_SIZE(msm8960_back_cam_gpio),
  	.cam_gpio_set_tbl = msm8960_back_cam_gpio_set_tbl,
  	.cam_gpio_set_tbl_size = ARRAY_SIZE(msm8960_back_cam_gpio_set_tbl),
};

static struct i2c_board_info msm_act_main_cam_i2c_info = {
	I2C_BOARD_INFO("msm_actuator", 0x11),
};

static struct msm_camera_sensor_flash_data flash_mt9v113 = {
	.flash_type = MSM_CAMERA_FLASH_NONE
};

static struct msm_camera_csi_lane_params mt9v113_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x3,
};

static struct camera_vreg_t mt9v113_cam_vreg[] = {
	{"8921_l9", REG_LDO, 2800000, 2800000, 105000, 0},
	{"8921_lvs6", REG_VS, 1800000, 1800000, 1800000, 0},
	{"8921_l8", REG_LDO, 2800000, 2800000, 85600, 00},
};

static struct msm_camera_sensor_platform_info sensor_board_info_mt9v113 = {
	.mount_angle = 90,
	.cam_vreg = mt9v113_cam_vreg,
	.num_vreg = ARRAY_SIZE(mt9v113_cam_vreg),
	.gpio_conf = &msm_8960_front_cam_gpio_conf,
	.csi_lane_params = &mt9v113_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9v113_data = {
	.sensor_name = "mt9v113",
	.pdata = &msm_camera_csi_device_data[1],
	.flash_data = &flash_mt9v113,
	.sensor_platform_info = &sensor_board_info_mt9v113,
	.csi_if = 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = YUV_SENSOR,
        .vcm_enable = 1,
        .vcm_pwd = 0,
};

static struct msm_camera_sensor_flash_data flash_s5k3h2yx = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_csi_lane_params s5k3h2yx_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x3,
};

static struct msm_camera_sensor_platform_info sensor_board_info_s5k3h2yx = {
	.mount_angle  = 90,
	.cam_vreg = msm_8960_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_cam_vreg),
	.gpio_conf = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &s5k3h2yx_csi_lane_params,
};

static struct msm_actuator_info msm_act_main_cam_0_info = {
	.board_info     = &msm_act_main_cam_i2c_info,
	.cam_name   = MSM_ACTUATOR_MAIN_CAM_0,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3h2yx_data = {
	.sensor_name          = "s5k3h2yx",
	.pdata                = &msm_camera_csi_device_data[0],
	.flash_data           = &flash_s5k3h2yx,
	.sensor_platform_info = &sensor_board_info_s5k3h2yx,
	.csi_if               = 1,
	.camera_type          = BACK_CAMERA_2D,
	.sensor_type          = BAYER_SENSOR,
	.actuator_info    = &msm_act_main_cam_0_info,
	.use_rawchip = RAWCHIP_ENABLE,
};

static struct platform_device msm_camera_server = {
	.name = "msm_cam_server",
	.id = 0,
};

void __init ville_init_camera(void)
{
	msm_gpiomux_install(msm8960_cam_common_configs,
			ARRAY_SIZE(msm8960_cam_common_configs));

        ville_init_rawchip();
	platform_device_register(&msm_camera_server);
	platform_device_register(&msm8960_device_i2c_mux_gsbi4);
	platform_device_register(&msm8960_device_csiphy0);
	platform_device_register(&msm8960_device_csiphy1);
	platform_device_register(&msm8960_device_csid0);
	platform_device_register(&msm8960_device_csid1);
	platform_device_register(&msm8960_device_ispif);
	platform_device_register(&msm8960_device_vfe);
	platform_device_register(&msm8960_device_vpe);
}

#ifdef CONFIG_I2C
static struct i2c_board_info msm8960_camera_i2c_boardinfo[] = {
#ifdef CONFIG_MT9V113
	{
          I2C_BOARD_INFO("mt9v113", 0x3C),
          .platform_data = &msm_camera_sensor_mt9v113_data,
	},
#endif
#ifdef CONFIG_S5K3H2YX
        {
          I2C_BOARD_INFO("s5k3h2yx", 0x20 >> 1),
          .platform_data = &msm_camera_sensor_s5k3h2yx_data,
        },
#endif
};

struct msm_camera_board_info ville_camera_board_info = {
	.board_info = msm8960_camera_i2c_boardinfo,
	.num_i2c_board_info = ARRAY_SIZE(msm8960_camera_i2c_boardinfo),
};
#endif
#endif
