#include <asm/mach-types.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <mach/msm_bus_board.h>
#include <mach/gpiomux.h>
#include <asm/setup.h>

#include "devices.h"
#include "board-ville.h"

#include <linux/spi/spi.h>

#ifdef CONFIG_RAWCHIP
static int ville_use_ext_1v2(void)
{
	return 0;
}

//static struct regulator *reg_8921_l2;
static struct regulator *reg_8921_l8;
static struct regulator *reg_8921_l9;
static struct regulator *reg_8921_lvs6;

static int ville_rawchip_vreg_on(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);

	/* VCM */
	rc = camera_sensor_power_enable("8921_l9", 2800000, &reg_8921_l9); // Mu Lee for sequence with raw chip 20120116
	if (rc < 0) {
		pr_err("[CAM] rawchip_power_enable(\"8921_l9\", 2.8V) FAILED %d\n", rc);
		goto enable_VCM_fail;
	}

	/* PM8921_lvs6 1800000 */
	rc = camera_sensor_power_enable("8921_lvs6", 1800000, &reg_8921_lvs6);
	if (rc < 0) {
		pr_err("[CAM] rawchip_power_enable(\"8921_lvs6\", 1.8V) FAILED %d\n", rc);
		goto enable_1v8_fail;
	}

	mdelay(5);

	/* digital */
	rc = gpio_request(VILLE_GPIO_V_CAM_D1V2_EN, "CAM_D1V2_EN");
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"gpio %d\", 1.2V) FAILED %d\n", VILLE_GPIO_V_CAM_D1V2_EN, rc);
		goto enable_1v2_fail;
	}
	gpio_direction_output(VILLE_GPIO_V_CAM_D1V2_EN, 1);
	gpio_free(VILLE_GPIO_V_CAM_D1V2_EN);

	/* analog */
	/* Mu Lee for sequence with raw chip 20120116 */
	rc = camera_sensor_power_enable("8921_l8", 2800000, &reg_8921_l8);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8921_l8\", 2.8V) FAILED %d\n", rc);
		goto enable_analog_fail;
	}

	/* LCMIO */
	rc = gpio_request(VILLE_GPIO_V_LCMIO_1V8_EN, "CAM_D1V8_EN"); /* Mu Lee for sequence with raw chip 20120116 */
	if (rc < 0) {
		pr_err("[CAM] %s:GPIO_CAM_D1V8_EN gpio %d request failed, rc=%d\n", __func__,  VILLE_GPIO_V_LCMIO_1V8_EN, rc);
		goto lcmio_hi_fail;
	}
	gpio_direction_output(VILLE_GPIO_V_LCMIO_1V8_EN, 1); /* Mu Lee for sequence with raw chip 20120116 */
	gpio_free(VILLE_GPIO_V_LCMIO_1V8_EN); /* Mu Lee for sequence with raw chip 20120116 */

	return rc;

lcmio_hi_fail:
	camera_sensor_power_disable(reg_8921_l8);
enable_analog_fail:
	gpio_request(VILLE_GPIO_V_CAM_D1V2_EN, "CAM_D1V2_EN");
	gpio_direction_output(VILLE_GPIO_V_CAM_D1V2_EN, 0);
	gpio_free(VILLE_GPIO_V_CAM_D1V2_EN);
enable_1v2_fail:
	camera_sensor_power_disable(reg_8921_lvs6);
enable_1v8_fail:
	camera_sensor_power_disable(reg_8921_l9);
enable_VCM_fail:
	return rc;
}

static int ville_rawchip_vreg_off(void)
{
	int rc = 0;

	pr_info("[CAM] %s\n", __func__);

	/* Mu Lee for sequence with raw chip 20120116 */
	rc = camera_sensor_power_disable(reg_8921_l8);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8921_l8\") FAILED %d\n", rc);
		goto ville_rawchip_vreg_off_fail;
	}

	rc = gpio_request(VILLE_GPIO_V_CAM_D1V2_EN, "CAM_D1V2_EN");
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"gpio %d\", 1.2V) FAILED %d\n", VILLE_GPIO_V_CAM_D1V2_EN, rc);
		goto ville_rawchip_vreg_off_fail;
	}
	gpio_direction_output(VILLE_GPIO_V_CAM_D1V2_EN, 0);
	gpio_free(VILLE_GPIO_V_CAM_D1V2_EN);

	udelay(50);

	rc = gpio_request(VILLE_GPIO_V_LCMIO_1V8_EN, "CAM_D1V8_EN");
	if (rc < 0) {
		pr_err("[CAM] %s:GPIO_CAM_D1V8_EN gpio %d request failed, rc=%d\n", __func__,  VILLE_GPIO_V_LCMIO_1V8_EN, rc);
		goto ville_rawchip_vreg_off_fail;
	}
	gpio_direction_output(VILLE_GPIO_V_LCMIO_1V8_EN, 0);
	gpio_free(VILLE_GPIO_V_LCMIO_1V8_EN);

	mdelay(5);

	rc = camera_sensor_power_disable(reg_8921_lvs6);
	if (rc < 0) {
		pr_err("[CAM] rawchip_power_disable(\"8921_lvs6\", 1.8V) FAILED %d\n", rc);
		goto ville_rawchip_vreg_off_fail;
	}

	/* VCM */
	/* Mu Lee for sequenc with raw chip 20120116 */
	rc = camera_sensor_power_disable(reg_8921_l9);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8921_l9\") FAILED %d\n", rc);
		goto ville_rawchip_vreg_off_fail;
	}

	return rc;

ville_rawchip_vreg_off_fail:
	return rc;
}

static struct msm_camera_rawchip_info msm_rawchip_board_info = {
	.rawchip_reset	= VILLE_GPIO_RAW_RSTN,
	.rawchip_intr0	= MSM_GPIO_TO_INT(VILLE_GPIO_RAW_INTR0),
        .rawchip_intr1	= MSM_GPIO_TO_INT(VILLE_GPIO_RAW_INTR1),
	.rawchip_spi_freq = 27, /* MHz, should be the same to spi max_speed_hz */
	.rawchip_mclk_freq = 24, /* MHz, should be the same as cam csi0 mclk_clk_rate */
	.camera_rawchip_power_on = ville_rawchip_vreg_on,
	.camera_rawchip_power_off = ville_rawchip_vreg_off,
	.rawchip_use_ext_1v2 = ville_use_ext_1v2,
};

static struct platform_device msm_rawchip_device = {
	.name	= "rawchip",
	.dev	= {
		.platform_data = &msm_rawchip_board_info,
	},
};

#endif

void __init ville_init_rawchip(void)
{
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_RAWCHIP
        platform_device_register(&msm_rawchip_device);
#endif
#endif
}
