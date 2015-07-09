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
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/memory.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/krait-regulator.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>
#include <asm/mach/map.h>
#include <asm/hardware/gic.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/msm_iomap.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_smd.h>
#include <mach/restart.h>
#include <mach/rpm-smd.h>
#include <mach/rpm-regulator-smd.h>
#include <mach/socinfo.h>
#include <mach/msm_smem.h>
#include "../board-dt.h"
#include "../clock.h"
#include "../devices.h"
#include "../spm.h"
#include "../pm.h"
#include "../modem_notifier.h"
#include "../platsmp.h"
#include <linux/persistent_ram.h>

static struct platform_device *ram_console_dev;

static struct persistent_ram_descriptor msm_prd[] __initdata = {
	{
		.name = "ram_console",
		.size = SZ_1M,
	},
};

static struct persistent_ram msm_pr __initdata = {
	.descs = msm_prd,
	.num_descs = ARRAY_SIZE(msm_prd),
	.start = PLAT_PHYS_OFFSET + SZ_1G + SZ_256M,
	.size = SZ_1M,
};

void __init msm_8974_reserve(void)
{
	persistent_ram_early_init(&msm_pr);
	of_scan_flat_dt(dt_scan_for_memory_reserve, NULL);
}

#ifdef CONFIG_BATTERY_ASUS
static struct resource a86_asus_bat_resources[] = {
	{
		.name = "bat_low_gpio",
		.start = 46,
		.end = 46,
		.flags = IORESOURCE_IO,
	},
};
static struct platform_device a86_asus_bat_device = {
	.name = "asus_bat",
	.id = 0,
	.num_resources = ARRAY_SIZE(a86_asus_bat_resources),
	.resource = a86_asus_bat_resources,	
};	

//ASUS BSP Eason_Chang : A86 porting ---
//ASUS BSP Hank_Chen : A86 1032 porting+++
static struct platform_device *msm_a86_bat_devices[] = {
	&a86_asus_bat_device,
};
#endif  //CONFIG_BATTERY_ASUS 

//ASUS BSP Hank_Chen : A86 1032 porting---

// +++ ASUS_BSP : add novatek virtual key map Deeo
#define MAX_LEN		200 //ASUS_BSP Deeo : add for creating virtual_key_maps ++

static ssize_t novaTP_virtual_keys_register(struct kobject *kobj,
		     struct kobj_attribute *attr, char *buf)
{
	char *virtual_keys = 	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":270:1960:30:30" "\n" \
						__stringify(EV_KEY) ":" __stringify(KEY_HOME) ":540:1960:30:30" "\n" \
						__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":810:1960:30:30" "\n" ;

	return snprintf(buf, strnlen(virtual_keys, MAX_LEN) + 1 , "%s",	virtual_keys);
}

static struct kobj_attribute novaTP_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.elan-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &novaTP_virtual_keys_register,
};

static struct attribute *virtual_key_properties_attrs[] = {
	&novaTP_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group virtual_key_properties_attr_group = {
	.attrs = virtual_key_properties_attrs,
};

//struct kobject *nova_virtual_key_properties_kobj;
static void nv_init_vkeys_8974(void)
{
	int ret = 0;
	static struct kobject *nova_virtual_key_properties_kobj;

	nova_virtual_key_properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (nova_virtual_key_properties_kobj)
		ret = sysfs_create_group(nova_virtual_key_properties_kobj, &virtual_key_properties_attr_group);
	if (!nova_virtual_key_properties_kobj || ret)
		pr_err("[Touch_N] failed to create novaTP virtual key map!\n");

	return;
}
// --- ASUS_BSP : add novatek virtual key map Deeo

// ASUS_BSP +++ Victor_Fu "proximity driver"
#include <linux/ProximityBasic.h>
static proximity_resource a91_proximity_resources[] = {
    {
        .index = 0,
        .type = PROXIMITY_FILE_SOURCE,
        .algo_type = PROXIMITY_BODYSAR_NOTIFY_ALGO_TYPE,
        .initEventState = PROXIMITY_EVNET_FAR,
        .name = "proximity_test1",
    },
    {
        .index = 1,
        .type = PROXIMITY_FILE_SOURCE,
        .algo_type = PROXIMITY_BODYSAR_NOTIFY_ALGO_TYPE,
        .initEventState = PROXIMITY_EVNET_FAR,
        .name = "proximity_test2",
    },    
    {
        .index = 2,
        .type = PROXIMITY_CAP1106_SOURCE,
        .algo_type = PROXIMITY_BODYSAR_NOTIFY_ALGO_TYPE,
        .initEventState = PROXIMITY_EVNET_FAR,
        .name = "cap_test",
    }, 
};
static proximity_platform_data  a91_proximity_data = {
    .resource        = a91_proximity_resources,
    .nResource       = ARRAY_SIZE(a91_proximity_resources),
};

static struct platform_device a91_proximity_device = {
    .name = "proximity-core-sensor",
    .id = 0,
    .dev            = {
        .platform_data  = &a91_proximity_data,
    },
};	

static struct platform_device *msm_a91_proximity_devices[] = {
	&a91_proximity_device,
};
// ASUS_BSP --- Victor_Fu "proximity driver"


// ASUS_BSP +++ Peter_Lu "Lightsensor"
static struct platform_device cm36283_device = {
	.name = "cm36283",
	.id = 0,
};

static struct platform_device cm3628_device = {
	.name = "cm3628",
	.id = 0,
};

static struct platform_device *msm_a90_sensor_devices[] = {
	&cm36283_device,
};

static struct platform_device *msm_a91_sensor_devices[] = {
	&cm3628_device,
};
// ASUS_BSP ---

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8974_add_drivers(void)
{
	msm_init_modem_notifier_list();
	msm_smd_init();
	msm_rpm_driver_init();
	msm_pm_sleep_status_init();
	rpm_regulator_smd_driver_init();
	msm_spm_device_init();
	krait_power_init();
	if (of_board_is_rumi())
		msm_clock_init(&msm8974_rumi_clock_init_data);
	else
		msm_clock_init(&msm8974_clock_init_data);
	tsens_tm_init_driver();
	msm_thermal_device_init();
	nv_init_vkeys_8974(); // +++ ASUS_BSP : add novatek virtual key map Deeo

	// ASUS_BSP +++ Victor_Fu "proximity driver"
	platform_add_devices(msm_a91_proximity_devices, ARRAY_SIZE(msm_a91_proximity_devices));
	// ASUS_BSP --- Victor_Fu "proximity driver"
#ifdef CONFIG_BATTERY_ASUS
	//ASUS BSP Hank_Chen : A86 1032 porting+++
	platform_add_devices(msm_a86_bat_devices, ARRAY_SIZE(msm_a86_bat_devices));
	//ASUS BSP Hank_Chen : A86 1032 porting---
#endif
	// ASUS_BSP +++ Peter_Lu "cm3628 & cm36283 Lightsensor"
	if ( g_ASUS_hwID == A90_EVB0 )	{
		printk("Add_CM36283_sensor +++\n");
		platform_add_devices(msm_a90_sensor_devices, ARRAY_SIZE(msm_a90_sensor_devices));
	}else if( g_ASUS_hwID >= A91_SR1 && g_ASUS_hwID < A91_SR5 ) {
		printk("Add_CM3628_sensor +++\n");
		platform_add_devices(msm_a91_sensor_devices, ARRAY_SIZE(msm_a91_sensor_devices));
	}
	// ASUS_BSP ---
}

static struct of_dev_auxdata msm_hsic_host_adata[] = {
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, "msm_hsic_host", NULL),
	{}
};

static struct of_dev_auxdata msm8974_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("qcom,hsusb-otg", 0xF9A55000, \
			"msm_otg", NULL),
	OF_DEV_AUXDATA("qcom,ehci-host", 0xF9A55000, \
			"msm_ehci_host", NULL),
	OF_DEV_AUXDATA("qcom,dwc-usb3-msm", 0xF9200000, \
			"msm_dwc3", NULL),
	OF_DEV_AUXDATA("qcom,usb-bam-msm", 0xF9304000, \
			"usb_bam", NULL),
	OF_DEV_AUXDATA("qcom,spi-qup-v2", 0xF9924000, \
			"spi_qsd.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9824000, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98A4000, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9864000, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98E4000, \
			"msm_sdcc.4", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9824900, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98A4900, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9864900, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98E4900, \
			"msm_sdcc.4", NULL),
	OF_DEV_AUXDATA("qcom,msm-rng", 0xF9BFF000, \
			"msm_rng", NULL),
	OF_DEV_AUXDATA("qcom,qseecom", 0xFE806000, \
			"qseecom", NULL),
	OF_DEV_AUXDATA("qcom,mdss_mdp", 0xFD900000, "mdp.0", NULL),
	OF_DEV_AUXDATA("qcom,msm-tsens", 0xFC4A8000, \
			"msm-tsens", NULL),
	OF_DEV_AUXDATA("qcom,qcedev", 0xFD440000, \
			"qcedev.0", NULL),
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, \
			"msm_hsic_host", NULL),
	OF_DEV_AUXDATA("qcom,hsic-smsc-hub", 0, "msm_smsc_hub",
			msm_hsic_host_adata),
	{}
};

static void __init msm8974_map_io(void)
{
	msm_map_8974_io();
}

static void __init asus_config_ramconsole(void)
{
	int ret;

	ram_console_dev = platform_device_alloc("ram_console", -1);
	if (!ram_console_dev) {
		pr_err("%s: Unable to allocate memory for RAM console device",
				__func__);
		return;
	}

	ret = platform_device_add(ram_console_dev);
	if (ret) {
		pr_err("%s: Unable to add RAM console device", __func__);
		return;
	}
}

//++ASUS_BSP : add for miniporting
#include <linux/init.h>
#include <linux/ioport.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
extern int __init device_gpio_init(void);
void __init device_gpiomux_init(void)
{
	int rc;

	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}

	device_gpio_init();

}
//--ASUS_BSP : add for miniporting

void __init msm8974_init(void)
{
	struct of_dev_auxdata *adata = msm8974_auxdata_lookup;

	/*
	 * populate devices from DT first so smem probe will get called as part
	 * of msm_smem_init.  socinfo_init needs smem support so call
	 * msm_smem_init before it.  msm_8974_init_gpiomux needs socinfo so
	 * call socinfo_init before it.
	 */
	board_dt_populate(adata);

	msm_smem_init();

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	device_gpiomux_init();
	regulator_has_full_constraints();
	msm8974_add_drivers();
	asus_config_ramconsole();
}

static const char *msm8974_dt_match[] __initconst = {
	"qcom,msm8974",
	"qcom,apq8074",
	NULL
};

DT_MACHINE_START(MSM8974_DT, "Qualcomm MSM 8974 (Flattened Device Tree)")
	.map_io = msm8974_map_io,
	.init_irq = msm_dt_init_irq,
	.init_machine = msm8974_init,
	.handle_irq = gic_handle_irq,
	.timer = &msm_dt_timer,
	.dt_compat = msm8974_dt_match,
	.reserve = msm_8974_reserve,
	.restart = msm_restart,
	.smp = &msm8974_smp_ops,
MACHINE_END
