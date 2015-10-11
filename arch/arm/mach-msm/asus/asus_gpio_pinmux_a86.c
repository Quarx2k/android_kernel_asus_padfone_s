#include <linux/kernel.h>
#include <mach/gpiomux.h>

/////////////////////////////////////////////////////////////////////
//includ add asus platform gpio table
#include "a86_evb_gpio_pinmux.h"
#include "a86_sr1_gpio_pinmux.h"
#include "a86_sr2_gpio_pinmux.h"
#include "a86_sr3_gpio_pinmux.h"
#include "a86_sr4_gpio_pinmux.h"
#include "a86_er1_gpio_pinmux.h"
/////////////////////////////////////////////////////////////////////
//define asus gpio var.

int __init device_gpio_init_a86(void)
{
	switch (g_ASUS_hwID)
	{
		case A86_EVB:
		printk("[KERNEL] a86 gpio config table = EVB\n");

             msm_gpiomux_install(a86_evb_msm8974_gpio_configs,
             ARRAY_SIZE(a86_evb_msm8974_gpio_configs));

             break;

		case A86_SR1:
             printk("[KERNEL] a86 gpio config table = SR1\n");

		msm_gpiomux_install(a86_sr1_msm8974_gpio_configs,
             ARRAY_SIZE(a86_sr1_msm8974_gpio_configs));
             break;

		case A86_SR2:
             printk("[KERNEL] a86 gpio config table = SR2\n");

		msm_gpiomux_install(a86_sr2_msm8974_gpio_configs,
             ARRAY_SIZE(a86_sr2_msm8974_gpio_configs));
             break;

		case A86_SR3:
             printk("[KERNEL] a86 gpio config table = SR3\n");

		msm_gpiomux_install(a86_sr3_msm8974_gpio_configs,
             ARRAY_SIZE(a86_sr3_msm8974_gpio_configs));
             break;

		case A86_SR4:
             printk("[KERNEL] a86 gpio config table = SR4\n");

		msm_gpiomux_install(a86_sr4_msm8974_gpio_configs,
             ARRAY_SIZE(a86_sr4_msm8974_gpio_configs));
             break;

		case A86_ER1:
		case A86_PR:
		case A86_MP:
             printk("[KERNEL] a86 gpio config table = ER1\n");

		msm_gpiomux_install(a86_er1_msm8974_gpio_configs,
             ARRAY_SIZE(a86_er1_msm8974_gpio_configs));
             break;

		default:
             printk(KERN_ERR "[ERROR] There is NO valid hardware ID\n");
             msm_gpiomux_install(a86_sr2_msm8974_gpio_configs,
             ARRAY_SIZE(a86_sr2_msm8974_gpio_configs));
             break;
   }
   return 0;
}
