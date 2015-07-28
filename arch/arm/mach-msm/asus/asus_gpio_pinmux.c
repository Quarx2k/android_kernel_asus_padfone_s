#include <linux/kernel.h>
#include <mach/gpiomux.h>

/////////////////////////////////////////////////////////////////////
//includ add asus platform gpio table
#ifdef ASUS_A91_PROJECT
#include "a90_evb0_gpio_pinmux.h"
#include "a90_evb_gpio_pinmux.h"
#include "a91_sr1_gpio_pinmux.h"
#include "a91_sr3_gpio_pinmux.h"
#include "a91_sr4_gpio_pinmux.h"
#include "a91_sr5_gpio_pinmux.h"
#include "a91_er1_gpio_pinmux.h"
#endif
#ifdef ASUS_PF500KL_PROJECT
#include "a90_evb0_gpio_pinmux.h"
#include "a90_evb_gpio_pinmux.h"
#include "a91_sr1_gpio_pinmux.h"
#include "a91_sr3_gpio_pinmux.h"
#include "a91_sr4_gpio_pinmux.h"
#include "a91_sr5_gpio_pinmux.h"
#include "a91_er1_gpio_pinmux.h"
#include "pf500kl_er1_gpio_pinmux.h"
#include "pf500kl_er2_2_gpio_pinmux.h"
#include "pf500kl_pr_gpio_pinmux.h"
#endif
/////////////////////////////////////////////////////////////////////
//define asus gpio var.

int __init device_gpio_init(void)
{
#ifdef ASUS_A91_PROJECT
	switch (g_ASUS_hwID)
	{
		case A90_EVB0:
		printk("[KERNEL] a90 gpio config table = EVB0\n");  
	   
             msm_gpiomux_install(a90_evb0_msm8974_gpio_configs,
             ARRAY_SIZE(a90_evb0_msm8974_gpio_configs));	 	  

             break;
			   
         	case A90_EVB:
         	case A90_SR1:	
         	case A90_SR2:		
         	case A90_SR3:	
         	case A90_ER1:
         	case A90_PR:		
         	case A90_MP:						
             printk("[KERNEL] a90 gpio config table = EVB\n"); 
		   
		msm_gpiomux_install(a90_evb_msm8974_gpio_configs,
             ARRAY_SIZE(a90_evb_msm8974_gpio_configs));	 	  
             break;

		case A91_SR1:
		case A91_SR2:
		printk("[KERNEL] a91 gpio config table = SR1\n");

		msm_gpiomux_install(a91_sr1_msm8974_gpio_configs,
		ARRAY_SIZE(a91_sr1_msm8974_gpio_configs));
	     break;

		case A91_SR3:
		printk("[KERNEL] a91 gpio config table = SR3\n");

		msm_gpiomux_install(a91_sr3_msm8974_gpio_configs,
		ARRAY_SIZE(a91_sr3_msm8974_gpio_configs));
	     break;

		case A91_SR4:
		printk("[KERNEL] a91 gpio config table = SR4\n");

		msm_gpiomux_install(a91_sr4_msm8974_gpio_configs,
		ARRAY_SIZE(a91_sr4_msm8974_gpio_configs));
	     break;

		case A91_SR5:
		printk("[KERNEL] a91 gpio config table = SR5\n");

		msm_gpiomux_install(a91_sr5_msm8974_gpio_configs,
		ARRAY_SIZE(a91_sr5_msm8974_gpio_configs));
	     break;

		case A91_ER1:
		case A91_ER2:
		case A91_PR:
		case A91_MP:
		printk("[KERNEL] a91 gpio config table = ER1\n");
		msm_gpiomux_install(a91_er1_msm8974_gpio_configs,
		ARRAY_SIZE(a91_er1_msm8974_gpio_configs));

	     break;

	     default:
		printk(KERN_ERR "[ERROR] There is NO valid hardware ID\n");			   
		msm_gpiomux_install(a91_er1_msm8974_gpio_configs,
		ARRAY_SIZE(a91_er1_msm8974_gpio_configs));
             break;
   }
#endif
   
#ifdef ASUS_PF500KL_PROJECT
	// +++ ASUS_BSP : add for pf500kl
   	switch (g_ASUS_hwID)
   	{
		case A90_EVB0:
		printk("[KERNEL] a90 gpio config table = EVB0\n");  
	   
             msm_gpiomux_install(a90_evb0_msm8974_gpio_configs,
             ARRAY_SIZE(a90_evb0_msm8974_gpio_configs));	 	  

             break;
			   
         	case A90_EVB:
         	case A90_SR1:	
         	case A90_SR2:		
         	case A90_SR3:	
         	case A90_ER1:
         	case A90_PR:		
         	case A90_MP:						
             printk("[KERNEL] a90 gpio config table = EVB\n"); 
		   
		msm_gpiomux_install(a90_evb_msm8974_gpio_configs,
             ARRAY_SIZE(a90_evb_msm8974_gpio_configs));	 	  
             break;

		case A91_SR1:
		case A91_SR2:
		printk("[KERNEL] a91 gpio config table = SR1\n");

		msm_gpiomux_install(a91_sr1_msm8974_gpio_configs,
		ARRAY_SIZE(a91_sr1_msm8974_gpio_configs));
	     break;

		case A91_SR3:
		printk("[KERNEL] a91 gpio config table = SR3\n");

		msm_gpiomux_install(a91_sr3_msm8974_gpio_configs,
		ARRAY_SIZE(a91_sr3_msm8974_gpio_configs));
	     break;

		case A91_SR4:
		printk("[KERNEL] a91 gpio config table = SR4\n");

		msm_gpiomux_install(a91_sr4_msm8974_gpio_configs,
		ARRAY_SIZE(a91_sr4_msm8974_gpio_configs));
	     break;

		case A91_SR5:
		printk("[KERNEL] a91 gpio config table = SR5\n");

		msm_gpiomux_install(a91_sr5_msm8974_gpio_configs,
		ARRAY_SIZE(a91_sr5_msm8974_gpio_configs));
	     break;

		case A91_ER1:
		case A91_ER2:
		case A91_PR:
		case A91_MP:
		printk("[KERNEL] a91 gpio config table = ER1\n");
		msm_gpiomux_install(a91_er1_msm8974_gpio_configs,
		ARRAY_SIZE(a91_er1_msm8974_gpio_configs));

	     break;
             case PF500KL_ER1:
	   	case PF500KL_ER2:
		printk("[KERNEL] pf500kl gpio config table = ER1\n"); 
		   
		msm_gpiomux_install(pf500kl_er1_msm8974_gpio_configs,
		ARRAY_SIZE(pf500kl_er1_msm8974_gpio_configs));	 	  
             break;			
		
		case PF500KL_ER2_2:	  
		printk("[KERNEL] pf500kl gpio config table = ER2_2\n"); 
		   
		msm_gpiomux_install(pf500kl_er2_2_msm8974_gpio_configs,
		ARRAY_SIZE(pf500kl_er2_2_msm8974_gpio_configs));
		
	     break;	

		case PF500KL_PR:
		case PF500KL_MP:	 
		printk("[KERNEL] pf500kl gpio config table = PR\n"); 
		   
		msm_gpiomux_install(pf500kl_pr_msm8974_gpio_configs,
		ARRAY_SIZE(pf500kl_pr_msm8974_gpio_configs));
		
	     break;
		 
	     default:
		printk(KERN_ERR "[ERROR] There is NO valid hardware ID\n");			   
		msm_gpiomux_install(pf500kl_er1_msm8974_gpio_configs,
		ARRAY_SIZE(pf500kl_er1_msm8974_gpio_configs));
             break;
  	}
	// --- ASUS_BSP : add for pf500kl
#endif    
   return 0;
}
