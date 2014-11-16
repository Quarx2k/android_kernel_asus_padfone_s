/* interrupt.h */
#ifndef _ASUS_FASTBOOT_H
#define _ASUS_FASTBOOT_H

#include <linux/types.h>
#include <linux/delay.h>

#define TIME_FOR_VIBRATE_IN_FASTBOOT_MODE (300)
#define GPIO_POWERKEY (15)
#define DELAY_FOR_RECHECKOUT_POWERKEY_STATUS (1000)
#define TIMES_FOR_RECHECKOUT_POWERKEY_STATUS (3) 
#define TIME_FOR_POWERKEY_LONGPRESS (DELAY_FOR_RECHECKOUT_POWERKEY_STATUS * TIMES_FOR_RECHECKOUT_POWERKEY_STATUS)// total 3 secs

extern bool is_fastboot_enable(void);
extern void ready_to_wake_up_and_send_power_key_press_event_in_fastboot(bool isNeedToSendkey);
extern bool is_power_key_pressed(void);
extern int is_cable_plugged_in(void);
extern void kernel_power_off(void);
extern bool is_batt_low(void);

#ifdef CONFIG_KEYBOARD_GPIO
extern void send_fake_power_key_event(bool keyPressed);
#else
#define send_fake_power_key_event 
#endif
#endif

