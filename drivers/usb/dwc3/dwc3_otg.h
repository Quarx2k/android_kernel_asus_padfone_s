/**
 * dwc3_otg.h - DesignWare USB3 DRD Controller OTG
 *
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#ifndef __LINUX_USB_DWC3_OTG_H
#define __LINUX_USB_DWC3_OTG_H

#include <linux/workqueue.h>
#include <linux/power_supply.h>

#include <linux/usb/otg.h>
#include "power.h"

#define DWC3_IDEV_CHG_MAX 1500

//ASUS_BSP+++ BennyCheng "add host/client mode switch support"
#include <linux/microp_notify.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include <linux/microp.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>

enum dwc3_usb_mode_type {
	DWC3_USB_NONE = 0,
	DWC3_USB_PERIPHERAL,
	DWC3_USB_HOST,
	DWC3_USB_OTG,
	DWC3_USB_AUTO,
};

extern int dwc3_pm_count;
extern bool pad_exist(void);
extern void asus_dwc3_vbus_out_enable(bool enable, bool force);
//ASUS_BSP--- BennyCheng "add host/client mode switch support"

//ASUS_BSP+++ BennyCheng "add usb mydp switch support"
#define GPIO_USB_SW_SEL_EVB0 65
#define GPIO_USB_SW_SEL_EVB 73
#define GPIO_USB_SW_SEL_SR1 65

enum usb_mydp_sw {
	MYDP_PORT = 0,
	USB_PORT,
};
//ASUS_BSP--- BennyCheng "add usb mydp switch support"

//ASUS_BSP+++ BennyCheng "add microp related debug files"
enum microp_mode_sw {
	MICROP_SLEEP = 0,
	MICROP_ACTIVE,
};

enum microp_host_sw {
	MICROP_USB_PATH_HOST = 0,
	MICROP_USB_PATH_CLIENT,
};
//ASUS_BSP--- BennyCheng "add microp related debug files"

//ASUS_BSP+++ BennyCheng "add phone mode usb OTG support"
#ifdef CONFIG_ASUS_CARKIT
#include <linux/switch.h>

enum asus_otg_state {
	ASUS_OTG_NONE,
	ASUS_OTG_DISCONNECT,
	ASUS_OTG_CONNECT,
	ASUS_OTG_CARKIT,
	ASUS_OTG_HOST,
};

#define ASUS_CARKIT_OFFLINE 0
#define ASUS_CARKIT_ONLINE 2 //android native defined. Desk:1, CAR:2...

extern int asus_state_otg; //for ID low switch control
extern int asus_state_carkit; //carkit state
extern struct switch_dev asus_switch_otg_carkit;

extern void asus_dwc3_mode_switch(enum dwc3_usb_mode_type req_mode);
extern int dp_registerCarkitInOutNotificaition(void (*callback)(int));
#else
extern int dp_registerCarkitInOutNotificaition(void (*callback)(int));
#endif
//ASUS_BSP--- BennyCheng "add phone mode usb OTG support"

//ASUS_BSP+++ BennyCheng "register microp event for pad mode switch"
extern void asus_dwc3_host_mode_prepare(void);
extern void asus_dwc3_host_mode_cleanup(void);
//ASUS_BSP--- BennyCheng "register microp event for pad mode switch"

//ASUS_BSP+++ BennyCheng "support dynamic hsphy parameter_override_x setting"
#define PARAMETER_OVERRIDE_X_DEBUG               0x0
#define PARAMETER_OVERRIDE_X_PHONE               0xd190e4
//ASUS_BSP--- BennyCheng "support dynamic hsphy parameter_override_x setting"

//ASUS_BSP+++ "[USB][NA][Spec] add ASUS Charger support"
#ifdef CONFIG_CHARGER_ASUS
#include <linux/asus_chg.h>
#else
enum asus_chg_src {
	ASUS_CHG_SRC_NONE = 0,
	ASUS_CHG_SRC_USB,
	ASUS_CHG_SRC_DC,
	ASUS_CHG_SRC_UNKNOWN,
};
#endif
//ASUS_BSP--- "[USB][NA][Spec] add ASUS Charger support"

struct dwc3_charger;

/**
 * struct dwc3_otg: OTG driver data. Shared by HCD and DCD.
 * @otg: USB OTG Transceiver structure.
 * @irq: IRQ number assigned for HSUSB controller.
 * @regs: ioremapped register base address.
 * @sm_work: OTG state machine work.
 * @charger: DWC3 external charger detector
 * @inputs: OTG state machine inputs
 */
struct dwc3_otg {
	struct usb_otg		otg;
	int			irq;
	struct dwc3		*dwc;
	void __iomem		*regs;
	struct regulator	*vbus_otg;
	struct delayed_work	sm_work;
	struct dwc3_charger	*charger;
	struct dwc3_ext_xceiv	*ext_xceiv;
#define ID		0
#define B_SESS_VLD	1
	unsigned long inputs;
	struct power_supply	*psy;
	struct completion	dwc3_xcvr_vbus_init;
	int			host_bus_suspend;
	int			charger_retry_count;
	int			vbus_retry_count;
};

/**
 * USB charger types
 *
 * DWC3_INVALID_CHARGER	Invalid USB charger.
 * DWC3_SDP_CHARGER	Standard downstream port. Refers to a downstream port
 *                      on USB compliant host/hub.
 * DWC3_DCP_CHARGER	Dedicated charger port (AC charger/ Wall charger).
 * DWC3_CDP_CHARGER	Charging downstream port. Enumeration can happen and
 *                      IDEV_CHG_MAX can be drawn irrespective of USB state.
 * DWC3_PROPRIETARY_CHARGER A proprietary charger pull DP and DM to specific
 *                     voltages between 2.0-3.3v for identification.
 * DWC3_FLOATED_CHARGER Non standard charger whose data lines are floating.
 */
enum dwc3_chg_type {
	DWC3_INVALID_CHARGER = 0,
	DWC3_SDP_CHARGER,
	DWC3_DCP_CHARGER,
	DWC3_CDP_CHARGER,
	DWC3_PROPRIETARY_CHARGER,
	DWC3_FLOATED_CHARGER,
};

struct dwc3_charger {
	enum dwc3_chg_type	chg_type;
	unsigned		max_power;
	bool			charging_disabled;

	bool			skip_chg_detect;

	/* start/stop charger detection, provided by external charger module */
	void	(*start_detection)(struct dwc3_charger *charger, bool start);

	/* to notify OTG about charger detection completion, provided by OTG */
	void	(*notify_detection_complete)(struct usb_otg *otg,
						struct dwc3_charger *charger);
};

/* for external charger driver */
extern int dwc3_set_charger(struct usb_otg *otg, struct dwc3_charger *charger);

enum dwc3_ext_events {
	DWC3_EVENT_NONE = 0,		/* no change event */
	DWC3_EVENT_PHY_RESUME,		/* PHY has come out of LPM */
	DWC3_EVENT_XCEIV_STATE,		/* XCEIV state (id/bsv) has changed */
};

enum dwc3_id_state {
	DWC3_ID_GROUND = 0,
	DWC3_ID_FLOAT,
};

/* external transceiver that can perform connect/disconnect monitoring in LPM */
struct dwc3_ext_xceiv {
	enum dwc3_id_state	id;
	bool			bsv;
	bool			otg_capability;
	//ASUS_BSP+++ BennyCheng "add host/client mode switch support"
	enum dwc3_usb_mode_type otg_mode;
	bool host_mode;
	//ASUS_BSP--- BennyCheng "add host/client mode switch support"
	//ASUS_BSP+++ BennyCheng "add phone mode usb OTG support"
	bool vbus_state;
	bool id_state;
	//ASUS_BSP--- BennyCheng "add phone mode usb OTG support"

	/* to notify OTG about LPM exit event, provided by OTG */
	void	(*notify_ext_events)(struct usb_otg *otg,
					enum dwc3_ext_events ext_event);
	/* for block reset USB core */
	void	(*ext_block_reset)(struct dwc3_ext_xceiv *ext_xceiv,
					bool core_reset);
};

/* for external transceiver driver */
extern int dwc3_set_ext_xceiv(struct usb_otg *otg,
				struct dwc3_ext_xceiv *ext_xceiv);

#endif /* __LINUX_USB_DWC3_OTG_H */
