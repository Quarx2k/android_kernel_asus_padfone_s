/*
 * kernel/power/autosleep.c
 *
 * Opportunistic sleep support.
 *
 * Copyright (C) 2012 Rafael J. Wysocki <rjw@sisk.pl>
 */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/module.h>

#include "power.h"

#define WAIT_FOR_PRINT_WAKEUP_SOURCE	1000 * 10 // 10 secs
void check_wakeup_source_timer_expired(unsigned long);
extern void print_wakeup_sources(void);
DEFINE_TIMER(check_wakeup_source_timer, check_wakeup_source_timer_expired, 0, 0);

static suspend_state_t autosleep_state;
static struct workqueue_struct *autosleep_wq;
static struct switch_dev pmsp_dev; //austin++
struct work_struct pms_printer;

static struct switch_dev pm_wakeup_source_dev;
struct work_struct pm_wakeup_source_printer;
unsigned long print_period = WAIT_FOR_PRINT_WAKEUP_SOURCE;

void pmsp_print(void);
extern bool g_resume_status; //austin+++
/*
 * Note: it is only safe to mutex_lock(&autosleep_lock) if a wakeup_source
 * is active, otherwise a deadlock with try_to_suspend() is possible.
 * Alternatively mutex_lock_interruptible() can be used.  This will then fail
 * if an auto_sleep cycle tries to freeze processes.
 */
static DEFINE_MUTEX(autosleep_lock);
static struct wakeup_source *autosleep_ws;

void check_wakeup_source_timer_expired(unsigned long data)
{
	print_wakeup_sources();

	if (!timer_pending(&check_wakeup_source_timer)){
		printk("[PM]check_wakeup_source_timer_expired, next printer will trigger in %ld seconds\n", print_period / 1000);
		mod_timer(&check_wakeup_source_timer, jiffies + msecs_to_jiffies(print_period));
		if(print_period * 2 < 1000 * 60 * 60 )
			print_period *= 2;
		else
			print_period = 1000 * 60 * 60;
				
	}else{
		printk("[PM]Another check_wakeup_source_timer is running\n");
	}
}

static void try_to_suspend(struct work_struct *work)
{
	unsigned int initial_count, final_count;

	if (!pm_get_wakeup_count(&initial_count, true))
		goto out;
    
	mutex_lock(&autosleep_lock);

	if (!pm_save_wakeup_count(initial_count)) {
		mutex_unlock(&autosleep_lock);
		goto out;
	}

	if (autosleep_state == PM_SUSPEND_ON) {
		mutex_unlock(&autosleep_lock);
		return;
	}
	if (autosleep_state >= PM_SUSPEND_MAX)
		hibernate();
	else
		pm_suspend(autosleep_state);

	mutex_unlock(&autosleep_lock);

	if (!pm_get_wakeup_count(&final_count, false))
		goto out;

	/*
	 * If the wakeup occured for an unknown reason, wait to prevent the
	 * system from trying to suspend and waking up in a tight loop.
	 */
	if (final_count == initial_count)
		schedule_timeout_uninterruptible(HZ / 2);

 out:
	queue_up_suspend_work();
}

static DECLARE_WORK(suspend_work, try_to_suspend);

void queue_up_suspend_work(void)
{
	if (!work_pending(&suspend_work) && autosleep_state > PM_SUSPEND_ON)
		queue_work(autosleep_wq, &suspend_work);
}

suspend_state_t pm_autosleep_state(void)
{
	return autosleep_state;
}

int pm_autosleep_lock(void)
{
	return mutex_lock_interruptible(&autosleep_lock);
}

void pm_autosleep_unlock(void)
{
	mutex_unlock(&autosleep_lock);
}

//Add a timer to trigger wakelock debug
extern struct timer_list unattended_timer;

int pm_autosleep_set_state(suspend_state_t state)
{

#ifndef CONFIG_HIBERNATION
	if (state >= PM_SUSPEND_MAX)
		return -EINVAL;
#endif

	__pm_stay_awake(autosleep_ws);

	mutex_lock(&autosleep_lock);

	autosleep_state = state;

	__pm_relax(autosleep_ws);

	if (state > PM_SUSPEND_ON) {
		pm_wakep_autosleep_enabled(true);
        
        g_resume_status = false; //austin+++
		//Add a timer to trigger wakelock debug
        pr_info("[PM]unattended_timer: mod_timer (auto_sleep)\n");
        mod_timer(&unattended_timer, jiffies + msecs_to_jiffies(PM_UNATTENDED_TIMEOUT));

		if (!timer_pending(&check_wakeup_source_timer))
			mod_timer(&check_wakeup_source_timer, jiffies + msecs_to_jiffies(WAIT_FOR_PRINT_WAKEUP_SOURCE));

		queue_up_suspend_work();
	} else {
		pm_wakep_autosleep_enabled(false);
		//Add a timer to trigger wakelock debug
        pr_info("[PM]unattended_timer: del_timer (late_resume)\n");
        del_timer(&unattended_timer);
		if (timer_pending(&check_wakeup_source_timer)){
			del_timer(&check_wakeup_source_timer);
			print_period = WAIT_FOR_PRINT_WAKEUP_SOURCE;
			printk("[PM]Delete pending check_wakeup_source_timer\n");
		}		
	}

	mutex_unlock(&autosleep_lock);
	return 0;
}

void pmsp_print(void){
    schedule_work(&pms_printer);
    return;
}

EXPORT_SYMBOL(pmsp_print);

void print_pm_wakeup_source(void){
    schedule_work(&pm_wakeup_source_printer);
    return;
}
EXPORT_SYMBOL(print_pm_wakeup_source);



void pms_printer_func(struct work_struct *work){

static int pmsp_counter = 0;

if(pmsp_counter % 2){
printk("%s:enter pmsprinter ready to send uevent 0 \n",__func__);
switch_set_state(&pmsp_dev,0);
pmsp_counter++;}
else{
printk("%s:enter pmsprinter ready to send uevent 1 \n",__func__);
switch_set_state(&pmsp_dev,1);
pmsp_counter++;}
}


void pm_wakeup_source_func(struct work_struct *work){

	static bool toggle = false;

	toggle = !toggle;
	printk("%s: Dump PowerManagerService wakelocks, toggle %d\n",__func__, toggle ? 1 : 0);
	switch_set_state(&pm_wakeup_source_dev, toggle);
}


int __init pm_autosleep_init(void)
{

    int ret;
    pmsp_dev.name = "PowerManagerServicePrinter";
    pmsp_dev.index = 0;
    INIT_WORK(&pms_printer, pms_printer_func);
    ret = switch_dev_register(&pmsp_dev);
    if (ret < 0)
        printk("%s:fail to register switch power_manager_printer \n",__func__);
    else
        printk("%s:success to register pmsp switch \n",__func__);


    pm_wakeup_source_dev.name = "PowerManagerWakelocks";
    pm_wakeup_source_dev.index = 0;
    INIT_WORK(&pm_wakeup_source_printer, pm_wakeup_source_func);
    ret = switch_dev_register(&pm_wakeup_source_dev);
    if (ret < 0)
        printk("%s:fail to register switch device pm_wakeup_source_dev\n",__func__);
    else
        printk("%s:success to register switch device pm_wakeup_source_dev\n",__func__);


	autosleep_ws = wakeup_source_register("autosleep");
	if (!autosleep_ws)
		return -ENOMEM;

	autosleep_wq = alloc_ordered_workqueue("autosleep", 0);
	if (autosleep_wq)
		return 0;

	wakeup_source_unregister(autosleep_ws);
	return -ENOMEM;
}
