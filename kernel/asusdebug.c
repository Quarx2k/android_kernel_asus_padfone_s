/* //20100930 jack_wong for asus debug mechanisms +++++
 *  asusdebug.c
 * //20100930 jack_wong for asus debug mechanisms -----
 *
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/rtc.h>
#include <linux/list.h>
//#include <linux/asus_ver.h>
#include <linux/syscalls.h>
#include <linux/earlysuspend.h> 
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/export.h>
#include <linux/rtc.h>
#include "rtmutex_common.h"


void ASUSEvtlog(const char *fmt, ...) {}
void save_phone_hang_log() {}
void delta_all_thread_info() {}
void save_all_thread_info() {}

