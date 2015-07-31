#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <linux/ProximityBasic.h>

#define LOGPREFIX "[ProximityFileSource]" 
#define FILENAME_PREFIX "driver/%s"
#define MAX_SIZE_OF_PATH_NAME (50)
#if 0
struct File_Proc_Status;
typedef struct File_Proc_Status{
    const char *name;
    int event_status;
    ProximityEventHandler *handler;
}File_Proc_Status;

static ssize_t read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
    File_Proc_Status *File_Status = (File_Proc_Status *)data;
    return sprintf(page, "%d\n", File_Status->event_status);
}
static ssize_t write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
    File_Proc_Status *File_Status = (File_Proc_Status *)data;
    int val;

    char messages[256];

    if (len > 256) {
        len = 256;
    }

    if (copy_from_user(messages, buff, len)) {
        return -EFAULT;
    }

    val = (int)simple_strtol(messages, NULL, 10);

    if(val >= PROXIMITY_EVNET_MAX){
        printk(KERN_ERR"%s:Wrong paramenter %d\n",LOGPREFIX, val);
        return -EFAULT;
    }

    if(File_Status->event_status != val){
        File_Status->event_status = val;
//printk("!!! %s onEvent +++ !!!\n\n", __FUNCTION__);
        File_Status->handler->onEvent(File_Status->handler, File_Status->name, File_Status->event_status);
//printk("!!! %s onEvent --- !!!\n\n", __FUNCTION__);
    }

    return len;
}
#endif
void create_ProximityTest_proc_file(
    const char *name, ProximityEventHandler *handler)
{
#if 0
    File_Proc_Status *file_status = NULL;
    char buf[MAX_SIZE_OF_PATH_NAME];

    struct proc_dir_entry *proc_file = NULL;

    snprintf(buf, 
        MAX_SIZE_OF_PATH_NAME,
        FILENAME_PREFIX, 
        name);
    
    proc_file = create_proc_entry(buf, 0644, NULL);

    file_status =  kzalloc(sizeof(File_Proc_Status), GFP_KERNEL);
    if (!file_status) {
        printk("%s file_status kzalloc fail!\n", __FUNCTION__);
        return;
    }
    file_status->handler = handler;
    file_status->name = name;
    file_status->event_status = 0;
    file_status->handler->onEvent(file_status->handler, file_status->name, file_status->event_status);

    if (proc_file) {
        
        proc_file->read_proc =  read_proc;
        
        proc_file->write_proc = write_proc;

        proc_file->data = (void *)file_status;
 
    }else {
        
        printk(KERN_ERR"%s:Can't create proc file\n",LOGPREFIX);
    }
#endif
    return;
}
