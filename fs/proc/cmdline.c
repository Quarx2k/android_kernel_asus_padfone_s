#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", saved_command_line);
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

//+++ ASUS_BSP : Add for parse cmdline info to proc/bootinfo
static int boot_cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", bootimage_command_line);
	return 0;
}

static int boot_cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, boot_cmdline_proc_show, NULL);
}

static const struct file_operations boot_cmdline_proc_fops = {
	.open		= boot_cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
//--- ASUS_BSP : Add for parse cmdline info to proc/bootinfo

static int __init proc_cmdline_init(void)
{
	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	proc_create("bootinfo", 0, NULL, &boot_cmdline_proc_fops);	//+++ ASUS_BSP : Add for parse cmdline info to proc/bootinfo
	return 0;
}
module_init(proc_cmdline_init);
