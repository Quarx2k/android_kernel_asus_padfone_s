#ifndef MICROP_NOTIFIER_CONTROLLER_H
#define MICROP_NOTIFIER_CONTROLLER_H

extern int notify_register_microp_notifier(struct notifier_block *nb, char* driver_name);
extern int notify_unregister_microp_notifier(struct notifier_block *nb, char* driver_name);

#endif
