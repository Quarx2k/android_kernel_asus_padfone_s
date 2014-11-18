#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/microp_notify.h>






/*
* notification part
* each driver have to use register_hs_notifier() to register a callback-function
* if the driver want to be notified while hall sensor status is changing
*
* register_microp_notifier(): drivers can use the func to register a callback
* unregister_microp_notifier(): drivers can use the func to unregister a callback
*/

static BLOCKING_NOTIFIER_HEAD(microp_chain_head);



int micropSendNotify(unsigned long val)
{ 
	return 0;
}
EXPORT_SYMBOL_GPL(micropSendNotify);




int register_microp_notifier(struct notifier_block *nb)
{	
	return blocking_notifier_chain_register(&microp_chain_head, nb);
}

EXPORT_SYMBOL_GPL(register_microp_notifier);

int unregister_microp_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&microp_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_microp_notifier);





