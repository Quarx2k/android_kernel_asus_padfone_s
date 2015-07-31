#ifndef __PROXIMITYBASIC_H__
#define __PROXIMITYBASIC_H__
#include <linux/types.h>

#define MAX_POXIMITY_SENSOR_SIZE (BITS_PER_LONG)

typedef enum { 
    PROXIMITY_EVNET_FAR = 0,
    PROXIMITY_EVNET_NEAR,
    PROXIMITY_EVNET_MAX,
} PROXIMITY_EVENT;

typedef enum { 
    PROXIMITY_FILE_SOURCE = 0,
    PROXIMITY_CAP1106_SOURCE,
    PROXIMITY_SOURCE_MAX,
} PROXIMITY_SOURCE_TYPE;

typedef enum { 
    PROXIMITY_DEBUG_ALGO_TYPE = 0,
    PROXIMITY_BODYSAR_NOTIFY_ALGO_TYPE,
    PROXIMITY_ALGO_TYPE_MAX,
} PROXIMITY_ALGO_TYPE;


typedef struct proximity_resource {
    unsigned int index; //max support size is BITS_PER_LONG
    PROXIMITY_SOURCE_TYPE type;
    PROXIMITY_ALGO_TYPE algo_type;
    PROXIMITY_EVENT initEventState;
    const char * name;
    unsigned int debounceInterval;
    unsigned int pollingInterval;
    unsigned int irq;
    int wakeup;
}proximity_resource;

typedef struct proximity_platform_data {
    struct proximity_resource *resource;
    int nResource;
}proximity_platform_data;

struct ProximityEventHandler;
typedef struct ProximityEventHandler{
    int eventStatus;
    void (*onEvent)(struct ProximityEventHandler *handler, const char *client_name, int event);
    int (*readEvent)(struct ProximityEventHandler *handler, char *client_name);
}ProximityEventHandler;

ProximityEventHandler *setProximityEventHandler(const char *name);

#endif //__PROXIMITYBASIC_H__
