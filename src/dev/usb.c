#include <nautilus/nautilus.h>
#include <nautilus/mm.h>
#include <nautilus/list.h>
#include <nautilus/naut_string.h>
#include <dev/usb.h>


#define INFO(fmt, args...)   INFO_PRINT("usb: " fmt, ##args)
#define DEBUG(fmt, args...)  DEBUG_PRINT("usb: " fmt, ##args)
#define ERROR(fmt, args...)  ERROR_PRINT("usb: " fmt, ##args)


// Global list of registered USB devices. Lazy-initialized on first
// register; xHCI is the only producer today.
static struct list_head usb_devices;
static int usb_devices_inited = 0;

static void usb_ensure_inited(void) {
    if (!usb_devices_inited) {
        INIT_LIST_HEAD(&usb_devices);
        usb_devices_inited = 1;
    }
}


struct usb_device *usb_alloc_device(void) {
    struct usb_device *dev = malloc(sizeof(*dev));
    if (!dev) {
        ERROR("cannot allocate usb_device\n");
        return NULL;
    }
    memset(dev, 0, sizeof(*dev));
    INIT_LIST_HEAD(&dev->node);
    return dev;
}

void usb_free_device(struct usb_device *dev) {
    if (!dev) return;
    if (!list_empty(&dev->node)) {
        ERROR("freeing device still on registry (slot %u)\n", dev->slot_id);
        list_del_init(&dev->node);
    }
    free(dev);
}


int usb_register_device(struct usb_device *dev) {
    if (!dev) return -1;
    usb_ensure_inited();
    list_add_tail(&dev->node, &usb_devices);
    INFO("registered slot %u addr %u: vendor=0x%04x product=0x%04x "
         "class=%u sub=%u proto=%u speed=%u port=%u\n",
         dev->slot_id, dev->address,
         dev->vendor_id, dev->product_id,
         dev->dev_class, dev->dev_subclass, dev->dev_protocol,
         dev->speed, dev->port);
    return 0;
}

void usb_unregister_device(struct usb_device *dev) {
    if (!dev) return;
    if (!list_empty(&dev->node)) {
        list_del_init(&dev->node);
    }
}


int usb_for_each_device(int (*cb)(struct usb_device *, void *), void *arg) {
    if (!cb) return 0;
    usb_ensure_inited();
    struct list_head *cur, *next;
    int rc = 0;
    list_for_each_safe(cur, next, &usb_devices) {
        struct usb_device *dev = list_entry(cur, struct usb_device, node);
        rc = cb(dev, arg);
        if (rc) break;
    }
    return rc;
}

static const char *speed_name(uint8_t speed) {
    switch (speed) {
    case USB_SPEED_LS:  return "LS";
    case USB_SPEED_FS:  return "FS";
    case USB_SPEED_HS:  return "HS";
    case USB_SPEED_SS:  return "SS";
    case USB_SPEED_SSP: return "SSP";
    default:            return "?";
    }
}

void usb_dump_devices(void) {
    usb_ensure_inited();
    if (list_empty(&usb_devices)) {
        INFO("no devices registered\n");
        return;
    }
    struct list_head *cur;
    list_for_each(cur, &usb_devices) {
        struct usb_device *dev = list_entry(cur, struct usb_device, node);
        INFO("  slot %u addr %u %-3s port %u  %04x:%04x  class=%u/%u/%u  configs=%u\n",
             dev->slot_id, dev->address, speed_name(dev->speed), dev->port,
             dev->vendor_id, dev->product_id,
             dev->dev_class, dev->dev_subclass, dev->dev_protocol,
             dev->num_configs);
    }
}
