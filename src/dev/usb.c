#include <nautilus/nautilus.h>
#include <nautilus/mm.h>
#include <nautilus/list.h>
#include <nautilus/naut_string.h>
#include <dev/usb.h>
#include <dev/xhci.h>


#define INFO(fmt, args...)   INFO_PRINT("usb: " fmt, ##args)
#define DEBUG(fmt, args...)  DEBUG_PRINT("usb: " fmt, ##args)
#define ERROR(fmt, args...)  ERROR_PRINT("usb: " fmt, ##args)


// Global list of registered USB devices. Lazy-initialized on first register
static struct list_head usb_devices;
static int usb_devices_inited = 0;

// Class driver table
#define USB_MAX_DRIVERS 16
static const struct usb_driver *usb_drivers[USB_MAX_DRIVERS];
static uint32_t usb_drivers_n = 0;

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


// Does drv match dev's interface (or fallback to device-level) triple?
static int usb_driver_matches(const struct usb_driver *drv,
                              const struct usb_device *dev) {
    uint8_t cls = dev->iface_class;
    uint8_t sub = dev->iface_subclass;
    uint8_t pro = dev->iface_protocol;
    if (drv->match_class    != USB_ANY_CLASS    && drv->match_class    != cls) goto try_dev;
    if (drv->match_subclass != USB_ANY_SUBCLASS && drv->match_subclass != sub) goto try_dev;
    if (drv->match_protocol != USB_ANY_PROTOCOL && drv->match_protocol != pro) goto try_dev;
    return 1;
try_dev:
    if (dev->dev_class == 0) return 0;   // no device-level fallback to try
    cls = dev->dev_class;
    sub = dev->dev_subclass;
    pro = dev->dev_protocol;
    if (drv->match_class    != USB_ANY_CLASS    && drv->match_class    != cls) return 0;
    if (drv->match_subclass != USB_ANY_SUBCLASS && drv->match_subclass != sub) return 0;
    if (drv->match_protocol != USB_ANY_PROTOCOL && drv->match_protocol != pro) return 0;
    return 1;
}

// Walk the driver table and probe the first match. Stops at the first
// driver whose probe returns 0 (successfully bound).
static void usb_probe_drivers(struct usb_device *dev) {
    for (uint32_t i = 0; i < usb_drivers_n; i++) {
        const struct usb_driver *drv = usb_drivers[i];
        if (!usb_driver_matches(drv, dev)) continue;
        DEBUG("probing driver '%s' for slot %u\n", drv->name, dev->slot_id);
        int rc = drv->probe(dev);
        if (rc == 0) {
            dev->driver = drv;
            INFO("bound driver '%s' to slot %u (addr %u)\n",
                 drv->name, dev->slot_id, dev->address);
            return;
        }
        DEBUG("driver '%s' declined slot %u (rc=%d)\n", drv->name, dev->slot_id, rc);
    }
    DEBUG("no driver bound to slot %u\n", dev->slot_id);
}


int usb_driver_register(const struct usb_driver *drv) {
    if (!drv || !drv->probe || !drv->name) {
        ERROR("invalid driver registration\n");
        return -1;
    }
    if (usb_drivers_n >= USB_MAX_DRIVERS) {
        ERROR("driver table full; cannot register '%s'\n", drv->name);
        return -1;
    }
    usb_drivers[usb_drivers_n++] = drv;
    INFO("registered driver '%s' (class=%u sub=%u proto=%u)\n",
         drv->name, drv->match_class, drv->match_subclass, drv->match_protocol);
    return 0;
}


int usb_register_device(struct usb_device *dev) {
    if (!dev) return -1;
    usb_ensure_inited();
    list_add_tail(&dev->node, &usb_devices);
    INFO("registered slot %u addr %u: vendor=0x%04x product=0x%04x "
         "iface_class=%u/%u/%u dev_class=%u speed=%u port=%u\n",
         dev->slot_id, dev->address,
         dev->vendor_id, dev->product_id,
         dev->iface_class, dev->iface_subclass, dev->iface_protocol,
         dev->dev_class,
         dev->speed, dev->port);
    usb_probe_drivers(dev);
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


//
// transfer interface
//

int usb_control_transfer(struct usb_device *dev, uint8_t request_type, uint8_t request, uint16_t value, uint16_t index, void *data, uint16_t length) {
    if (!dev || !dev->hc) {
        ERROR("control xfer: NULL device or hc\n");
        return -1;
    }
    // from the caller's perspective, the device carries everything needed to talk to it
    return xhci_control_transfer(dev->hc, dev->slot_id, request_type, request, value, index, data, length);
}

struct usb_endpoint *usb_find_endpoint(struct usb_device *dev,
                                       uint8_t ep_num, int dir_in) {
    if (!dev) return NULL;
    for (uint32_t i = 0; i < dev->num_endpoints; i++) {
        struct usb_endpoint *e = &dev->endpoints[i];
        if (!e->active) continue;
        if (USB_EP_NUM(e->address) == ep_num &&
            USB_EP_DIR_IN(e->address) == (dir_in != 0)) {
            return e;
        }
    }
    return NULL;
}

// Bulk and interrupt both queue NORMAL TRBs on a non-EP0 transfer ring;
// the EP_TYPE in the controller's EP context (set up by CONFIGURE_ENDPOINT)
// is what makes the wire behavior differ. Same dispatch logic for both.
static int usb_normal_transfer(struct usb_device *dev, uint8_t ep,
                               void *data, size_t length, int dir,
                               uint8_t expected_xfer_type, const char *what) {
    if (!dev || !dev->hc) {
        ERROR("%s xfer: NULL device or hc\n", what);
        return -1;
    }
    if (length > 0xffff) {
        ERROR("%s xfer slot=%u ep=%u: len %lu exceeds single-TRB max\n",
              what, dev->slot_id, ep, (unsigned long)length);
        return -1;
    }
    int dir_in = (dir & USB_DIR_IN) != 0;
    struct usb_endpoint *e = usb_find_endpoint(dev, ep, dir_in);
    if (!e) {
        ERROR("%s xfer slot=%u ep=%u dir=%s: endpoint not found\n",
              what, dev->slot_id, ep, dir_in ? "IN" : "OUT");
        return -1;
    }
    if ((e->attributes & USB_EP_XFER_MASK) != expected_xfer_type) {
        ERROR("%s xfer slot=%u ep=%u: wrong endpoint type (got %u, want %u)\n",
              what, dev->slot_id, ep, e->attributes & USB_EP_XFER_MASK,
              expected_xfer_type);
        return -1;
    }
    return xhci_normal_transfer(dev->hc, dev->slot_id, e->dci,
                                data, (uint16_t)length);
}

int usb_bulk_transfer(struct usb_device *dev, uint8_t ep,
                      void *data, size_t length, int dir) {
    return usb_normal_transfer(dev, ep, data, length, dir,
                               USB_EP_XFER_BULK, "bulk");
}

int usb_interrupt_transfer(struct usb_device *dev, uint8_t ep,
                           void *data, size_t length, int dir) {
    return usb_normal_transfer(dev, ep, data, length, dir,
                               USB_EP_XFER_INTR, "intr");
}

// dispatches to xhci_isoch_transfer
int usb_isoch_transfer(struct usb_device *dev, uint8_t ep, void *data, size_t length, int dir) {
    if (!dev || !dev->hc) {
        ERROR("isoch xfer: NULL device or hc\n");
        return -1;
    }
    if (length > 0xffff) {
        ERROR("isoch xfer slot=%u ep=%u: len %lu exceeds single-TRB max\n",
              dev->slot_id, ep, (unsigned long)length);
        return -1;
    }
    int dir_in = (dir & USB_DIR_IN) != 0;
    struct usb_endpoint *e = usb_find_endpoint(dev, ep, dir_in);
    if (!e) {
        ERROR("isoch xfer slot=%u ep=%u dir=%s: endpoint not found\n",
              dev->slot_id, ep, dir_in ? "IN" : "OUT");
        return -1;
    }
    if ((e->attributes & USB_EP_XFER_MASK) != USB_EP_XFER_ISOCH) {
        ERROR("isoch xfer slot=%u ep=%u: wrong endpoint type (got %u, want %u)\n",
              dev->slot_id, ep, e->attributes & USB_EP_XFER_MASK,
              USB_EP_XFER_ISOCH);
        return -1;
    }
    return xhci_isoch_transfer(dev->hc, dev->slot_id, e->dci, data, (uint16_t)length);
}

int usb_get_descriptor(struct usb_device *dev, uint8_t dt_type, uint8_t dt_index, void *buf, uint16_t length) {
    // Standard "device-to-host, standard, device" descriptor read.
    // wValue's high byte is the descriptor type, low byte is the index.
    return usb_control_transfer(dev, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, (uint16_t)(dt_type << 8) | dt_index, 0, buf, length);
}
