#ifndef __USB_H__
#define __USB_H__

#include <nautilus/naut_types.h>
#include <nautilus/list.h>

struct xhci_hc;

//
// USB constants
//

#define USB_DIR_IN              0x80
#define USB_DIR_OUT             0x00

// Standard request codes (bRequest)
#define USB_REQ_GET_STATUS         0
#define USB_REQ_CLEAR_FEATURE      1
#define USB_REQ_SET_FEATURE        3
#define USB_REQ_SET_ADDRESS        5
#define USB_REQ_GET_DESCRIPTOR     6
#define USB_REQ_SET_DESCRIPTOR     7
#define USB_REQ_GET_CONFIGURATION  8
#define USB_REQ_SET_CONFIGURATION  9

// Descriptor types (high byte of wValue for GET_DESCRIPTOR)
#define USB_DT_DEVICE              1
#define USB_DT_CONFIGURATION       2
#define USB_DT_STRING              3
#define USB_DT_INTERFACE           4
#define USB_DT_ENDPOINT            5

// Endpoint address / attribute decoders (USB endpoint descriptor)
#define USB_EP_NUM(addr)       ((addr) & 0xf)
#define USB_EP_DIR_IN(addr)    (((addr) & 0x80) != 0)
#define USB_EP_XFER_MASK       0x3
#define USB_EP_XFER_CONTROL    0
#define USB_EP_XFER_ISOCH      1
#define USB_EP_XFER_BULK       2
#define USB_EP_XFER_INTR       3

// USB device speeds
#define USB_SPEED_UNKNOWN      0
#define USB_SPEED_FS           1
#define USB_SPEED_LS           2
#define USB_SPEED_HS           3
#define USB_SPEED_SS           4
#define USB_SPEED_SSP          5

//
// USB descriptors
//

struct usb_device_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} __attribute__((packed));

struct usb_config_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;        // total bytes of this config bundle (header + interfaces + endpoints + class)
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue; // passed to SET_CONFIGURATION
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;           // bus current draw in 2 mA units
} __attribute__((packed));

struct usb_interface_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;       // excludes EP0
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} __attribute__((packed));

struct usb_endpoint_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;    // bit 7 = direction (1=IN), bits 3:0 = ep number
    uint8_t  bmAttributes;        // bits 1:0 = transfer type
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
} __attribute__((packed));

//
// USB device handle
//

struct usb_device {
    uint8_t  slot_id;        // xHCI slot id (1..max_slots)
    uint8_t  address;        // assigned USB bus address
    uint8_t  speed;          // USB_SPEED_*
    uint8_t  port;           // root hub port number (1-based)

    // From device descriptor
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t bcd_usb;
    uint16_t bcd_device;
    uint8_t  dev_class;      // bDeviceClass (often 0 = "see interfaces")
    uint8_t  dev_subclass;
    uint8_t  dev_protocol;
    uint8_t  max_packet0;    // bMaxPacketSize0 (literal byte count, post-decode)
    uint8_t  num_configs;    // bNumConfigurations

    // From first interface descriptor of config 0
    uint8_t  iface_class;
    uint8_t  iface_subclass;
    uint8_t  iface_protocol;
    uint8_t  iface_num_eps;

    // Backing host controller (today always xHCI)
    struct xhci_hc *hc;

    // Linkage on the global device list
    struct list_head node;

    // Bound class driver (NULL = unbound) and its per-device state.
    // The probe sets driver_data; the framework only writes `driver`.
    const struct usb_driver *driver;
    void                    *driver_data;
};

//
// class-driver probe table
//

#define USB_ANY_CLASS    0xffu
#define USB_ANY_SUBCLASS 0xffu
#define USB_ANY_PROTOCOL 0xffu

struct usb_driver {
    uint8_t      match_class;
    uint8_t      match_subclass;
    uint8_t      match_protocol;
    const char  *name;
    int        (*probe)(struct usb_device *dev);
};

int usb_driver_register(const struct usb_driver *drv);

//
// Public API
//

struct usb_device *usb_alloc_device(void);
void usb_free_device(struct usb_device *dev);

int  usb_register_device(struct usb_device *dev);
void usb_unregister_device(struct usb_device *dev);

// Iterate the global device list. cb returns nonzero to stop early.
// Returns the cb's last return value, or 0 if it ran on every device.
int  usb_for_each_device(int (*cb)(struct usb_device *, void *), void *arg);

void usb_dump_devices(void);

// Issue a USB control transfer on EP0. Returns bytes transferred on
// success (possibly < length on short packet), or -1 on timeout / error.
int usb_control_transfer(struct usb_device *dev,
                         uint8_t request_type, uint8_t request,
                         uint16_t value, uint16_t index,
                         void *data, uint16_t length);

// Issue a USB bulk transfer on the given endpoint number. Returns bytes transferred or -1.
int usb_bulk_transfer(struct usb_device *dev, uint8_t ep,
                      void *data, size_t length, int dir);

// Composite helper: GET_DESCRIPTOR(type, index) into buf
int usb_get_descriptor(struct usb_device *dev,
                       uint8_t dt_type, uint8_t dt_index,
                       void *buf, uint16_t length);

#endif // __USB_H__
