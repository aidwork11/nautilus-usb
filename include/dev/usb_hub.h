#ifndef __USB_HUB_H__
#define __USB_HUB_H__

#include <nautilus/naut_types.h>

struct usb_device;

// USB 2.0 hub class descriptor type
#define USB_DT_HUB                 0x29

// Hub class-specific control requests use the standard request codes
// (GET_STATUS=0, CLEAR_FEATURE=1, SET_FEATURE=3) but a class+other recipient
// in bmRequestType. Wrappers below pick the right encoding.

// Hub characteristics (wHubCharacteristics) bit fields
#define USB_HUB_CHAR_LPSM_MASK     0x0003  // Logical Power Switching Mode
#define USB_HUB_CHAR_COMPOUND      0x0004  // Identifies compound device
#define USB_HUB_CHAR_OCPM_MASK     0x0018  // Over-current Protection Mode
#define USB_HUB_CHAR_TTTT_MASK     0x0060  // TT Think Time (8/16/24/32 FS bit times)
#define USB_HUB_CHAR_TTTT_SHIFT    5
#define USB_HUB_CHAR_PORTIND       0x0080  // Port indicators supported

// Port feature selectors (wValue for SET_FEATURE / CLEAR_FEATURE on a port)
#define USB_PORT_FEAT_CONNECTION       0
#define USB_PORT_FEAT_ENABLE           1
#define USB_PORT_FEAT_SUSPEND          2
#define USB_PORT_FEAT_OVERCURRENT      3
#define USB_PORT_FEAT_RESET            4
#define USB_PORT_FEAT_POWER            8
#define USB_PORT_FEAT_LOWSPEED         9
#define USB_PORT_FEAT_C_CONNECTION    16
#define USB_PORT_FEAT_C_ENABLE        17
#define USB_PORT_FEAT_C_SUSPEND       18
#define USB_PORT_FEAT_C_OVERCURRENT   19
#define USB_PORT_FEAT_C_RESET         20

// USB 2.0 hub descriptor. Variable-length tail (DeviceRemovable + PortPwrCtrlMask
// bitmaps) trails the 7-byte header; we only consume the fixed prefix.
struct usb_hub_descriptor {
    uint8_t  bDescLength;
    uint8_t  bDescriptorType;       // 0x29
    uint8_t  bNbrPorts;             // number of downstream ports
    uint16_t wHubCharacteristics;
    uint8_t  bPwrOn2PwrGood;        // in 2 ms units
    uint8_t  bHubContrCurrent;      // mA
    // Variable-length: DeviceRemovable bitmap, then PortPwrCtrlMask
} __attribute__((packed));

// Per-device driver state, bound to usb_device->driver_data
struct usb_hub_dev {
    struct usb_device *udev;
    uint8_t  num_ports;
    uint8_t  mtt;                   // 1 if multi-TT hub
    uint8_t  ttt;                   // TT Think Time encoded value (0..3)
    uint8_t  power_on_delay_ms;     // bPwrOn2PwrGood * 2
};

// Register the hub class driver. Called once before xHCI begins enumerating.
int usb_hub_register(void);

_Static_assert(sizeof(struct usb_hub_descriptor) == 7,
               "usb_hub_descriptor prefix must be 7 bytes");

#endif // __USB_HUB_H__
