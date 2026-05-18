#include <nautilus/nautilus.h>
#include <nautilus/mm.h>
#include <nautilus/naut_string.h>
#include <dev/usb.h>
#include <dev/usb_hub.h>
#include <dev/xhci.h>


#ifndef NAUT_CONFIG_DEBUG_USB_HUB
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...)   INFO_PRINT("usb_hub: " fmt, ##args)
#define DEBUG(fmt, args...)  DEBUG_PRINT("usb_hub: " fmt, ##args)
#define ERROR(fmt, args...)  ERROR_PRINT("usb_hub: " fmt, ##args)


// Class-recipient-device GET_DESCRIPTOR for the 7-byte hub descriptor prefix.
// USB 2.0 hubs return descriptor type 0x29; SuperSpeed hubs use 0x2A and
// have a different layout (not handled here -- Phase 1 is HS/FS only).
static int usb_hub_get_descriptor(struct usb_device *udev,
                                  struct usb_hub_descriptor *out) {
    memset(out, 0, sizeof(*out));
    int n = usb_control_transfer(udev,
                                 0xA0,                              // dev->host, class, device
                                 USB_REQ_GET_DESCRIPTOR,
                                 (uint16_t)(USB_DT_HUB << 8) | 0,   // wValue
                                 0,                                 // wIndex
                                 out, sizeof(*out));
    if (n < (int)sizeof(*out)) {
        ERROR("slot %u: hub descriptor read returned %d\n", udev->slot_id, n);
        return -1;
    }
    if (out->bDescriptorType != USB_DT_HUB) {
        ERROR("slot %u: unexpected hub descriptor type 0x%02x\n",
              udev->slot_id, out->bDescriptorType);
        return -1;
    }
    return 0;
}

// SET_PORT_FEATURE on a downstream port. wIndex carries the 1-based port
// number; class-other-recipient encoding in bmRequestType.
static int usb_hub_set_port_feature(struct usb_device *udev,
                                    uint8_t port, uint16_t feature) {
    int rc = usb_control_transfer(udev,
                                  0x23,                  // host->dev, class, other
                                  USB_REQ_SET_FEATURE,
                                  feature,
                                  (uint16_t)port,
                                  NULL, 0);
    if (rc < 0) {
        ERROR("slot %u: SET_PORT_FEATURE(%u, feat=%u) failed (rc=%d)\n",
              udev->slot_id, port, feature, rc);
        return -1;
    }
    return 0;
}


static int usb_hub_probe(struct usb_device *udev) {
    INFO("probing slot %u (vendor=0x%04x product=0x%04x dev_class=%u)\n",
         udev->slot_id, udev->vendor_id, udev->product_id, udev->dev_class);

    struct usb_hub_descriptor hd;
    if (usb_hub_get_descriptor(udev, &hd) < 0) {
        return -1;
    }

    uint8_t  ttt = (uint8_t)((hd.wHubCharacteristics & USB_HUB_CHAR_TTTT_MASK)
                             >> USB_HUB_CHAR_TTTT_SHIFT);
    // A multi-TT hub advertises its second interface (alt 1) with protocol=2;
    // we don't switch alts here, so a plain MTT detection would need a
    // second-interface scan. For Phase 1 default to single-TT and let
    // future work flip the bit when MTT is wired up.
    int mtt = 0;

    INFO("slot %u: hub descriptor: ports=%u, char=0x%04x, "
         "pwr_on_delay=%u ms, controller_current=%u mA, TTT=%u\n",
         udev->slot_id, hd.bNbrPorts, hd.wHubCharacteristics,
         hd.bPwrOn2PwrGood * 2, hd.bHubContrCurrent, ttt);

    if (hd.bNbrPorts == 0 || hd.bNbrPorts > 31) {
        ERROR("slot %u: implausible port count %u\n",
              udev->slot_id, hd.bNbrPorts);
        return -1;
    }

    struct usb_hub_dev *h = malloc(sizeof(*h));
    if (!h) {
        ERROR("slot %u: cannot allocate hub state\n", udev->slot_id);
        return -1;
    }
    memset(h, 0, sizeof(*h));
    h->udev              = udev;
    h->num_ports         = hd.bNbrPorts;
    h->mtt               = (uint8_t)mtt;
    h->ttt               = ttt;
    h->power_on_delay_ms = hd.bPwrOn2PwrGood * 2;

    // Tell the xHC this slot is a hub so subsequent descendants get the
    // right TT scheduling. Must happen before any downstream device tries
    // to enumerate through this hub.
    if (xhci_evaluate_hub_context(udev->hc, udev->slot_id,
                                  h->num_ports, h->mtt, h->ttt) < 0) {
        free(h);
        return -1;
    }

    // Power on every downstream port. Per USB 2.0 §11.11 the hub waits
    // bPwrOn2PwrGood (2 ms units) before the port is queryable; the
    // downstream device's PORT_CONNECTION bit may not be set until that
    // delay has passed. Phase 2 will need to honor power_on_delay_ms
    // before issuing GET_PORT_STATUS.
    for (uint8_t p = 1; p <= h->num_ports; p++) {
        if (usb_hub_set_port_feature(udev, p, USB_PORT_FEAT_POWER) < 0) {
            ERROR("slot %u: failed to power port %u\n", udev->slot_id, p);
            // Keep going -- some ports may still come up.
        } else {
            DEBUG("slot %u: powered port %u\n", udev->slot_id, p);
        }
    }
    INFO("slot %u: powered %u downstream port(s); waiting %u ms for power-good\n",
         udev->slot_id, h->num_ports, h->power_on_delay_ms);

    udev->driver_data = h;
    return 0;
}


// Hubs advertise their class at the device level (bDeviceClass=9), not the
// interface level. Match accordingly.
static const struct usb_driver usb_hub_driver = {
    .match_class    = USB_CLASS_HUB,
    .match_subclass = USB_ANY_SUBCLASS,
    .match_protocol = USB_ANY_PROTOCOL,
    .name           = "usb-hub",
    .probe          = usb_hub_probe,
};

int usb_hub_register(void) {
    return usb_driver_register(&usb_hub_driver);
}
