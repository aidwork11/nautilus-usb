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

static int usb_hub_clear_port_feature(struct usb_device *udev,
                                      uint8_t port, uint16_t feature) {
    int rc = usb_control_transfer(udev, 0x23, USB_REQ_CLEAR_FEATURE,
                                  feature, (uint16_t)port, NULL, 0);
    if (rc < 0) {
        DEBUG("slot %u: CLEAR_PORT_FEATURE(%u, feat=%u) failed (rc=%d)\n",
              udev->slot_id, port, feature, rc);
        return -1;
    }
    return 0;
}

// GET_PORT_STATUS returns wPortStatus (low 16) + wPortChange (high 16)
// as a single 32-bit response. Packed into a uint32_t here for easy
// masking against USB_PORT_STAT_* / USB_PORT_CHG_* bits.
static int usb_hub_get_port_status(struct usb_device *udev,
                                   uint8_t port, uint32_t *out) {
    uint8_t buf[4] = { 0 };
    int n = usb_control_transfer(udev,
                                 0xA3,                   // dev->host, class, other
                                 USB_REQ_GET_STATUS,
                                 0, (uint16_t)port,
                                 buf, sizeof(buf));
    if (n < (int)sizeof(buf)) {
        ERROR("slot %u: GET_PORT_STATUS(%u) returned %d\n",
              udev->slot_id, port, n);
        return -1;
    }
    *out = (uint32_t)buf[0]       |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
    return 0;
}

// Decode xHCI speed code from a hub port status word. USB 2.0 hub status
// bits: bit 9 set = LS, bit 10 set = HS, neither = FS. Returns the
// corresponding USB_SPEED_* value, or 0 if the port reports nothing useful.
static uint32_t usb_hub_decode_speed(uint32_t status) {
    if (status & USB_PORT_STAT_HIGH_SPEED) return USB_SPEED_HS;
    if (status & USB_PORT_STAT_LOW_SPEED)  return USB_SPEED_LS;
    if (status & USB_PORT_STAT_CONNECTION) return USB_SPEED_FS;
    return 0;
}

// Issue PORT_RESET and poll until C_PORT_RESET asserts (USB 2.0 §11.5.1.5
// says reset takes ~10-20 ms). Adds a 10 ms post-reset recovery delay
// (TRSTRCY) before returning so the caller can immediately enumerate.
static int usb_hub_reset_port(struct usb_device *udev, uint8_t port) {
    if (usb_hub_set_port_feature(udev, port, USB_PORT_FEAT_RESET) < 0) {
        return -1;
    }
    for (int ms = 0; ms < 500; ms++) {
        uint32_t status;
        if (usb_hub_get_port_status(udev, port, &status) < 0) return -1;
        if (status & USB_PORT_CHG_RESET) {
            usb_hub_clear_port_feature(udev, port, USB_PORT_FEAT_C_RESET);
            udelay(10000);   // TRSTRCY: 10 ms before talking to the device
            return 0;
        }
        udelay(1000);
    }
    ERROR("slot %u: port %u reset timed out\n", udev->slot_id, port);
    return -1;
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
    // delay has passed.
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

    // After power-good, walk the ports and enumerate anything already
    // connected. Hot-plug events arriving later need a worker thread
    // that polls each hub periodically; that's not wired up yet.
    if (h->power_on_delay_ms > 0) {
        udelay((uint32_t)h->power_on_delay_ms * 1000);
    }
    for (uint8_t p = 1; p <= h->num_ports; p++) {
        uint32_t status = 0;
        if (usb_hub_get_port_status(udev, p, &status) < 0) continue;
        DEBUG("slot %u port %u: GET_PORT_STATUS=0x%08x\n",
              udev->slot_id, p, status);
        if (!(status & USB_PORT_STAT_CONNECTION)) continue;

        INFO("slot %u: port %u has device, resetting\n", udev->slot_id, p);
        // Ack any stale change bits before driving the reset.
        if (status & USB_PORT_CHG_CONNECTION) {
            usb_hub_clear_port_feature(udev, p, USB_PORT_FEAT_C_CONNECTION);
        }
        if (usb_hub_reset_port(udev, p) < 0) continue;

        // Re-read status -- the speed bits are only valid after reset.
        if (usb_hub_get_port_status(udev, p, &status) < 0) continue;
        uint32_t speed = usb_hub_decode_speed(status);
        if (speed == 0) {
            ERROR("slot %u port %u: no speed after reset (status=0x%08x)\n",
                  udev->slot_id, p, status);
            continue;
        }
        INFO("slot %u port %u: speed=%u, enumerating through hub\n",
             udev->slot_id, p, speed);
        xhci_enumerate_hub_port(udev->hc, udev, p, speed);
    }

    return 0;
}


// Release the per-hub state malloc'd by usb_hub_probe. Without this the
// usb_hub_dev leaks on every disconnect.
static void usb_hub_disconnect(struct usb_device *udev) {
    struct usb_hub_dev *h = (struct usb_hub_dev *)udev->driver_data;
    if (!h) return;
    INFO("slot %u: disconnect, freeing hub state\n", udev->slot_id);
    free(h);
}

// Hubs advertise their class at the device level (bDeviceClass=9), not the
// interface level. Match accordingly.
static const struct usb_driver usb_hub_driver = {
    .match_class    = USB_CLASS_HUB,
    .match_subclass = USB_ANY_SUBCLASS,
    .match_protocol = USB_ANY_PROTOCOL,
    .name           = "usb-hub",
    .probe          = usb_hub_probe,
    .disconnect     = usb_hub_disconnect,
};

int usb_hub_register(void) {
    return usb_driver_register(&usb_hub_driver);
}
