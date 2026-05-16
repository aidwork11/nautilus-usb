#include <nautilus/nautilus.h>
#include <nautilus/mm.h>
#include <nautilus/naut_string.h>
#include <dev/usb.h>
#include <dev/usb_msc.h>


#ifndef NAUT_CONFIG_DEBUG_USB_MSC
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...)   INFO_PRINT("usb_msc: " fmt, ##args)
#define DEBUG(fmt, args...)  DEBUG_PRINT("usb_msc: " fmt, ##args)
#define ERROR(fmt, args...)  ERROR_PRINT("usb_msc: " fmt, ##args)


//
// Bulk-Only Transport round trip
//
// Three wire phases:
//   1. CBW (31 B) on bulk-OUT  - command + direction + length
//   2. data (0..N B) on bulk-IN or bulk-OUT (optional, direction from CBW)
//   3. CSW (13 B) on bulk-IN   - status + residue
//
// Returns 0 on success (CSW status == 0). Returns -1 on any transport error
// or on CSW status != 0. On short data the residue is reported via the
// optional out_residue argument.
//
static int usb_msc_bot(struct usb_msc_dev *msc,
                       const uint8_t *cdb, uint8_t cdb_len,
                       void *data, uint32_t data_len, int dir_in,
                       uint32_t *out_residue) {
    if (cdb_len < 1 || cdb_len > 16) {
        ERROR("invalid CDB length %u\n", cdb_len);
        return -1;
    }

    // CBW + CSW go through bulk transfers, which read from buffer pointers.
    // Stack allocation is fine — the caller (probe or smoke test) is in
    // thread context with a generous stack, and the buffer outlives the
    // transfer because xhci_normal_transfer waits before returning.
    struct usb_msc_cbw cbw;
    memset(&cbw, 0, sizeof(cbw));
    cbw.signature   = USB_MSC_CBW_SIGNATURE;
    cbw.tag         = ++msc->tag_seq;
    cbw.data_length = data_len;
    cbw.flags       = dir_in ? USB_MSC_CBW_FLAG_IN : 0;
    cbw.lun         = 0;        // we only drive LUN 0 today
    cbw.cdb_length  = cdb_len;
    memcpy(cbw.cdb, cdb, cdb_len);

    int n = usb_bulk_transfer(msc->udev, msc->ep_out, &cbw, sizeof(cbw),
                              USB_DIR_OUT);
    if (n != (int)sizeof(cbw)) {
        ERROR("CBW write returned %d (expected %u)\n", n, (unsigned)sizeof(cbw));
        return -1;
    }

    uint32_t residue = data_len;
    if (data_len > 0 && data) {
        int dn = usb_bulk_transfer(msc->udev,
                                   dir_in ? msc->ep_in : msc->ep_out,
                                   data, data_len,
                                   dir_in ? USB_DIR_IN : USB_DIR_OUT);
        if (dn < 0) {
            ERROR("data stage failed (dir=%s len=%u)\n",
                  dir_in ? "IN" : "OUT", data_len);
            return -1;
        }
        residue = data_len - (uint32_t)dn;
    }

    struct usb_msc_csw csw;
    memset(&csw, 0, sizeof(csw));
    n = usb_bulk_transfer(msc->udev, msc->ep_in, &csw, sizeof(csw),
                          USB_DIR_IN);
    if (n != (int)sizeof(csw)) {
        ERROR("CSW read returned %d (expected %u)\n", n, (unsigned)sizeof(csw));
        return -1;
    }
    if (csw.signature != USB_MSC_CSW_SIGNATURE) {
        ERROR("CSW signature 0x%08x invalid\n", csw.signature);
        return -1;
    }
    if (csw.tag != cbw.tag) {
        ERROR("CSW tag mismatch: sent 0x%x got 0x%x\n", cbw.tag, csw.tag);
        return -1;
    }
    if (csw.status != USB_MSC_CSW_OK) {
        ERROR("CSW status=%u (cdb opcode 0x%02x)\n", csw.status, cbw.cdb[0]);
        return -1;
    }

    if (out_residue) {
        // Prefer the device's residue accounting, falling back to the
        // bulk transfer's residue if the device reported 0.
        *out_residue = csw.data_residue ? csw.data_residue : residue;
    }
    return 0;
}


//
// SCSI command wrappers
//

int usb_msc_inquiry(struct usb_msc_dev *msc,
                    struct scsi_inquiry_data *out) {
    uint8_t cdb[6] = {
        SCSI_OP_INQUIRY,
        0,                          // EVPD=0 (standard inquiry)
        0,                          // page code
        0,                          // alloc len hi
        sizeof(*out),               // alloc len lo
        0,                          // control
    };
    memset(out, 0, sizeof(*out));
    return usb_msc_bot(msc, cdb, sizeof(cdb), out, sizeof(*out),
                       1 /* dir_in */, NULL);
}

int usb_msc_read_capacity(struct usb_msc_dev *msc,
                          uint32_t *out_last_lba, uint32_t *out_block_size) {
    uint8_t cdb[10] = {
        SCSI_OP_READ_CAPACITY_10,
        0, 0, 0, 0, 0,              // reserved + LBA (PMI=0 so LBA must be 0)
        0,                          // reserved
        0,                          // PMI=0
        0, 0,                       // reserved + control
    };
    uint8_t resp[8] = { 0 };
    if (usb_msc_bot(msc, cdb, sizeof(cdb), resp, sizeof(resp),
                    1 /* dir_in */, NULL) < 0) {
        return -1;
    }
    // Both fields are big-endian 32-bit.
    if (out_last_lba) {
        *out_last_lba = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) |
                        ((uint32_t)resp[2] << 8)  |  (uint32_t)resp[3];
    }
    if (out_block_size) {
        *out_block_size = ((uint32_t)resp[4] << 24) | ((uint32_t)resp[5] << 16) |
                          ((uint32_t)resp[6] << 8)  |  (uint32_t)resp[7];
    }
    return 0;
}

int usb_msc_read10(struct usb_msc_dev *msc,
                   uint32_t lba, uint16_t nblocks, void *buf) {
    if (nblocks == 0) return 0;
    uint32_t bsize = msc->block_size ? msc->block_size : 512;
    uint32_t want  = (uint32_t)nblocks * bsize;
    uint8_t cdb[10] = {
        SCSI_OP_READ_10,
        0,                                 // flags (FUA=0, DPO=0)
        (uint8_t)(lba >> 24), (uint8_t)(lba >> 16),
        (uint8_t)(lba >> 8),  (uint8_t)(lba & 0xff),
        0,                                 // group number
        (uint8_t)(nblocks >> 8), (uint8_t)(nblocks & 0xff),
        0,                                 // control
    };
    return usb_msc_bot(msc, cdb, sizeof(cdb), buf, want,
                       1 /* dir_in */, NULL);
}


//
// Get Max LUN — class-specific control request. Returns max LUN index
// (so 0 means a single LUN). Some devices STALL this; treat that as
// "single LUN" rather than failing the probe.
//
static int usb_msc_get_max_lun(struct usb_msc_dev *msc, uint8_t *out_max_lun) {
    uint8_t resp = 0;
    int n = usb_control_transfer(msc->udev,
                                 0xA1,                          // device-to-host, class, interface
                                 USB_MSC_REQ_GET_MAX_LUN,
                                 0,                             // wValue
                                 msc->iface,                    // wIndex
                                 &resp, 1);
    if (n < 0) {
        DEBUG("Get Max LUN stalled; assuming single LUN\n");
        *out_max_lun = 0;
        return 0;
    }
    if (n != 1) {
        ERROR("Get Max LUN returned %d bytes\n", n);
        return -1;
    }
    *out_max_lun = resp & 0x0f;
    return 0;
}


//
// Per-driver state. Static array because nothing in NK frees class
// drivers, and the table is bounded by max_slots (= 64 today).
//

#define USB_MSC_MAX_DEVS 8
static struct usb_msc_dev usb_msc_devs[USB_MSC_MAX_DEVS];
static uint32_t           usb_msc_devs_n = 0;

static struct usb_msc_dev *usb_msc_alloc(void) {
    if (usb_msc_devs_n >= USB_MSC_MAX_DEVS) {
        ERROR("device table full\n");
        return NULL;
    }
    struct usb_msc_dev *m = &usb_msc_devs[usb_msc_devs_n++];
    memset(m, 0, sizeof(*m));
    return m;
}


// Copy a fixed-width, space-padded INQUIRY ASCII field into a
// null-terminated buffer, trimming trailing spaces.
static void copy_inq_field(char *dst, size_t dst_sz,
                           const char *src, size_t src_len) {
    size_t n = src_len < dst_sz - 1 ? src_len : dst_sz - 1;
    memcpy(dst, src, n);
    dst[n] = 0;
    while (n > 0 && dst[n - 1] == ' ') {
        dst[--n] = 0;
    }
}


//
// Probe
//
// Match runs on a single interface. We discover bulk endpoints, query
// max LUN, do INQUIRY + READ CAPACITY, and as a smoke test read sector
// 0 and log the first bytes. Returns 0 on bind, nonzero to decline.
//
static int usb_msc_probe(struct usb_device *udev) {
    INFO("probing slot %u (vendor=0x%04x product=0x%04x iface=%u/%u/%u)\n",
         udev->slot_id, udev->vendor_id, udev->product_id,
         udev->iface_class, udev->iface_subclass, udev->iface_protocol);

    struct usb_msc_dev *m = usb_msc_alloc();
    if (!m) return -1;
    m->udev  = udev;
    m->iface = 0;       // single-interface parser; always interface 0

    // Walk the parsed endpoint list for one bulk-IN and one bulk-OUT.
    for (uint32_t i = 0; i < udev->num_endpoints; i++) {
        struct usb_endpoint *e = &udev->endpoints[i];
        if (!e->active) continue;
        if ((e->attributes & USB_EP_XFER_MASK) != USB_EP_XFER_BULK) continue;
        uint8_t num = USB_EP_NUM(e->address);
        if (USB_EP_DIR_IN(e->address)) {
            if (m->ep_in == 0)  m->ep_in  = num;
        } else {
            if (m->ep_out == 0) m->ep_out = num;
        }
    }
    if (m->ep_in == 0 || m->ep_out == 0) {
        ERROR("slot %u: missing bulk IN (%u) or OUT (%u) endpoint\n",
              udev->slot_id, m->ep_in, m->ep_out);
        return -1;
    }
    DEBUG("slot %u: bulk IN ep%u, OUT ep%u\n",
          udev->slot_id, m->ep_in, m->ep_out);

    if (usb_msc_get_max_lun(m, &m->max_lun) < 0) {
        return -1;
    }
    INFO("slot %u: max_lun=%u\n", udev->slot_id, m->max_lun);

    struct scsi_inquiry_data inq;
    if (usb_msc_inquiry(m, &inq) < 0) {
        return -1;
    }
    copy_inq_field(m->vendor,   sizeof(m->vendor),   inq.vendor,   8);
    copy_inq_field(m->product,  sizeof(m->product),  inq.product, 16);
    copy_inq_field(m->revision, sizeof(m->revision), inq.revision, 4);
    INFO("slot %u: INQUIRY vendor='%s' product='%s' rev='%s' type=%u removable=%u\n",
         udev->slot_id, m->vendor, m->product, m->revision,
         inq.peripheral & 0x1f, !!(inq.rmb & 0x80));

    uint32_t last_lba = 0;
    if (usb_msc_read_capacity(m, &last_lba, &m->block_size) < 0) {
        return -1;
    }
    m->num_blocks = last_lba + 1;
    INFO("slot %u: capacity %u blocks x %u B (%u MiB)\n",
         udev->slot_id, m->num_blocks, m->block_size,
         (uint32_t)(((uint64_t)m->num_blocks * m->block_size) >> 20));

    // Smoke test: read LBA 0 and log the first 16 bytes. Class drivers
    // wouldn't normally I/O during probe — this is here so the test plan
    // (T5) has an automatic check that the full path works end-to-end.
    if (m->block_size > 0 && m->block_size <= 4096) {
        uint8_t *buf = kmem_mallocz(m->block_size);
        if (buf) {
            if (usb_msc_read10(m, 0, 1, buf) == 0) {
                INFO("slot %u: LBA 0 first 16 B: "
                     "%02x %02x %02x %02x %02x %02x %02x %02x "
                     "%02x %02x %02x %02x %02x %02x %02x %02x\n",
                     udev->slot_id,
                     buf[0],  buf[1],  buf[2],  buf[3],
                     buf[4],  buf[5],  buf[6],  buf[7],
                     buf[8],  buf[9],  buf[10], buf[11],
                     buf[12], buf[13], buf[14], buf[15]);
            } else {
                ERROR("slot %u: READ(10) of LBA 0 failed\n", udev->slot_id);
            }
            kmem_free(buf);
        }
    }

    udev->driver_data = m;
    return 0;
}


//
// usb_dump_devices counterpart for MSC. Iterates our static table.
//

void usb_msc_dump(void) {
    if (usb_msc_devs_n == 0) {
        INFO("no MSC devices bound\n");
        return;
    }
    for (uint32_t i = 0; i < usb_msc_devs_n; i++) {
        struct usb_msc_dev *m = &usb_msc_devs[i];
        if (!m->udev) continue;
        INFO("  slot %u  '%s' '%s' '%s'  %u x %u B  ep%u/ep%u\n",
             m->udev->slot_id, m->vendor, m->product, m->revision,
             m->num_blocks, m->block_size, m->ep_in, m->ep_out);
    }
}


//
// Registration
//

static const struct usb_driver usb_msc_driver = {
    .match_class    = USB_CLASS_MASS_STORAGE,
    .match_subclass = USB_MSC_SUBCLASS_SCSI,
    .match_protocol = USB_MSC_PROTOCOL_BBB,
    .name           = "usb-msc",
    .probe          = usb_msc_probe,
};

int usb_msc_register(void) {
    return usb_driver_register(&usb_msc_driver);
}
