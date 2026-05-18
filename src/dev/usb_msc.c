#include <nautilus/nautilus.h>
#include <nautilus/mm.h>
#include <nautilus/naut_string.h>
#include <dev/usb.h>
#include <dev/usb_msc.h>
#include <dev/xhci.h>     // XHCI_CC_STALL_ERROR (transfer fns return -cc on stall)


#ifndef NAUT_CONFIG_DEBUG_USB_MSC
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...)   INFO_PRINT("usb_msc: " fmt, ##args)
#define DEBUG(fmt, args...)  DEBUG_PRINT("usb_msc: " fmt, ##args)
#define ERROR(fmt, args...)  ERROR_PRINT("usb_msc: " fmt, ##args)



// BBB Reset Recovery (USB MSC BBB §5.3.4): class-specific RESET request
// followed by CLEAR_FEATURE(ENDPOINT_HALT) on both bulk endpoints. Used
// after a CSW_PHASE_ERROR or back-to-back STALLs to put the device back
// into a known state.
static int usb_msc_reset_recovery(struct usb_msc_dev *msc) {
    INFO("BBB Reset Recovery on slot %u (iface %u)\n",
         msc->udev->slot_id, msc->iface);
    int rc = usb_control_transfer(msc->udev,
                                  0x21,                    // host->device, class, interface
                                  USB_MSC_REQ_RESET,
                                  0,                       // wValue
                                  msc->iface,              // wIndex
                                  NULL, 0);
    if (rc < 0) {
        ERROR("BBB Reset failed (rc=%d)\n", rc);
        return -1;
    }
    if (usb_clear_halt(msc->udev, msc->ep_in,  1) < 0) return -1;
    if (usb_clear_halt(msc->udev, msc->ep_out, 0) < 0) return -1;
    return 0;
}

// Three wire phases: CBW, data (0..N B) (optional), CSW
// Returns 0 on success. On short data the residue is
// returned in the out_residue argument.
// STALLs during the data or CSW phase are recovered with clear-halt
// (per BBB §6.7.2/6.7.3); a CSW_PHASE_ERROR triggers Reset Recovery.
static int usb_msc_bot(struct usb_msc_dev *msc,
                       const uint8_t *cdb, uint8_t cdb_len,
                       void *data, uint32_t data_len, int dir_in,
                       uint32_t *out_residue) {
    if (cdb_len < 1 || cdb_len > 16) {
        ERROR("invalid CDB length %u\n", cdb_len);
        return -1;
    }

    // CBW + CSW go through bulk transfers, which read from buffer pointers
    struct usb_msc_cbw cbw;
    memset(&cbw, 0, sizeof(cbw));
    cbw.signature   = USB_MSC_CBW_SIGNATURE;
    cbw.tag         = ++msc->tag_seq;
    cbw.data_length = data_len;
    cbw.flags       = dir_in ? USB_MSC_CBW_FLAG_IN : 0;
    cbw.lun         = 0;
    cbw.cdb_length  = cdb_len;
    memcpy(cbw.cdb, cdb, cdb_len);

    int n = usb_bulk_transfer(msc->udev, msc->ep_out, &cbw, sizeof(cbw), USB_DIR_OUT);
    if (n != (int)sizeof(cbw)) {
        ERROR("CBW write returned %d (expected %u)\n", n, (unsigned)sizeof(cbw));
        // A stalled bulk-OUT before the device even has the CBW is a sign
        // the device is wedged -- full BBB reset.
        if (n == -(int)XHCI_CC_STALL_ERROR) {
            usb_msc_reset_recovery(msc);
        }
        return -1;
    }

    uint32_t residue = data_len;
    if (data_len > 0 && data) {
        int dn = usb_bulk_transfer(msc->udev,
                                   dir_in ? msc->ep_in : msc->ep_out,
                                   data, data_len,
                                   dir_in ? USB_DIR_IN : USB_DIR_OUT);
        if (dn < 0) {
            if (dn == -(int)XHCI_CC_STALL_ERROR) {
                // BBB §6.7.2/6.7.3: device may stall the data endpoint to
                // signal command failure, but the CSW is still readable
                // after a clear-halt. Don't return yet -- read the CSW so
                // the caller learns the SCSI status.
                ERROR("data stage stalled (dir=%s len=%u); clearing halt\n",
                      dir_in ? "IN" : "OUT", data_len);
                usb_clear_halt(msc->udev,
                               dir_in ? msc->ep_in : msc->ep_out,
                               dir_in);
                residue = data_len;
            } else {
                ERROR("data stage failed (dir=%s len=%u rc=%d)\n",
                      dir_in ? "IN" : "OUT", data_len, dn);
                return -1;
            }
        } else {
            residue = data_len - (uint32_t)dn;
        }
    }

    struct usb_msc_csw csw;
    memset(&csw, 0, sizeof(csw));
    n = usb_bulk_transfer(msc->udev, msc->ep_in, &csw, sizeof(csw), USB_DIR_IN);
    if (n == -(int)XHCI_CC_STALL_ERROR) {
        // First CSW read stalled — clear halt and retry once.
        ERROR("CSW read stalled; clearing halt and retrying\n");
        usb_clear_halt(msc->udev, msc->ep_in, 1);
        n = usb_bulk_transfer(msc->udev, msc->ep_in, &csw, sizeof(csw), USB_DIR_IN);
    }
    if (n != (int)sizeof(csw)) {
        ERROR("CSW read returned %d (expected %u)\n", n, (unsigned)sizeof(csw));
        usb_msc_reset_recovery(msc);
        return -1;
    }
    if (csw.signature != USB_MSC_CSW_SIGNATURE) {
        ERROR("CSW signature 0x%08x invalid\n", csw.signature);
        usb_msc_reset_recovery(msc);
        return -1;
    }
    if (csw.tag != cbw.tag) {
        ERROR("CSW tag mismatch: sent 0x%x got 0x%x\n", cbw.tag, csw.tag);
        usb_msc_reset_recovery(msc);
        return -1;
    }
    if (csw.status == USB_MSC_CSW_PHASE_ERROR) {
        ERROR("CSW phase error (cdb opcode 0x%02x); reset recovery\n", cbw.cdb[0]);
        usb_msc_reset_recovery(msc);
        return -1;
    }
    if (csw.status != USB_MSC_CSW_OK) {
        ERROR("CSW status=%u (cdb opcode 0x%02x)\n", csw.status, cbw.cdb[0]);
        // Caller may follow up with usb_msc_request_sense() to learn why.
        return -1;
    }

    if (out_residue) {
        *out_residue = csw.data_residue ? csw.data_residue : residue;
    }
    return 0;
}


//
// SCSI command wrappers
//

// REQUEST_SENSE: 6-byte CDB asking for fixed-format sense data.
// Standard SCSI follow-up after any non-OK CSW.
int usb_msc_request_sense(struct usb_msc_dev *msc, struct scsi_sense_data *out) {
    uint8_t cdb[6] = {
        SCSI_OP_REQUEST_SENSE,
        0,                          // DESC=0 (fixed format)
        0, 0,                       // reserved
        sizeof(*out),               // allocation length (18)
        0,                          // control
    };
    memset(out, 0, sizeof(*out));
    return usb_msc_bot(msc, cdb, sizeof(cdb), out, sizeof(*out), 1, NULL);
}

int usb_msc_inquiry(struct usb_msc_dev *msc, struct scsi_inquiry_data *out) {
    uint8_t cdb[6] = { // command description block
        SCSI_OP_INQUIRY,
        0,                          // EVPD=0 (standard inquiry)
        0,                          // page code
        0,                          // alloc len hi
        sizeof(*out),               // alloc len lo
        0,                          // control
    };
    memset(out, 0, sizeof(*out));
    return usb_msc_bot(msc, cdb, sizeof(cdb), out, sizeof(*out), 1, NULL); // return residue
}

int usb_msc_read_capacity(struct usb_msc_dev *msc, uint32_t *out_last_lba, uint32_t *out_block_size) {
    uint8_t cdb[10] = {
        SCSI_OP_READ_CAPACITY_10,
        0, 0, 0, 0, 0,              // reserved + LBA (PMI=0 so LBA must be 0)
        0,                          // reserved
        0,                          // PMI=0
        0, 0,                       // reserved + control
    };
    uint8_t resp[8] = { 0 };
    if (usb_msc_bot(msc, cdb, sizeof(cdb), resp, sizeof(resp), 1, NULL) < 0) {
        return -1;
    }
  
    if (out_last_lba) { // logical block address
        *out_last_lba = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) |
                        ((uint32_t)resp[2] << 8)  |  (uint32_t)resp[3];
    }
    if (out_block_size) {
        *out_block_size = ((uint32_t)resp[4] << 24) | ((uint32_t)resp[5] << 16) |
                          ((uint32_t)resp[6] << 8)  |  (uint32_t)resp[7];
    }
    return 0;
}

// 10 here is metadata about the op
int usb_msc_read10(struct usb_msc_dev *msc, uint32_t lba, uint16_t nblocks,
                   void *buf, uint32_t *out_residue) {
    if (nblocks == 0) return 0;
    uint32_t bsize = msc->block_size ? msc->block_size : 512;
    uint32_t want  = (uint32_t)nblocks * bsize;
    uint8_t cdb[10] = {
        SCSI_OP_READ_10,                   // opcode
        0,                                 // flags (FUA=0, DPO=0)
        (uint8_t)(lba >> 24), (uint8_t)(lba >> 16), // lba
        (uint8_t)(lba >> 8),  (uint8_t)(lba & 0xff),
        0,                                 // group number
        (uint8_t)(nblocks >> 8), (uint8_t)(nblocks & 0xff), // block count
        0,                                 // control
    };
    return usb_msc_bot(msc, cdb, sizeof(cdb), buf, want, 1, out_residue);
}

// WRITE(10): mirror of READ(10) with bulk-OUT data phase.
int usb_msc_write10(struct usb_msc_dev *msc, uint32_t lba, uint16_t nblocks,
                    const void *buf, uint32_t *out_residue) {
    if (nblocks == 0) return 0;
    uint32_t bsize = msc->block_size ? msc->block_size : 512;
    uint32_t want  = (uint32_t)nblocks * bsize;
    uint8_t cdb[10] = {
        SCSI_OP_WRITE_10,                  // opcode
        0,                                 // flags (FUA=0, DPO=0)
        (uint8_t)(lba >> 24), (uint8_t)(lba >> 16),
        (uint8_t)(lba >> 8),  (uint8_t)(lba & 0xff),
        0,                                 // group number
        (uint8_t)(nblocks >> 8), (uint8_t)(nblocks & 0xff),
        0,                                 // control
    };
    // usb_msc_bot takes `void *` for the data buffer but only writes when
    // dir_in is set; here we're host->device so the cast strips const safely.
    return usb_msc_bot(msc, cdb, sizeof(cdb), (void *)buf, want, 0, out_residue);
}


// Returns max LUN index (so 0 means a single LUN)
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
// Per-driver state
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


// Copy a INQUIRY ASCII field into a buffer
static void copy_inq_field(char *dst, size_t dst_sz, const char *src, size_t src_len) {
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
    m->iface = 0;

    // Walk the parsed endpoint list for one bulk-IN and one bulk-OUT
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

    udev->driver_data = m;
    return 0;
}

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


// Registration
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
