#ifndef __USB_MSC_H__
#define __USB_MSC_H__

#include <nautilus/naut_types.h>

struct usb_device;

// USB Mass Storage class constants
#define USB_CLASS_MASS_STORAGE      0x08
#define USB_MSC_SUBCLASS_SCSI       0x06
#define USB_MSC_PROTOCOL_BBB        0x50    // bulk only transport - BOT

// Class-specific control requests
#define USB_MSC_REQ_GET_MAX_LUN     0xFE // logical unit number
#define USB_MSC_REQ_RESET           0xFF 

// BOT wire formats
#define USB_MSC_CBW_SIGNATURE       0x43425355u    // USBC little-endian
#define USB_MSC_CSW_SIGNATURE       0x53425355u    // USBS little-endian

#define USB_MSC_CBW_FLAG_IN         0x80u          // bmCBWFlags bit 7

// command block wrapper, sent on bulk-out
struct usb_msc_cbw {
    uint32_t signature;
    uint32_t tag;
    uint32_t data_length;       // bytes for the optional data stage
    uint8_t  flags;             // bit 7 = direction
    uint8_t  lun;               // bits 3:0
    uint8_t  cdb_length;        // bits 4:0
    uint8_t  cdb[16];           // SCSI command description blocks
} __attribute__((packed));

// Command Status Wrapper, received on bulk-in
struct usb_msc_csw {
    uint32_t signature;
    uint32_t tag;               // must equal the CBWs tag so the host can match
    uint32_t data_residue;      // how many bytes the device did not transfer
    uint8_t  status;            // 0=passed, 1=failed, 2=phase error
} __attribute__((packed));

#define USB_MSC_CSW_OK              0x00
#define USB_MSC_CSW_FAILED          0x01
#define USB_MSC_CSW_PHASE_ERROR     0x02

// SCSI op codes we use
#define SCSI_OP_TEST_UNIT_READY     0x00
#define SCSI_OP_REQUEST_SENSE       0x03
#define SCSI_OP_INQUIRY             0x12
#define SCSI_OP_READ_CAPACITY_10    0x25
#define SCSI_OP_READ_10             0x28

// Standard INQUIRY data prefix (first 36 bytes are enough for ID strings).
struct scsi_inquiry_data {
    uint8_t  peripheral;            // bits 4:0 = device type; 0 = direct-access
    uint8_t  rmb;                   // bit 7 = removable
    uint8_t  version;
    uint8_t  response_format;
    uint8_t  additional_length;
    uint8_t  reserved[3];
    char     vendor[8];
    char     product[16];
    char     revision[4];
} __attribute__((packed));

// Per-device driver state. Bound to usb_device->driver_data after probe.
struct usb_msc_dev {
    struct usb_device *udev;
    uint8_t            iface;           // interface number
    uint8_t            max_lun;         // 0 = single LUN
    uint8_t            ep_in;           // bulk-IN endpoint number (1..15)
    uint8_t            ep_out;          // bulk-OUT endpoint number
    uint32_t           tag_seq;         // monotonic CBW tag
    uint32_t           block_size;      // bytes per LBA
    uint32_t           num_blocks;      // LBA count (last_lba + 1)
    char               vendor[9];       // null-terminated copies of INQUIRY fields
    char               product[17];
    char               revision[5];
};

//
// Public API
//

// Register the class driver. Called once before xHCI begins enumerating
int usb_msc_register(void);

// SCSI helpers. Return 0 on success, -1 on transport/command error
int usb_msc_inquiry(struct usb_msc_dev *msc,
                    struct scsi_inquiry_data *out);
int usb_msc_read_capacity(struct usb_msc_dev *msc,
                          uint32_t *out_last_lba, uint32_t *out_block_size);
int usb_msc_read10(struct usb_msc_dev *msc,
                   uint32_t lba, uint16_t nblocks, void *buf);

// Dump all bound MSC devices
void usb_msc_dump(void);

#endif // __USB_MSC_H__
