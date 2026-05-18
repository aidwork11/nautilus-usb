#ifndef __XHCI_H__
#define __XHCI_H__

#include <nautilus/naut_types.h>
#include <nautilus/list.h>
#include <nautilus/spinlock.h>
#include <nautilus/waitqueue.h>
#include <dev/usb.h>

struct pci_dev;
struct naut_info;

//
// MMIO accessors
//

#define xhci_readb(addr)      (*((volatile uint8_t  *)(addr)))
#define xhci_readw(addr)      (*((volatile uint16_t *)(addr)))
#define xhci_readl(addr)      (*((volatile uint32_t *)(addr)))
#define xhci_readq(addr)      (*((volatile uint64_t *)(addr)))

#define xhci_writeb(addr, v)  ((*((volatile uint8_t  *)(addr))) = (v))
#define xhci_writew(addr, v)  ((*((volatile uint16_t *)(addr))) = (v))
#define xhci_writel(addr, v)  ((*((volatile uint32_t *)(addr))) = (v))
#define xhci_writeq(addr, v)  ((*((volatile uint64_t *)(addr))) = (v))

// Prevent CPU/Compiler from reordering instructions in a way that might break hardware interactions
#define xhci_mb()             __asm__ volatile("mfence" ::: "memory") // read/write barrier. MB = Memory barrier
#define xhci_rmb()            __asm__ volatile("lfence" ::: "memory") // wait for all prior loads to finish before running next load. RMB = Read Memory Barrier
#define xhci_wmb()            __asm__ volatile("sfence" ::: "memory") // wait for all prior stores to finish before running next store. WMB = Write Memory Barrier

//
// PCI class identifiers for xHCI. Used at boot when the PCI bus is scanned to register xHCI
//

#define XHCI_PCI_CLASS         0x0c   // Serial Bus Controller
#define XHCI_PCI_SUBCLASS      0x03   // USB controller
#define XHCI_PCI_PROGIF        0x30   // xHCI interface

//
// Capability Registers (offsets from MMIO base).
// xHCI controller specific info that tells the driver how to configure itself
//

#define XHCI_CAP_CAPLENGTH     0x00   //  8-bit capability register length
#define XHCI_CAP_HCIVERSION    0x02   // 16-bit spec version (BCD) (xHCI v1.1, v1.2, etc.)
#define XHCI_CAP_HCSPARAMS1    0x04   // MaxSlots/MaxIntrs/MaxPorts (how many USB devices can be plugged in, how many ports on the motherboard, etc.)
#define XHCI_CAP_HCSPARAMS2    0x08   // IST, ERST max, scratchpad bufs (scratchpad is memory in RAM that the OS gives the controller to use)
#define XHCI_CAP_HCSPARAMS3    0x0C   // U1/U2 exit latencies (low power states i.e. deep sleep modes)
#define XHCI_CAP_HCCPARAMS1    0x10   // AC64, CSZ, ext-cap pointer (64 bit mem capability, 
#define XHCI_CAP_DBOFF         0x14   // Doorbell array offset
#define XHCI_CAP_RTSOFF        0x18   // Runtime register offset
#define XHCI_CAP_HCCPARAMS2    0x1C

// HCSPARAMS1 fields
#define XHCI_HCS1_MAXSLOTS(x)  ((x) & 0xff)
#define XHCI_HCS1_MAXINTRS(x)  (((x) >> 8) & 0x7ff)
#define XHCI_HCS1_MAXPORTS(x)  (((x) >> 24) & 0xff)

// HCSPARAMS2 fields
#define XHCI_HCS2_IST(x)             ((x) & 0xf)
#define XHCI_HCS2_ERST_MAX(x)        (((x) >> 4) & 0xf)
#define XHCI_HCS2_SPB_MAX_HI(x)      (((x) >> 21) & 0x1f)
#define XHCI_HCS2_SPB_MAX_LO(x)      (((x) >> 27) & 0x1f)
#define XHCI_HCS2_SPB_MAX(x) \
        ((XHCI_HCS2_SPB_MAX_HI(x) << 5) | XHCI_HCS2_SPB_MAX_LO(x))

// HCCPARAMS1 fields
#define XHCI_HCC1_AC64(x)      ((x) & 0x1)         // 64-bit addressing
#define XHCI_HCC1_BNC(x)       (((x) >> 1) & 0x1)  // bandwidth negotiation
#define XHCI_HCC1_CSZ(x)       (((x) >> 2) & 0x1)  // 0 = 32B, 1 = 64B ctx
#define XHCI_HCC1_PPC(x)       (((x) >> 3) & 0x1)  // port power control
#define XHCI_HCC1_PIND(x)      (((x) >> 4) & 0x1)  // port indicator ctl
#define XHCI_HCC1_LHRC(x)      (((x) >> 5) & 0x1)  // light HC reset cap
#define XHCI_HCC1_LTC(x)       (((x) >> 6) & 0x1)  // latency tolerance
#define XHCI_HCC1_NSS(x)       (((x) >> 7) & 0x1)  // no secondary SID
#define XHCI_HCC1_MAX_PSA(x)   (((x) >> 12) & 0xf) // max primary stream
#define XHCI_HCC1_XECP(x)      (((x) >> 16) & 0xffff) // ext-cap ptr (dwords)

// xHCI Extended Capabilities
#define XHCI_EXT_CAP_ID_USB_LEGACY     1
#define XHCI_USBLEGSUP_BIOS_SEM_OFF    2    // byte offset within cap, bit 0 = BIOS owned
#define XHCI_USBLEGSUP_OS_SEM_OFF      3    // byte offset within cap, bit 0 = OS owned

//
// Operational Registers (offsets from base + CAPLENGTH).
// Read caplength register to get offset
//

#define XHCI_OP_USBCMD         0x00   // USB command. Start/stop switch
#define XHCI_OP_USBSTS         0x04   // USB Status. Is the controller running? 
#define XHCI_OP_PAGESIZE       0x08
#define XHCI_OP_DNCTRL         0x14
#define XHCI_OP_CRCR           0x18   // Command Ring Control Register. Holds the physical address of the command ring
#define XHCI_OP_DCBAAP         0x30   // Device Context Base Address Array Pointer. Points to an array of pointers in RAM. Each pointer in that array leads to the state of a USB device
#define XHCI_OP_CONFIG         0x38   // How many device slots are enabled
#define XHCI_OP_PORT_BASE      0x400  // per-port reg block
#define XHCI_OP_PORT_STRIDE    0x10   // how far apart the registers for each USB port are

// Per-port register offsets within a port's stride
#define XHCI_PORT_PORTSC       0x00   // status and control. is something plugged in (bit 0), etc
#define XHCI_PORT_PORTPMSC     0x04   // power management status and control
#define XHCI_PORT_PORTLI       0x08   // link info 
#define XHCI_PORT_PORTHLPMC    0x0C   // hardware link power management control

// USBCMD bits
#define XHCI_CMD_RS            (1u << 0)   // run/stop
#define XHCI_CMD_HCRST         (1u << 1)   // host controller reset
#define XHCI_CMD_INTE          (1u << 2)   // interrupter enable
#define XHCI_CMD_HSEE          (1u << 3)   // host system error enable
#define XHCI_CMD_LHCRST        (1u << 7)   // light HC reset
#define XHCI_CMD_CSS           (1u << 8)   // controller save state
#define XHCI_CMD_CRS           (1u << 9)   // controller restore state
#define XHCI_CMD_EWE           (1u << 10)  // enable wrap event
#define XHCI_CMD_EU3S          (1u << 11)  // enable U3 MFINDEX stop

// USBSTS bits
#define XHCI_STS_HCH           (1u << 0)   // HC halted
#define XHCI_STS_HSE           (1u << 2)   // host system error
#define XHCI_STS_EINT          (1u << 3)   // event interrupt (W1C)
#define XHCI_STS_PCD           (1u << 4)   // port change detect (W1C)
#define XHCI_STS_SSS           (1u << 8)   // save state status 
#define XHCI_STS_RSS           (1u << 9)   // restore state status 
#define XHCI_STS_SRE           (1u << 10)  // save/restore error (W1C)
#define XHCI_STS_CNR           (1u << 11)  // controller not ready 
#define XHCI_STS_HCE           (1u << 12)  // host controller error

// CRCR bits
#define XHCI_CRCR_RCS          (1u << 0)   // ring cycle state
#define XHCI_CRCR_CS           (1u << 1)   // command stop
#define XHCI_CRCR_CA           (1u << 2)   // command abort
#define XHCI_CRCR_CRR          (1u << 3)   // command ring running

// CONFIG bits
#define XHCI_CONFIG_MAXSLOTSEN_MASK  0xff

// PORTSC bits
#define XHCI_PORTSC_CCS        (1u << 0)   // current connect status
#define XHCI_PORTSC_PED        (1u << 1)   // port enabled/disabled 
#define XHCI_PORTSC_OCA        (1u << 3)   // overcurrent active
#define XHCI_PORTSC_PR         (1u << 4)   // port reset
#define XHCI_PORTSC_PLS_SHIFT  5
#define XHCI_PORTSC_PLS_MASK   (0xfu << XHCI_PORTSC_PLS_SHIFT)
#define XHCI_PORTSC_PP         (1u << 9)   // port power
#define XHCI_PORTSC_SPEED_SHIFT 10
#define XHCI_PORTSC_SPEED_MASK (0xfu << XHCI_PORTSC_SPEED_SHIFT)
#define XHCI_PORTSC_PIC_SHIFT  14
#define XHCI_PORTSC_PIC_MASK   (0x3u << XHCI_PORTSC_PIC_SHIFT)
#define XHCI_PORTSC_LWS        (1u << 16)  // link state write strobe
#define XHCI_PORTSC_CSC        (1u << 17)  // connect status change   (W1C)
#define XHCI_PORTSC_PEC        (1u << 18)  // port enable/disable chg (W1C)
#define XHCI_PORTSC_WRC        (1u << 19)  // warm port reset change  (W1C)
#define XHCI_PORTSC_OCC        (1u << 20)  // overcurrent change      (W1C)
#define XHCI_PORTSC_PRC        (1u << 21)  // port reset change       (W1C)
#define XHCI_PORTSC_PLC        (1u << 22)  // port link state change  (W1C)
#define XHCI_PORTSC_CEC        (1u << 23)  // port config error change(W1C)
#define XHCI_PORTSC_CAS        (1u << 24)  // cold attach status
#define XHCI_PORTSC_WCE        (1u << 25)  // wake on connect enable
#define XHCI_PORTSC_WDE        (1u << 26)  // wake on disconnect enable
#define XHCI_PORTSC_WOE        (1u << 27)  // wake on overcurrent enable 
#define XHCI_PORTSC_DR         (1u << 30)  // device removable
#define XHCI_PORTSC_WPR        (1u << 31)  // warm port reset (USB3)

#define XHCI_PORTSC_W1C_MASK \
        (XHCI_PORTSC_CSC | XHCI_PORTSC_PEC | XHCI_PORTSC_WRC | \
         XHCI_PORTSC_OCC | XHCI_PORTSC_PRC | XHCI_PORTSC_PLC | \
         XHCI_PORTSC_CEC)

// USB speeds reported in PORTSC
#define XHCI_SPEED_FS          1   // Full-Speed   12 Mb/s
#define XHCI_SPEED_LS          2   // Low-Speed     1.5 Mb/s
#define XHCI_SPEED_HS          3   // High-Speed  480 Mb/s
#define XHCI_SPEED_SS          4   // SuperSpeed    5 Gb/s
#define XHCI_SPEED_SSP         5   // SuperSpeedPlus 10 Gb/s

//
// Runtime Registers (offsets from base + RTSOFF).
// Handling the event loop of the controller
//

#define XHCI_RT_MFINDEX        0x00   // timer synchronized with the USB bus. Used to send a transfer at a specifc time
#define XHCI_RT_IR_BASE        0x20   // start of the first interruptor
#define XHCI_RT_IR_STRIDE      0x20   // offset to the following interruptor

// Interrupter register offsets relative to its base
#define XHCI_IR_IMAN           0x00   // interrupt management
#define XHCI_IR_IMOD           0x04   // interrupt moderation 
#define XHCI_IR_ERSTSZ         0x08   // ERST size (low 16 bits)
#define XHCI_IR_ERSTBA         0x10   // ERST base address (64-bit)
#define XHCI_IR_ERDP           0x18   // event ring dequeue ptr (64-bit) 

// IMAN bits
#define XHCI_IMAN_IP           (1u << 0)  // interrupt pending (W1C)
#define XHCI_IMAN_IE           (1u << 1)  // interrupt enable

// ERDP bits (low 4 bits within the 64-bit value)
#define XHCI_ERDP_DESI_MASK    0x7u
#define XHCI_ERDP_EHB          (1u << 3)  // event handler busy (W1C). tells hardware the processor has finished handling an interrupt (thus reset the interrupt signal)
#define XHCI_ERDP_PTR_MASK     (~0xfULL)  // strip the status bits from the address

//
// Doorbell Registers (offsets from base + DBOFF)
// Alerting the hardware that the processor has given it new work
//

#define XHCI_DB_HOST           0    // doorbell 0 = command ring
#define XHCI_DB_TARGET(t)      ((t) & 0xff)             // endpoint ID
#define XHCI_DB_STREAM(s)      (((s) & 0xffff) << 16)   // stream ID

#define XHCI_DB_CMD_DOORBELL   0    // value to ring command doorbell

//
// Transfer Request Block layout and type constants
//

struct xhci_trb {
    uint64_t param;     // address or immediate data
    uint32_t status;    // transfer metadata - how many bytes to transfer, error codes, etc
    uint32_t control;   // defines what the TRB actually is
} __attribute__((packed));

// control-field layout helpers 
#define XHCI_TRB_CYCLE         (1u << 0)
#define XHCI_TRB_ENT           (1u << 1)   // evaluate next TRB
#define XHCI_TRB_ISP           (1u << 2)   // interrupt on short packet
#define XHCI_TRB_NS            (1u << 3)   // no snoop
#define XHCI_TRB_CH            (1u << 4)   // chain bit
#define XHCI_TRB_IOC           (1u << 5)   // interrupt on completion
#define XHCI_TRB_IDT           (1u << 6)   // immediate data
#define XHCI_TRB_BSR           (1u << 9)   // block set address request (ADDRESS_DEVICE only)
#define XHCI_TRB_SIA           (1u << 9)   // start isoch ASAP (ISOCH only)
#define XHCI_TRB_TC            (1u << 1)   // toggle cycle (LINK only)
#define XHCI_TRB_DIR_IN        (1u << 16)  // data stage direction = IN
#define XHCI_TRB_TYPE_SHIFT    10
#define XHCI_TRB_TYPE_MASK     (0x3fu << XHCI_TRB_TYPE_SHIFT)
#define XHCI_TRB_TYPE(t)       (((t) & 0x3fu) << XHCI_TRB_TYPE_SHIFT)
#define XHCI_TRB_GET_TYPE(c)   (((c) >> XHCI_TRB_TYPE_SHIFT) & 0x3fu)

// Slot ID lives in control[31:24] for slot-targeted commands and events
#define XHCI_TRB_SLOT_ID(s)    (((s) & 0xffu) << 24)
#define XHCI_TRB_GET_SLOT(c)   (((c) >> 24) & 0xffu)

// Endpoint ID lives in control[20:16] of TRANSFER_EVENT TRBs
#define XHCI_TRB_GET_EP(c)     (((c) >> 16) & 0x1fu)

// Transfer type
#define XHCI_TRB_SETUP_TRT_SHIFT     16
#define XHCI_TRB_SETUP_TRT_NO_DATA   (0u << XHCI_TRB_SETUP_TRT_SHIFT)
#define XHCI_TRB_SETUP_TRT_OUT       (2u << XHCI_TRB_SETUP_TRT_SHIFT)
#define XHCI_TRB_SETUP_TRT_IN        (3u << XHCI_TRB_SETUP_TRT_SHIFT)

// status-field helpers
#define XHCI_TRB_LEN(s)        ((s) & 0x1ffff)
#define XHCI_TRB_TD_SIZE(n)    (((n) & 0x1f) << 17)
#define XHCI_TRB_INTR(i)       (((i) & 0x3ff) << 22)
#define XHCI_TRB_GET_COMP(s)   (((s) >> 24) & 0xff)
#define XHCI_TRB_GET_LEN(s)    ((s) & 0xffffff)

// TRB types
enum xhci_trb_type {
    XHCI_TRB_RESERVED            = 0,

    // Transfer ring TRBs
    XHCI_TRB_NORMAL              = 1,
    XHCI_TRB_SETUP_STAGE         = 2,
    XHCI_TRB_DATA_STAGE          = 3,
    XHCI_TRB_STATUS_STAGE        = 4,
    XHCI_TRB_ISOCH               = 5,
    XHCI_TRB_LINK                = 6,
    XHCI_TRB_EVENT_DATA          = 7,
    XHCI_TRB_NO_OP               = 8,

    // Command ring TRBs
    XHCI_TRB_ENABLE_SLOT         = 9,
    XHCI_TRB_DISABLE_SLOT        = 10,
    XHCI_TRB_ADDRESS_DEVICE      = 11,
    XHCI_TRB_CONFIGURE_ENDPOINT  = 12,
    XHCI_TRB_EVALUATE_CONTEXT    = 13,
    XHCI_TRB_RESET_ENDPOINT      = 14,
    XHCI_TRB_STOP_ENDPOINT       = 15,
    XHCI_TRB_SET_TR_DEQUEUE      = 16,
    XHCI_TRB_RESET_DEVICE        = 17,
    XHCI_TRB_FORCE_EVENT         = 18,
    XHCI_TRB_NEG_BANDWIDTH       = 19,
    XHCI_TRB_SET_LATENCY_TOL     = 20,
    XHCI_TRB_GET_PORT_BANDWIDTH  = 21,
    XHCI_TRB_FORCE_HEADER        = 22,
    XHCI_TRB_NO_OP_CMD           = 23,

    // Event ring TRBs
    XHCI_TRB_TRANSFER_EVENT      = 32,
    XHCI_TRB_COMMAND_COMPLETION  = 33,
    XHCI_TRB_PORT_STATUS_CHANGE  = 34,
    XHCI_TRB_BANDWIDTH_REQUEST   = 35,
    XHCI_TRB_DOORBELL_EVENT      = 36,
    XHCI_TRB_HOST_CONTROLLER_EVT = 37,
    XHCI_TRB_DEVICE_NOTIFICATION = 38,
    XHCI_TRB_MFINDEX_WRAP        = 39,
};

// Completion codes
enum xhci_completion_code {
    XHCI_CC_INVALID              = 0,
    XHCI_CC_SUCCESS              = 1,
    XHCI_CC_DATA_BUFFER_ERROR    = 2,
    XHCI_CC_BABBLE_DETECTED      = 3,
    XHCI_CC_USB_TRANSACTION_ERR  = 4,
    XHCI_CC_TRB_ERROR            = 5,
    XHCI_CC_STALL_ERROR          = 6,
    XHCI_CC_RESOURCE_ERROR       = 7,
    XHCI_CC_BANDWIDTH_ERROR      = 8,
    XHCI_CC_NO_SLOTS_AVAILABLE   = 9,
    XHCI_CC_INVALID_STREAM_TYPE  = 10,
    XHCI_CC_SLOT_NOT_ENABLED     = 11,
    XHCI_CC_EP_NOT_ENABLED       = 12,
    XHCI_CC_SHORT_PACKET         = 13,
    XHCI_CC_RING_UNDERRUN        = 14,
    XHCI_CC_RING_OVERRUN         = 15,
    XHCI_CC_PARAMETER_ERROR      = 17,
    XHCI_CC_CONTEXT_STATE_ERROR  = 19,
    XHCI_CC_EVENT_RING_FULL      = 21,
    XHCI_CC_COMMAND_ABORTED      = 24,
    XHCI_CC_STOPPED              = 26,
};

//
// Endpoint context types
//

#define XHCI_EP_TYPE_INVALID    0
#define XHCI_EP_TYPE_ISOCH_OUT  1
#define XHCI_EP_TYPE_BULK_OUT   2
#define XHCI_EP_TYPE_INTR_OUT   3
#define XHCI_EP_TYPE_CONTROL    4
#define XHCI_EP_TYPE_ISOCH_IN   5
#define XHCI_EP_TYPE_BULK_IN    6
#define XHCI_EP_TYPE_INTR_IN    7

// Endpoint ID encoding for doorbell / context
#define XHCI_EP_ID(ep, in)      ((((ep) & 0xf) << 1) | ((in) ? 1 : 0))
#define XHCI_EP0_ID             1   // control EP0 = doorbell target 1. ID0 reserved for slot context (general info about the device)

// 32-byte slot context
struct xhci_slot_ctx {
    uint32_t fields[8];
} __attribute__((packed));

// Slot context dword 0 fields
#define XHCI_SLOT_DW0_ROUTE_MASK        0x000fffffu // how to find a device through a chain of USB hubs
#define XHCI_SLOT_DW0_SPEED_SHIFT       20 // what speed is this device
#define XHCI_SLOT_DW0_SPEED_MASK        (0xfu << 20)
#define XHCI_SLOT_DW0_MTT               (1u << 25)
#define XHCI_SLOT_DW0_HUB               (1u << 26) // signifies that this device is actually a host with other devices plugged into it
#define XHCI_SLOT_DW0_CTX_ENTRIES_SHIFT 27
#define XHCI_SLOT_DW0_CTX_ENTRIES_MASK  (0x1fu << 27) // how many entrypoint contexts follow this slot context

// Slot context dword 1 fields
#define XHCI_SLOT_DW1_RH_PORT_SHIFT     16 // which USB port this device is plugged into
#define XHCI_SLOT_DW1_RH_PORT_MASK      (0xffu << 16)
#define XHCI_SLOT_DW1_NUM_PORTS_SHIFT   24 // if a HUB, how many ports it has
#define XHCI_SLOT_DW1_NUM_PORTS_MASK    (0xffu << 24)

// 32-byte endpoint context
struct xhci_ep_ctx {
    uint32_t fields[8];
} __attribute__((packed));

// Endpoint context dword 0 fields
// when and how to talk to the endpoint
#define XHCI_EP_DW0_EP_STATE_MASK       0x7u // Disabled, Running, Halted, or Error
#define XHCI_EP_DW0_MULT_SHIFT          8
#define XHCI_EP_DW0_MAX_PSTREAMS_SHIFT  10
#define XHCI_EP_DW0_LSA                 (1u << 15)
#define XHCI_EP_DW0_INTERVAL_SHIFT      16 // how often to poll the device

// Endpoint context dword 1 fields
// what kind of data is being moved and how large it is
#define XHCI_EP_DW1_CERR_SHIFT          1   // retry count
#define XHCI_EP_DW1_EP_TYPE_SHIFT       3   // endpoint context type
#define XHCI_EP_DW1_EP_TYPE_MASK        (0x7u << 3)
#define XHCI_EP_DW1_HID                 (1u << 7)
#define XHCI_EP_DW1_MAX_BURST_SHIFT     8
#define XHCI_EP_DW1_MAX_PKT_SIZE_SHIFT  16 // largest single packet size the endpoint can handle
#define XHCI_EP_DW1_MAX_PKT_SIZE_MASK   (0xffffu << 16)

// Max ESIT Payload — bytes per Endpoint Service Interval Time
#define XHCI_EP_DW4_MAX_ESIT_PAY_SHIFT  16

// Each endpoint has its own transfer ring. Tells the hardware that the command at this address is ready
#define XHCI_EP_DCS                     (1u << 0)


// Device context: 1 slot context + 31 endpoint contexts (EP0 + 1..15 in/out)
struct xhci_device_ctx {
    struct xhci_slot_ctx slot;
    struct xhci_ep_ctx   ep[31];
} __attribute__((packed));


// The Input Context as a whole is the input control context immediately
// followed by a Device Context.
// Pass this context to the controller to update hardware state
struct xhci_input_ctrl_ctx {
    uint32_t drop_flags;
    uint32_t add_flags;
    uint32_t rsvd[6];
} __attribute__((packed));

struct xhci_input_ctx {
    struct xhci_input_ctrl_ctx ctrl;
    struct xhci_device_ctx     device;
} __attribute__((packed));

// Convenient add/drop bits: bit 0 = slot, bit 1 = EP0, bit N+1 = EP N.
#define XHCI_INPUT_A0_SLOT      (1u << 0)
#define XHCI_INPUT_A_EP(epid)   (1u << (epid))

//
// Event Ring Segment Table entry
//

struct xhci_erst_entry {
    uint64_t base;          // segment base physical address (64-byte aligned)
    uint16_t size;          // number of TRBs in segment (16..4096) 
    uint16_t rsvd1;
    uint32_t rsvd2;
} __attribute__((packed));

//
// Software ring tracking
//

#define XHCI_RING_SIZE          256   // TRBs per ring segment 
#define XHCI_DCBAA_ALIGN        64
#define XHCI_RING_ALIGN         64
#define XHCI_CTX_ALIGN          64
#define XHCI_ERST_ALIGN         64

// keep track of driver progress
struct xhci_ring {
    struct xhci_trb *trbs;      // contiguous TRB array (RING_SIZE entries)
    uint64_t         trbs_phys; // physical address of trbs[0]
    uint32_t         enq;       // enqueue index (driver writes here)
    uint32_t         deq;       // dequeue index (events: HW advances)
    uint8_t          cycle;     // current Producer Cycle State
    uint8_t          rsvd[3];
};

// In-flight command tracking
// linked into hc->cmd_waiters
struct xhci_cmd_wait {
    struct list_head    node;
    uint64_t            cmd_trb_phys;       // physical addr of the issued command TRB
    volatile uint8_t    completed;          // 0 -> 1 when event arrives
    volatile uint8_t    completion_code;    // XHCI_CC_*
    volatile uint8_t    slot_id;            // for ENABLE_SLOT
    uint8_t             rsvd;
};

// In-flight transfer tracking
// linked into hc->xfer_waiters
struct xhci_xfer_wait {
    struct list_head    node;
    uint64_t            last_trb_phys;
    volatile uint8_t    completed;
    volatile uint8_t    completion_code;
    uint8_t             slot_id;
    uint8_t             ep_id;
    volatile uint32_t   residual_len;       // bytes NOT transferred
};

//
// Per-controller state.
//

struct xhci_hc {
    // MMIO regions. cap_base is the BAR mapping; the others are derived.
    void     *cap_base;     // capability registers (BAR0)
    void     *op_base;      // cap_base + CAPLENGTH
    void     *rt_base;      // cap_base + RTSOFF
    uint32_t *db_base;      // cap_base + DBOFF

    // Capabilities snapshot
    uint8_t  cap_length;
    uint8_t  context_size;  // 32 or 64 bytes per context
    uint8_t  max_slots;
    uint8_t  max_ports;
    uint16_t max_intrs;
    uint16_t hci_version;
    uint32_t hcc_params1;
    uint32_t hcs_params1;
    uint32_t hcs_params2;

    // Data structures (all physically contiguous, identity-mapped)
    uint64_t            *dcbaa;          // DCBAA[0..max_slots]. Device context base address array. master list of all connected devices
    uint64_t             dcbaa_phys;
    struct xhci_ring     cmd_ring;
    struct xhci_ring     event_ring;
    struct xhci_erst_entry *erst;        // event ring segment table
    uint64_t             erst_phys;
    uint32_t             erst_entries;

    // Optional scratchpad buffers (HCSPARAMS2.SPB_MAX > 0)
    uint64_t            *scratchpad_array;
    uint64_t             scratchpad_array_phys;
    uint32_t             scratchpad_count;

    // Slot tracking, indexed 1..max_slots (index 0 unused).
    struct xhci_device_ctx **device_ctxs;
    struct xhci_input_ctx  **input_ctxs;
    struct xhci_ring        *ep0_rings;     // EP0 transfer rings, one per slot
    // Non-EP0 transfer rings indexed by [slot_id * 32 + dci]. NULL = not allocated.
    struct xhci_ring       **ep_rings;
    struct usb_device      **usb_devices;   // HCI-agnostic per-slot handle

    // In-flight cmd / transfer waiters.
    // the drain matches each event against the appropriate list and wakes hc->waitq
    struct list_head     cmd_waiters;
    struct list_head     xfer_waiters;

    // Bitmap of ports whose reset has completed and need enumeration.
    // Bit N = port N.
    uint64_t            pending_port_enum;

    // Bitmap of ports whose device disconnected and need slot teardown.
    uint64_t            pending_port_disconnect;

    //Synchronization. hc->lock guards: cmd ring + per-EP transfer ring
    // enqueue state, event ring drain state, and both waiter lists
    spinlock_t           lock;

    // Woken by the IRQ handler's drain after each event batch is processed
    nk_wait_queue_t     *waitq;

    // Interrupt routing 
    int                  irq_vec;

    // Backing PCI device
    struct pci_dev      *pci_dev;

    // Linkage on the driver's controller list
    struct list_head     node;
};

//
// Public driver entry points
//

int xhci_pci_init(struct naut_info *naut);
int xhci_pci_deinit(void);

// exposed for the USB-layer transfer API. usb_control_transfer dispatches to this
int xhci_control_transfer(struct xhci_hc *hc, int slot_id,
                          uint8_t bmRequestType, uint8_t bRequest,
                          uint16_t wValue, uint16_t wIndex,
                          void *buf, uint16_t wLength);

// NORMAL TRB transfer on a non-EP0 endpoint.
// Used for bulk and interrupt
int xhci_normal_transfer(struct xhci_hc *hc, int slot_id, int dci,
                         void *buf, uint16_t length);

// ISOCH TRB transfer on a non-EP0 endpoint. Single-TRB, SIA-scheduled
int xhci_isoch_transfer(struct xhci_hc *hc, int slot_id, int dci,
                        void *buf, uint16_t length);

// Re-issue CONFIGURE_ENDPOINT with explicit drop/add endpoint lists.
// Used for setting up a new alt
int xhci_reconfigure_endpoints(struct xhci_hc *hc, int slot_id, uint8_t speed,
                               struct usb_endpoint *drop_eps, uint32_t drop_n,
                               struct usb_endpoint *add_eps,  uint32_t add_n);

// Halt-recovery primitives. Used by usb_clear_halt() to undo a STALL.
int xhci_reset_endpoint(struct xhci_hc *hc, int slot_id, int dci);
int xhci_set_tr_dequeue_ptr(struct xhci_hc *hc, int slot_id, int dci);

// Mark an enumerated slot as a USB hub. Sets HUB+MTT bits, NumPorts,
// and TT Think Time in the slot context via EVALUATE_CONTEXT so the
// controller knows to apply TT scheduling for descendants.
int xhci_evaluate_hub_context(struct xhci_hc *hc, int slot_id,
                              uint8_t num_ports, int mtt, uint8_t ttt);

// Enumerate a device discovered on `hub_port` of `hub_dev`. Caller has
// already verified PORT_CONNECTION and run PORT_RESET on the hub side.
// `speed` is decoded from the hub port status (FS/LS/HS).
int xhci_enumerate_hub_port(struct xhci_hc *hc, struct usb_device *hub_dev,
                            uint8_t hub_port, uint32_t speed);

// Compile-time size checks 
_Static_assert(sizeof(struct xhci_trb)            == 16, "xhci_trb must be 16 bytes");
_Static_assert(sizeof(struct xhci_slot_ctx)       == 32, "xhci_slot_ctx must be 32 bytes (32-byte context variant)");
_Static_assert(sizeof(struct xhci_ep_ctx)         == 32, "xhci_ep_ctx must be 32 bytes (32-byte context variant)");
_Static_assert(sizeof(struct xhci_device_ctx)     == 1024, "xhci_device_ctx must be 1024 bytes (32 + 31 * 32)");
_Static_assert(sizeof(struct xhci_input_ctrl_ctx) == 32, "xhci_input_ctrl_ctx must be 32 bytes");
_Static_assert(sizeof(struct xhci_input_ctx)      == 1056, "xhci_input_ctx must be 1056 bytes (32 + 1024)");
_Static_assert(sizeof(struct xhci_erst_entry)     == 16, "xhci_erst_entry must be 16 bytes");

#endif // __XHCI_H__
