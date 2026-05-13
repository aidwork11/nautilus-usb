#include <nautilus/nautilus.h>
#include <nautilus/cpu.h>
#include <nautilus/mm.h>
#include <nautilus/list.h>
#include <nautilus/spinlock.h>
#include <nautilus/naut_string.h>
#include <nautilus/idt.h>            // idt_find_and_reserve_range
#include <nautilus/irq.h>            // register_int_handler
#include <dev/apic.h>                // apic_do_eoi
#include <dev/pci.h>
#include <dev/xhci.h>


#ifndef NAUT_CONFIG_DEBUG_XHCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...)   INFO_PRINT("xhci: " fmt, ##args)
#define DEBUG(fmt, args...)  DEBUG_PRINT("xhci: " fmt, ##args)
#define ERROR(fmt, args...)  ERROR_PRINT("xhci: " fmt, ##args)


// Driver-global list of probed controllers.
static struct list_head xhci_controllers;
static int xhci_controllers_inited = 0;


//
// PCI Matching
//

static int
xhci_match(struct pci_dev *pdev) {
    return pdev->cfg.class_code == XHCI_PCI_CLASS &&
           pdev->cfg.subclass   == XHCI_PCI_SUBCLASS &&
           pdev->cfg.prog_if    == XHCI_PCI_PROGIF;
}


//
// Capability-register parse 
//

// Snapshot the capability registers
// derive the operational/runtime/doorbell
// register windows, and stash everything the rest of the driver needs int struct xhci_hc. 
// Capability registers are read-only

static int xhci_read_capabilities(struct xhci_hc *hc) {
    uint8_t *bar0 = (uint8_t *)hc->cap_base;

    uint32_t cap0 = xhci_readl(bar0);
    hc->cap_length  = cap0 & 0xff;
    hc->hci_version = (cap0 >> 16) & 0xffff;
    hc->hcs_params1 = xhci_readl(bar0 + XHCI_CAP_HCSPARAMS1);
    hc->hcs_params2 = xhci_readl(bar0 + XHCI_CAP_HCSPARAMS2);
    hc->hcc_params1 = xhci_readl(bar0 + XHCI_CAP_HCCPARAMS1);

    hc->max_slots    = XHCI_HCS1_MAXSLOTS(hc->hcs_params1);
    hc->max_intrs    = XHCI_HCS1_MAXINTRS(hc->hcs_params1);
    hc->max_ports    = XHCI_HCS1_MAXPORTS(hc->hcs_params1);
    hc->context_size = XHCI_HCC1_CSZ(hc->hcc_params1) ? 64 : 32;

    // DBOFF and RTSOFF are the byte offsets to the doorbell array and runtime register set, with reserved low bits
    uint32_t dboff  = xhci_readl(bar0 + XHCI_CAP_DBOFF)  & ~0x3u;
    uint32_t rtsoff = xhci_readl(bar0 + XHCI_CAP_RTSOFF) & ~0x1fu;

    if (hc->cap_length == 0 || hc->cap_length > 0x40) {
        ERROR("implausible CAPLENGTH=0x%x\n", hc->cap_length);
        return -1;
    }
    if (hc->max_slots == 0) {
        ERROR("controller reports MaxSlots=0; cannot use\n");
        return -1;
    }

    hc->op_base = bar0 + hc->cap_length; // operational regs
    hc->db_base = (uint32_t *)(bar0 + dboff); // doorbell regs
    hc->rt_base = bar0 + rtsoff; // runtime regs

    return 0;
}


//
// Polling helper
//
// Spin reading a 32-bit MMIO register until (val & mask) == (expected & mask),
// or until timeout_us microseconds have elapsed. Returns 0 on success, -1 on
// timeout.
//
static int xhci_poll_until(void *reg, uint32_t mask, uint32_t expected,
                           uint32_t timeout_us, const char *what) {
    while (timeout_us > 0) {
        uint32_t v = xhci_readl(reg);
        if ((v & mask) == (expected & mask)) {
            return 0;
        }
        udelay(1);
        timeout_us--;
    }
    ERROR("timeout waiting for %s (mask=0x%08x expected=0x%08x got=0x%08x)\n",
          what, mask, expected, xhci_readl(reg));
    return -1;
}


//
// Controller reset
//
//
//   1. Halt: clear USBCMD.RS, then poll USBSTS.HCH until it sets.
//   2. Reset: set USBCMD.HCRST. The bit self-clears when reset is done.
//      USBSTS.CNR must also clear before any other operational-register access.
//   3. Re-read capabilities
//
static int xhci_reset(struct xhci_hc *hc) {
    uint8_t *op = (uint8_t *)hc->op_base;

    // halt the controller if it isn't already.
    uint32_t cmd = xhci_readl(op + XHCI_OP_USBCMD);
    if (cmd & XHCI_CMD_RS) { // reset
        DEBUG("controller running; clearing USBCMD.RS\n");
        xhci_writel(op + XHCI_OP_USBCMD, cmd & ~XHCI_CMD_RS);
        if (xhci_poll_until(op + XHCI_OP_USBSTS, // status
                            XHCI_STS_HCH, XHCI_STS_HCH, // hch = halt
                            1000000, "USBSTS.HCH (halt)") < 0) {
            return -1;
        }
    } else {
        DEBUG("controller already halted (USBSTS.HCH=%u)\n",
              !!(xhci_readl(op + XHCI_OP_USBSTS) & XHCI_STS_HCH));
    }

    // assert HCRST and wait for the self-clear + CNR=0.
    cmd = xhci_readl(op + XHCI_OP_USBCMD);
    xhci_writel(op + XHCI_OP_USBCMD, cmd | XHCI_CMD_HCRST);

    if (xhci_poll_until(op + XHCI_OP_USBCMD,
                        XHCI_CMD_HCRST, 0,
                        1000000, "USBCMD.HCRST self-clear") < 0) { // spec says the hardware will flip this bit back to 0 once the internal reset is finished
        return -1;
    }
    if (xhci_poll_until(op + XHCI_OP_USBSTS,
                        XHCI_STS_CNR, 0, // controller not ready
                        1000000, "USBSTS.CNR (controller ready)") < 0) {
        return -1;
    }

    INFO("controller reset complete\n");

    // re-read capabilities.
    return xhci_read_capabilities(hc);
}


//
// Program CONFIG.MaxSlotsEn
// tells the controller how many device slots to enable.
// Must be done after reset and before the controller is started.
// We enable the maximum the controller advertises (HCSPARAMS1.MaxSlots).
//
static int xhci_set_max_slots(struct xhci_hc *hc) {
    uint8_t *op = (uint8_t *)hc->op_base; 

    // update the config to enable all available slots
    uint32_t config = xhci_readl(op + XHCI_OP_CONFIG);
    config = (config & ~XHCI_CONFIG_MAXSLOTSEN_MASK) |
             (hc->max_slots & XHCI_CONFIG_MAXSLOTSEN_MASK);
    xhci_writel(op + XHCI_OP_CONFIG, config);

    // Read back and confirm the write actually landed
    uint32_t after = xhci_readl(op + XHCI_OP_CONFIG) & XHCI_CONFIG_MAXSLOTSEN_MASK;
    if (after != hc->max_slots) {
        ERROR("MaxSlotsEn write did not stick (wrote=%u read=%u)\n",
              hc->max_slots, after);
        return -1;
    }
    INFO("CONFIG.MaxSlotsEn = %u\n", after);
    return 0;
}


//
// Allocate and program the Device Context Base Address Array
// Index 0 is scratchpad pointer or NULL
// slots 1..MaxSlots will point to per-device output contexts
//
static int xhci_setup_dcbaa(struct xhci_hc *hc) {
    size_t size = (hc->max_slots + 1) * sizeof(uint64_t); // +1 for SCB

    hc->dcbaa = kmem_mallocz(size);
    if (!hc->dcbaa) {
        ERROR("cannot allocate DCBAA (%lu bytes)\n", size);
        return -1;
    }
    
    hc->dcbaa_phys = (uint64_t)hc->dcbaa;

    if (hc->dcbaa_phys & (XHCI_DCBAA_ALIGN - 1)) {
        ERROR("DCBAA not %u-byte aligned (got 0x%lx)\n",
              XHCI_DCBAA_ALIGN, hc->dcbaa_phys);
        kmem_free(hc->dcbaa);
        hc->dcbaa = NULL;
        return -1;
    }

    uint8_t *op = (uint8_t *)hc->op_base;
    xhci_writeq(op + XHCI_OP_DCBAAP, hc->dcbaa_phys);

    INFO("DCBAA at phys=0x%lx (%lu bytes, %u entries)\n",
         hc->dcbaa_phys, size, hc->max_slots + 1);
    return 0;
}


//
// Allocate scratchpad buffers if the controller requires them
// Mem needed = CSPARAMS2.MaxScratchpadBufs * 4 KB pages
//
static int xhci_setup_scratchpads(struct xhci_hc *hc) {
    uint32_t spb = XHCI_HCS2_SPB_MAX(hc->hcs_params2);
    hc->scratchpad_count = spb;

    if (spb == 0) {
        DEBUG("no scratchpad buffers required\n");
        return 0;
    }

    // each array index is a pointer to the start of a page
    size_t array_size = spb * sizeof(uint64_t);
    hc->scratchpad_array = kmem_mallocz(array_size);
    if (!hc->scratchpad_array) {
        ERROR("cannot allocate scratchpad pointer array\n");
        return -1;
    }
    hc->scratchpad_array_phys = (uint64_t)hc->scratchpad_array;

    for (uint32_t i = 0; i < spb; i++) {
        void *page = kmem_mallocz(4096);
        if (!page) {
            ERROR("cannot allocate scratchpad %u\n", i);
            return -1;   // teardown handles partial cleanup
        }
        hc->scratchpad_array[i] = (uint64_t)page;
    }

    hc->dcbaa[0] = hc->scratchpad_array_phys;

    INFO("allocated %u scratchpad buffer(s), array at 0x%lx\n",
         spb, hc->scratchpad_array_phys);
    return 0;
}


//
// Initialize the command ring and program CRCR
// The last TRB is a LINK pointing back to TRB[0] with TC=1 (toggle cycle on traverse)
// CRCR holds the ring's physical address and the initial ring cycle state
//
static int xhci_init_cmd_ring(struct xhci_hc *hc) {
    size_t size = XHCI_RING_SIZE * sizeof(struct xhci_trb);

    hc->cmd_ring.trbs = kmem_mallocz(size);
    if (!hc->cmd_ring.trbs) {
        ERROR("cannot allocate command ring (%lu bytes)\n", size);
        return -1;
    }
    hc->cmd_ring.trbs_phys = (uint64_t)hc->cmd_ring.trbs;

    if (hc->cmd_ring.trbs_phys & (XHCI_RING_ALIGN - 1)) {
        ERROR("command ring not %u-byte aligned (got 0x%lx)\n",
              XHCI_RING_ALIGN, hc->cmd_ring.trbs_phys);
        kmem_free(hc->cmd_ring.trbs);
        hc->cmd_ring.trbs = NULL;
        return -1;
    }

    // Producer state
    hc->cmd_ring.enq   = 0;
    hc->cmd_ring.deq   = 0;
    hc->cmd_ring.cycle = 1;

    // Last TRB links back to TRB[0] with TC=1 so the controller's CCS
    // toggles each time it traverses the ring boundary.
    struct xhci_trb *link = &hc->cmd_ring.trbs[XHCI_RING_SIZE - 1];
    link->param   = hc->cmd_ring.trbs_phys;
    link->status  = 0;
    link->control = XHCI_TRB_TYPE(XHCI_TRB_LINK) | XHCI_TRB_TC;

    // mask off the alignement bits - only send base address
    uint8_t *op = (uint8_t *)hc->op_base;
    uint64_t crcr = (hc->cmd_ring.trbs_phys & ~0x3fULL) | XHCI_CRCR_RCS; // command ring control register. bottom bits are flags
    xhci_writeq(op + XHCI_OP_CRCR, crcr);

    INFO("command ring at phys=0x%lx (%u TRBs, %lu bytes)\n",
         hc->cmd_ring.trbs_phys, XHCI_RING_SIZE, size);
    return 0;
}

//
// Port management
//

static void xhci_drain_event_ring(struct xhci_hc *hc);
static int  xhci_enumerate_port(struct xhci_hc *hc, uint32_t port);
static void xhci_process_pending_enumerations(struct xhci_hc *hc);

// All RW1C bits in PORTSC - Mask these off in any read/modify/write so we
// don't clear them by accident.
// portsc = port status and control
#define XHCI_PORTSC_RW1C_ALL \
        (XHCI_PORTSC_W1C_MASK | XHCI_PORTSC_PED)

// math helper to get the status/control register for a specific port
static inline volatile uint32_t * xhci_portsc(struct xhci_hc *hc, uint32_t port) {
    return (volatile uint32_t *)((uint8_t *)hc->op_base + XHCI_OP_PORT_BASE
                                 + (port - 1) * XHCI_OP_PORT_STRIDE
                                 + XHCI_PORT_PORTSC);
}

static uint32_t xhci_port_read(struct xhci_hc *hc, uint32_t port) {
    return xhci_readl(xhci_portsc(hc, port));
}

static void xhci_port_write(struct xhci_hc *hc, uint32_t port, uint32_t val) {
    xhci_writel(xhci_portsc(hc, port), val & ~XHCI_PORTSC_RW1C_ALL);
}

// ACK the W1C change bits in `change_mask` by writing 1 to them.
static void xhci_port_clear_changes(struct xhci_hc *hc, uint32_t port, uint32_t change_mask) {
    uint32_t cur = xhci_port_read(hc, port);
    uint32_t v = (cur & ~XHCI_PORTSC_RW1C_ALL) | (change_mask & XHCI_PORTSC_W1C_MASK);
    xhci_writel(xhci_portsc(hc, port), v);
}

static int xhci_port_power_on(struct xhci_hc *hc, uint32_t port) {
    
    // if 0, the hardware handles power automatically without driver involvement
    // ppc = port power control
    if (!XHCI_HCC1_PPC(hc->hcc_params1)) {
        return 0;
    }

    // if port power is already on, return success
    uint32_t v = xhci_port_read(hc, port);
    if (v & XHCI_PORTSC_PP) {
        return 0;
    }
    
    xhci_port_write(hc, port, v | XHCI_PORTSC_PP);
    udelay(20000);   // ~20ms power-on debounce
    v = xhci_port_read(hc, port);
    if (!(v & XHCI_PORTSC_PP)) {
        ERROR("port %u failed to power on (PORTSC=0x%08x)\n", port, v);
        return -1;
    }
    DEBUG("port %u powered on (PORTSC=0x%08x)\n", port, v);
    return 0;
}

// Trigger a port reset by setting PORTSC.PR. 
// Asynchronous - no polling here.
static int xhci_port_reset(struct xhci_hc *hc, uint32_t port) {
    uint32_t v = xhci_port_read(hc, port);
    if (!(v & XHCI_PORTSC_CCS)) { // current connect status
        DEBUG("port %u: no device connected, skipping reset\n", port);
        return -1;
    }
    DEBUG("port %u: asserting PR (PORTSC=0x%08x)\n", port, v);
    xhci_port_write(hc, port, v | XHCI_PORTSC_PR);
    return 0;
}

// React to a PORT_STATUS_CHANGE_EVENT TRB
// 1. Decode the affected port from param[31:24]
// 2. read PORTSC and dispatch on the change bits that are set
// Each branch acks its own change bit so the interrupter doesn't keep
// re-firing on it.
static void xhci_handle_port_status_change(struct xhci_hc *hc, struct xhci_trb *trb) {
    
    uint32_t port = (trb->param >> 24) & 0xff;
    if (port < 1 || port > hc->max_ports) {
        ERROR("PORT_STATUS_CHANGE for invalid port %u\n", port);
        return;
    }
    uint32_t psc   = xhci_port_read(hc, port);
    uint32_t speed = (psc & XHCI_PORTSC_SPEED_MASK) >> XHCI_PORTSC_SPEED_SHIFT;

    INFO("port %u status change: PORTSC=0x%08x "
         "[CCS=%u PED=%u PR=%u CSC=%u PRC=%u speed=%u]\n",
         port, psc,
         !!(psc & XHCI_PORTSC_CCS), !!(psc & XHCI_PORTSC_PED),
         !!(psc & XHCI_PORTSC_PR),  !!(psc & XHCI_PORTSC_CSC),
         !!(psc & XHCI_PORTSC_PRC), speed);

    if (psc & XHCI_PORTSC_PRC) { // port reset change
        // reset finished; port is ready for slot assignment
        INFO("port %u: reset complete (speed=%u, PED=%u)\n",
             port, speed, !!(psc & XHCI_PORTSC_PED));
        xhci_port_clear_changes(hc, port, XHCI_PORTSC_PRC);
        // at this port to the bitmap. technically could have up to 255 ports, but realistically this will be fine
        if (port < 64) {
            hc->pending_port_enum |= (1ULL << port);
        }
    }

    if (psc & XHCI_PORTSC_CSC) { // connect status change
        if (psc & XHCI_PORTSC_CCS) { // current connect status
            INFO("port %u: device connected\n", port);
            xhci_port_clear_changes(hc, port, XHCI_PORTSC_CSC);
            xhci_port_reset(hc, port);
        } else {
            INFO("port %u: device disconnected\n", port);
            xhci_port_clear_changes(hc, port, XHCI_PORTSC_CSC);
        }
    }

    // Ack any other change bits we don't specifically handle so they don't keep firing
    uint32_t residual = psc & XHCI_PORTSC_W1C_MASK & ~(XHCI_PORTSC_CSC | XHCI_PORTSC_PRC);
    if (residual) {
        DEBUG("port %u: acking residual change bits 0x%08x\n", port, residual);
        xhci_port_clear_changes(hc, port, residual);
    }
}

// Walks the ring while the next TRB's cycle bit match the consumer cycle state
// writes ERDP at the end (with EHB=1) to ack the event
static void xhci_drain_event_ring(struct xhci_hc *hc) {
    struct xhci_ring *er = &hc->event_ring;
    uint8_t *ir = (uint8_t *)hc->rt_base + XHCI_RT_IR_BASE;
    int n = 0;

    while (1) {
        struct xhci_trb *trb = &er->trbs[er->deq]; // get current trb
        xhci_rmb(); // read memory barrier. cycle check must be last
        uint32_t ctrl = trb->control;
        if ((ctrl & XHCI_TRB_CYCLE) != er->cycle) {
            break;   // hardware hasn't filled this slot yet
        }

        uint32_t type = XHCI_TRB_GET_TYPE(ctrl);
        switch (type) {
        case XHCI_TRB_PORT_STATUS_CHANGE:
            xhci_handle_port_status_change(hc, trb);
            break;
        case XHCI_TRB_COMMAND_COMPLETION: {
            // a COMMAND_COMPLETE trb holds the phys address of its corresponding command trb in the param field
            uint8_t cc  = XHCI_TRB_GET_COMP(trb->status); // completion code
            uint8_t sid = (ctrl >> 24) & 0xff; // slot id
            DEBUG("event: command completion (cc=%u slot=%u trb=0x%lx)\n",
                  cc, sid, trb->param);
            volatile struct xhci_cmd_wait *w = hc->current_cmd;
            if (w && w->cmd_trb_phys == trb->param) { // if there is a command in flight and if the waiter is waiting for this specific command
                // the issuer is waiting on .completed. when it sees that going high, it reads the cc and sid
                w->completion_code = cc;
                w->slot_id         = sid;
                xhci_wmb();
                w->completed       = 1;
            }
            break;
        }
        case XHCI_TRB_TRANSFER_EVENT: {
            // control carries the slot id and ep id of the EP that produced the event
            uint8_t  cc       = XHCI_TRB_GET_COMP(trb->status);
            uint8_t  sid      = XHCI_TRB_GET_SLOT(ctrl);
            uint8_t  eid      = XHCI_TRB_GET_EP(ctrl);
            uint32_t residual = trb->status & 0xffffffu;
            DEBUG("event: transfer (cc=%u slot=%u ep=%u resid=%u trb=0x%lx)\n",
                  cc, sid, eid, residual, trb->param);
            volatile struct xhci_xfer_wait *w = hc->current_xfer;
            if (w && w->slot_id == sid && w->ep_id == eid) {
                w->completion_code = cc;
                w->residual_len    = residual;
                xhci_wmb();
                w->completed       = 1;
            }
            break;
        }
        default:
            DEBUG("event: type=%u (unhandled)\n", type);
            break;
        }

        er->deq++; // advance on the ring
        if (er->deq == XHCI_RING_SIZE) {
            er->deq = 0;
            er->cycle ^= 1;
        }
        n++;
    }

    // Update ERDP to current dequeue position
    uint64_t phys = er->trbs_phys + er->deq * sizeof(struct xhci_trb);
    // mask off the flags, then set event handler busy (W1C) to tell hardware that processing finished
    xhci_writeq(ir + XHCI_IR_ERDP, (phys & XHCI_ERDP_PTR_MASK) | XHCI_ERDP_EHB);

    if (n > 0) {
        DEBUG("drained %d event(s); deq=%u cycle=%u\n", n, er->deq, er->cycle);
    }
}

// Power on every root hub port and pick up any devices already connected when the controller started
static void xhci_scan_ports(struct xhci_hc *hc) {
    INFO("scanning %u root hub port(s)\n", hc->max_ports);
    for (uint32_t p = 1; p <= hc->max_ports; p++) {
        xhci_port_power_on(hc, p);

        uint32_t v = xhci_port_read(hc, p);
        DEBUG("port %u initial PORTSC=0x%08x\n", p, v);

        // Clear any stale change bits from before the controller reset
        uint32_t pending = v & XHCI_PORTSC_W1C_MASK;
        if (pending) {
            xhci_port_clear_changes(hc, p, pending);
        }

        if (v & XHCI_PORTSC_CCS) { // current connect status
            INFO("port %u: device already connected at startup\n", p);
            xhci_port_reset(hc, p);
            // Wait for the controller to finish the reset and post PRC.
            xhci_poll_until((void *)xhci_portsc(hc, p),
                            XHCI_PORTSC_PRC, XHCI_PORTSC_PRC,
                            100000, "PORTSC.PRC");
        }
    }

    xhci_drain_event_ring(hc);

    // now that drain has marked any reset-complete ports in
    // pending_port_enum, run their enumerations sequentially.
    // Each enumeration may trigger more events (command completions); a
    // final drain after picks them up.
    xhci_process_pending_enumerations(hc);
    xhci_drain_event_ring(hc);
}


//
// device enumeration up through input-context setup
//
// After a port reset completes, the controller has trained the link (done in hardware) and
// the port reports a valid speed. The driver then:
//   1) ENABLE_SLOT command -> controller picks a slot ID
//   2) allocate output device context, register in DCBAA[slot_id]
//   3) allocate input context + EP0 transfer ring, populate slot+EP0
//   TODO: ADDRESS_DEVICE, GET_DESCRIPTOR, SET_ADDRESS, ...
//

// USB spec mandates max packet sizes for EP0 based on bus speed
// Idea is two fold: 1) slower speed means more wire time + overhead from packet headers,
// 2) low speed devices tend to be cheaper and have smaller buffers anyways
// For non EP0 endpoints, the packet size can be different, but the speed sets an upper bound
static uint16_t xhci_default_ep0_pkt_size(uint32_t speed) {
    switch (speed) {
    case XHCI_SPEED_LS:  return 8;
    case XHCI_SPEED_FS:  return 8; // start at 8, but byte7 is bMaxPacketSize0 which could tell us to go up to 64 max
    case XHCI_SPEED_HS:  return 64;
    case XHCI_SPEED_SS:  return 512;
    case XHCI_SPEED_SSP: return 512;
    default:             return 8;
    }
}

// Decode bMaxPacketSize0into a literal byte count
// SS encodes it as an exponent, others report the literal size
static uint16_t xhci_decode_max_pkt0(uint32_t speed, uint8_t reported) {
    if (speed == XHCI_SPEED_SS || speed == XHCI_SPEED_SSP) {
        return (uint16_t)(1u << reported);
    }
    return reported;
}

// Enqueue one TRB on the command ring and return its physical address
// Handles LINK trbs.
static uint64_t xhci_cmd_enqueue(struct xhci_hc *hc, uint64_t param, uint32_t status, uint32_t control) {
    struct xhci_ring *r = &hc->cmd_ring;

    // if we reached the end of the ring, loop back around before writing anything
    if (r->enq == XHCI_RING_SIZE - 1) {
        struct xhci_trb *link = &r->trbs[XHCI_RING_SIZE - 1];
        uint32_t lc = (link->control & ~XHCI_TRB_CYCLE) | (r->cycle & 1); // isolate the existing cycle bit and OR in the current cycle bit
        xhci_wmb();
        link->control = lc;
        r->enq = 0;
        r->cycle ^= 1;
    }

    struct xhci_trb *trb = &r->trbs[r->enq];
    trb->param  = param;
    trb->status = status;
    // wmb so HW never sees a partially-written trb
    uint32_t ctl = (control & ~XHCI_TRB_CYCLE) | (r->cycle & 1);
    xhci_wmb();
    trb->control = ctl;

    uint64_t trb_phys = r->trbs_phys + r->enq * sizeof(struct xhci_trb);
    r->enq++;
    return trb_phys;
}

// Issue a command and wait for its COMMAND_COMPLETION_EVENT on the event ring
// On success, returns 0 and writes the slot id (if non-NULL) from the
// event's slot field.
static int xhci_run_command(struct xhci_hc *hc, uint64_t param, uint32_t status, uint32_t control, uint8_t *out_slot_id, const char *what) {
    struct xhci_cmd_wait wait;
    memset(&wait, 0, sizeof(wait));

    wait.cmd_trb_phys = xhci_cmd_enqueue(hc, param, status, control);
    hc->current_cmd = &wait;

    // Order: TRB write -> doorbell write. wmb prevents the doorbell from racing past the TRB store.
    xhci_wmb();
    xhci_writel(&hc->db_base[XHCI_DB_HOST], XHCI_DB_CMD_DOORBELL);

    int timeout_ms = 1000;
    while (!wait.completed && timeout_ms > 0) {
        xhci_drain_event_ring(hc); // this will resolve the waiter (single threaded)
        if (wait.completed) break;
        udelay(1000);
        timeout_ms--;
    }
    hc->current_cmd = NULL;

    if (!wait.completed) {
        ERROR("%s: timeout waiting for completion\n", what);
        return -1;
    }
    if (wait.completion_code != XHCI_CC_SUCCESS) {
        ERROR("%s: completion code %u\n", what, wait.completion_code);
        return -1;
    }
    if (out_slot_id) *out_slot_id = wait.slot_id;
    return 0;
}

// returns the slot ID that the controller picked for this device
static int xhci_enable_slot(struct xhci_hc *hc) {
    uint8_t slot_id = 0;
    if (xhci_run_command(hc, 0, 0, XHCI_TRB_TYPE(XHCI_TRB_ENABLE_SLOT), &slot_id, "ENABLE_SLOT") < 0) {
        return -1;
    }
    if (slot_id < 1 || slot_id > hc->max_slots) {
        ERROR("ENABLE_SLOT returned invalid slot id %u\n", slot_id);
        return -1;
    }
    INFO("ENABLE_SLOT -> slot %u\n", slot_id);
    return (int)slot_id;
}

static int xhci_alloc_device_ctx(struct xhci_hc *hc, int slot_id) {
    size_t size = 32 * hc->context_size;        // 1024 bytes for CSZ=0
    struct xhci_device_ctx *ctx = kmem_mallocz(size);
    if (!ctx) {
        ERROR("slot %d: cannot allocate device context (%lu bytes)\n",
              slot_id, (unsigned long)size);
        return -1;
    }
    if ((uint64_t)ctx & (XHCI_CTX_ALIGN - 1)) {
        ERROR("slot %d: device context not %u-byte aligned (got %p)\n",
              slot_id, XHCI_CTX_ALIGN, ctx);
        kmem_free(ctx);
        return -1;
    }

    hc->device_ctxs[slot_id] = ctx;
    hc->dcbaa[slot_id]       = (uint64_t)ctx;

    INFO("slot %d: device ctx at phys=0x%lx (%lu bytes)\n",
         slot_id, (uint64_t)ctx, (unsigned long)size);
    return 0;
}

// allocate the input context and the per-slot EP0 transfer
// ring, then fill in:
//   - input control: add_flags = slot | EP0
//   - slot context:  route=0, port, speed, ctx_entries=1
//   - EP0 context:   ep_type=Control, max_pkt by speed, TR deq ptr, CErr=3
static int xhci_alloc_input_ctx(struct xhci_hc *hc, int slot_id, uint32_t port, uint32_t speed) {
    // input context = 1 input control ctx + 1 slot context + ep0 context + 30 other endpoint contexts
    size_t in_size = 33 * hc->context_size;
    struct xhci_input_ctx *in = kmem_mallocz(in_size);
    if (!in) {
        ERROR("slot %d: cannot allocate input context\n", slot_id);
        return -1;
    }
    if ((uint64_t)in & (XHCI_CTX_ALIGN - 1)) {
        ERROR("slot %d: input context not %u-byte aligned (got %p)\n",
              slot_id, XHCI_CTX_ALIGN, in);
        kmem_free(in);
        return -1;
    }

    size_t ring_bytes = XHCI_RING_SIZE * sizeof(struct xhci_trb);
    struct xhci_trb *trbs = kmem_mallocz(ring_bytes);
    if (!trbs) {
        ERROR("slot %d: cannot allocate EP0 transfer ring\n", slot_id);
        kmem_free(in);
        return -1;
    }
    if ((uint64_t)trbs & (XHCI_RING_ALIGN - 1)) {
        ERROR("slot %d: EP0 ring not %u-byte aligned (got %p)\n",
              slot_id, XHCI_RING_ALIGN, trbs);
        kmem_free(trbs);
        kmem_free(in);
        return -1;
    }

    // LINK TRB
    struct xhci_trb *link = &trbs[XHCI_RING_SIZE - 1];
    link->param   = (uint64_t)trbs; // link back to the first slot
    link->status  = 0;
    link->control = XHCI_TRB_TYPE(XHCI_TRB_LINK) | XHCI_TRB_TC; // TC = toggle cycle

    struct xhci_ring *ring = &hc->ep0_rings[slot_id];
    ring->trbs      = trbs;
    ring->trbs_phys = (uint64_t)trbs;
    ring->enq       = 0;
    ring->deq       = 0;
    ring->cycle     = 1;

    in->ctrl.add_flags  = XHCI_INPUT_A0_SLOT | XHCI_INPUT_A_EP(XHCI_EP0_ID);
    in->ctrl.drop_flags = 0;

    // Slot context
    // route = 0 for now (TODO) as hubs arent supported
    // lots of other to-dos here
    in->device.slot.fields[0] = ((speed << XHCI_SLOT_DW0_SPEED_SHIFT) & XHCI_SLOT_DW0_SPEED_MASK) | ((1u << XHCI_SLOT_DW0_CTX_ENTRIES_SHIFT) & XHCI_SLOT_DW0_CTX_ENTRIES_MASK);
    in->device.slot.fields[1] = ((port  << XHCI_SLOT_DW1_RH_PORT_SHIFT) & XHCI_SLOT_DW1_RH_PORT_MASK);

    // EP0 context
    uint16_t max_pkt = xhci_default_ep0_pkt_size(speed);
    struct xhci_ep_ctx *ep0 = &in->device.ep[XHCI_EP0_ID - 1];
    ep0->fields[1] =
        (XHCI_EP_TYPE_CONTROL << XHCI_EP_DW1_EP_TYPE_SHIFT) |
        (3u                   << XHCI_EP_DW1_CERR_SHIFT) | // retry count on error
        ((uint32_t)max_pkt    << XHCI_EP_DW1_MAX_PKT_SIZE_SHIFT);

    uint64_t deq = ring->trbs_phys | XHCI_EP_DCS; // page aligned, so bottom bits are zeroed. OR in the cycle state
    ep0->fields[2] = (uint32_t)(deq & 0xffffffffu);
    ep0->fields[3] = (uint32_t)(deq >> 32);

    hc->input_ctxs[slot_id] = in;

    INFO("slot %d: input ctx at 0x%lx, EP0 ring at 0x%lx (max_pkt=%u, speed=%u)\n",
         slot_id, (uint64_t)in, ring->trbs_phys, max_pkt, speed);
    return 0;
}

// enumerate ports one at a time
static void xhci_process_pending_enumerations(struct xhci_hc *hc) {
    while (hc->pending_port_enum) {
        int bit = __builtin_ffsll((long long)hc->pending_port_enum) - 1;
        if (bit < 0) break;
        uint32_t port = (uint32_t)bit;
        hc->pending_port_enum &= ~(1ULL << bit);
        xhci_enumerate_port(hc, port);
    }
}

// BSR: Block set address request (give this device a unique USB address on the bus)
static int xhci_address_device(struct xhci_hc *hc, int slot_id, int bsr) {
    struct xhci_input_ctx *in = hc->input_ctxs[slot_id];
    if (!in) {
        ERROR("slot %d: ADDRESS_DEVICE without input context\n", slot_id);
        return -1;
    }

    // ADDRESS_DEVICE expects A0 (slot) and A1 (EP0) both set
    in->ctrl.add_flags  = XHCI_INPUT_A0_SLOT | XHCI_INPUT_A_EP(XHCI_EP0_ID);
    in->ctrl.drop_flags = 0;

    uint64_t param   = (uint64_t)in;
    uint32_t status  = 0;
    uint32_t control = XHCI_TRB_TYPE(XHCI_TRB_ADDRESS_DEVICE)
                     | XHCI_TRB_SLOT_ID(slot_id)
                     | (bsr ? XHCI_TRB_BSR : 0);

    if (xhci_run_command(hc, param, status, control, NULL, "ADDRESS_DEVICE") < 0) {
        return -1;
    }

    INFO("slot %d: ADDRESS_DEVICE (BSR=%d) complete\n", slot_id, bsr);
    return 0;
}

// only FS devices need this as others use the default max pkt size
static int xhci_evaluate_ep0_max_pkt(struct xhci_hc *hc, int slot_id,
                                     uint16_t new_max_pkt) {
    struct xhci_input_ctx *in = hc->input_ctxs[slot_id];
    if (!in) {
        ERROR("slot %d: EVALUATE_CONTEXT without input context\n", slot_id);
        return -1;
    }

    in->ctrl.add_flags  = XHCI_INPUT_A_EP(XHCI_EP0_ID);
    in->ctrl.drop_flags = 0;

    // Rewrite EP0 fields[1] with the new max_pkt; preserve EP_TYPE/CErr.
    struct xhci_ep_ctx *ep0 = &in->device.ep[XHCI_EP0_ID - 1];
    ep0->fields[1] =
        (XHCI_EP_TYPE_CONTROL  << XHCI_EP_DW1_EP_TYPE_SHIFT) |
        (3u                    << XHCI_EP_DW1_CERR_SHIFT) |
        ((uint32_t)new_max_pkt << XHCI_EP_DW1_MAX_PKT_SIZE_SHIFT);

    uint64_t param   = (uint64_t)in;
    uint32_t control = XHCI_TRB_TYPE(XHCI_TRB_EVALUATE_CONTEXT)
                     | XHCI_TRB_SLOT_ID(slot_id);

    if (xhci_run_command(hc, param, 0, control,
                         NULL, "EVALUATE_CONTEXT") < 0) {
        return -1;
    }
    INFO("slot %d: EVALUATE_CONTEXT updated EP0 max_pkt to %u\n",
         slot_id, new_max_pkt);
    return 0;
}


//
// control-transfer plumbing on EP0
//
// A control transfer is a 3-TRB transfer description on an EP transfer ring:
//   SETUP  - 8-byte USB setup packet, IDT=1 so the bytes live in the TRB itself
//   DATA   - optional, points to the IN/OUT data buffer
//   STATUS - zero-length opposite-direction stage, IOC=1 so we get one event
//

// The TD must live on a single contiguous segment, so we wrap
// if it wouldn't fit before the ring's LINK TRB.
static void xhci_ring_reserve(struct xhci_ring *r, uint32_t n_trbs) {
    uint32_t avail = (XHCI_RING_SIZE - 1) - r->enq;
    if (avail >= n_trbs) {
        return;
    }
    struct xhci_trb *link = &r->trbs[XHCI_RING_SIZE - 1];
    uint32_t lc = (link->control & ~XHCI_TRB_CYCLE) | (r->cycle & 1);
    xhci_wmb();
    link->control = lc;
    r->enq = 0;
    r->cycle ^= 1;
}

// Enqueue a TRB on an arbitrary transfer ring
// returns the physical address of the TRB just written
static uint64_t xhci_xfer_enqueue(struct xhci_ring *r, uint64_t param, uint32_t status, uint32_t control) {
    struct xhci_trb *trb = &r->trbs[r->enq];
    trb->param  = param;
    trb->status = status;
    uint32_t ctl = (control & ~XHCI_TRB_CYCLE) | (r->cycle & 1);
    xhci_wmb();
    trb->control = ctl;

    uint64_t trb_phys = r->trbs_phys + r->enq * sizeof(struct xhci_trb);
    r->enq++;
    return trb_phys;
}

// Issue one USB control transfer on a slot's EP0 ring and wait for its TRANSFER_EVENT
// Returns bytes actually transferred on success
static int xhci_control_transfer(struct xhci_hc *hc, int slot_id, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, void *buf, uint16_t wLength) {
    struct xhci_ring *r = &hc->ep0_rings[slot_id];
    int dir_in   = (bmRequestType & USB_DIR_IN) != 0;
    int has_data = wLength > 0;
    uint32_t n_trbs = has_data ? 3u : 2u; // control transfers have setup and status; data transfers have setup, data, status
    
    xhci_ring_reserve(r, n_trbs);

    // SETUP
    uint64_t setup_param =
          ((uint64_t)bmRequestType)
        | ((uint64_t)bRequest    << 8)
        | ((uint64_t)wValue      << 16)
        | ((uint64_t)wIndex      << 32)
        | ((uint64_t)wLength     << 48);
    uint32_t setup_status = 8;   // Transfer Length = 8 bytes
    uint32_t setup_trt    = !has_data ? XHCI_TRB_SETUP_TRT_NO_DATA
                                      : (dir_in ? XHCI_TRB_SETUP_TRT_IN
                                                : XHCI_TRB_SETUP_TRT_OUT);
    uint32_t setup_ctl = XHCI_TRB_TYPE(XHCI_TRB_SETUP_STAGE)
                       | XHCI_TRB_IDT
                       | setup_trt;
    xhci_xfer_enqueue(r, setup_param, setup_status, setup_ctl);

    // DATA
    if (has_data) {
        uint32_t data_ctl = XHCI_TRB_TYPE(XHCI_TRB_DATA_STAGE);
        if (dir_in) {
            // controller will write data into the buf. if !dir_in the controller reads from the buf
            data_ctl |= XHCI_TRB_DIR_IN;
        }
        xhci_xfer_enqueue(r, (uint64_t)buf, (uint32_t)wLength, data_ctl);
    }

    // STATUS
    // opposite direction of data or in if no data stage
    // zero length, IOC=1 so we get a single completion event.
    int status_dir_in = !dir_in || !has_data;
    uint32_t status_ctl = XHCI_TRB_TYPE(XHCI_TRB_STATUS_STAGE) | XHCI_TRB_IOC;
    if (status_dir_in) {
        status_ctl |= XHCI_TRB_DIR_IN;
    }

    struct xhci_xfer_wait wait;
    memset(&wait, 0, sizeof(wait));
    wait.last_trb_phys = xhci_xfer_enqueue(r, 0, 0, status_ctl);
    wait.slot_id       = (uint8_t)slot_id;
    wait.ep_id         = XHCI_EP0_ID;
    hc->current_xfer   = &wait;

    // wmb so the doorbell doesn't race past the in-flight TRB stores.
    xhci_wmb();
    xhci_writel(&hc->db_base[slot_id], XHCI_EP0_ID);

    int timeout_ms = 1000;
    while (!wait.completed && timeout_ms > 0) {
        xhci_drain_event_ring(hc);
        if (wait.completed) {
            break;
        }
        udelay(1000);
        timeout_ms--;
    }
    hc->current_xfer = NULL;

    if (!wait.completed) {
        ERROR("control xfer slot=%d: timeout waiting for completion\n", slot_id);
        return -1;
    }
    if (wait.completion_code != XHCI_CC_SUCCESS &&
        wait.completion_code != XHCI_CC_SHORT_PACKET) {
        ERROR("control xfer slot=%d: completion code %u\n",
              slot_id, wait.completion_code);
        return -1;
    }

    // residual = bytes the controller did not transfer
    return (int)wLength - (int)wait.residual_len;
}

static int xhci_get_device_descriptor(struct xhci_hc *hc, int slot_id, void *buf, uint16_t len) {
    int desc = xhci_control_transfer(hc, slot_id, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, (uint16_t)(USB_DT_DEVICE << 8) | 0, 0, buf, len);
    if (desc < 0) return -1;
    if (desc < (int)len) {
        DEBUG("slot %d: GET_DESCRIPTOR returned short (%d/%u)\n",
              slot_id, desc, len);
    }
    return desc;
}

// Generic GET_DESCRIPTOR for non-device descriptor types 
// wValue's high byte is the descriptor type, low byte is the descriptor index.
static int xhci_get_descriptor(struct xhci_hc *hc, int slot_id, uint8_t dt_type, uint8_t dt_index, void *buf, uint16_t len) {
    int desc = xhci_control_transfer(hc, slot_id, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, (uint16_t)(dt_type << 8) | dt_index, 0, buf, len);
    if (desc < 0) return -1;
    if (desc < (int)len) {
        DEBUG("slot %d: GET_DESCRIPTOR(type=%u idx=%u) short (%d/%u)\n",
              slot_id, dt_type, dt_index, desc, len);
    }
    return desc;
}

// Walk a configuration descriptor blob and log each interface and endpoint.
// Class-specific descriptors interleave; we skip them.
static void xhci_parse_config_descriptor(int slot_id, uint8_t *buf, uint16_t len) {
    static const char *xfer_names[] = { "control", "isoch", "bulk", "intr" };
    uint16_t off = 0;
    while (off + 2 <= len) {
        uint8_t bLength         = buf[off];
        uint8_t bDescriptorType = buf[off + 1];
        if (bLength < 2 || off + bLength > len) {
            DEBUG("slot %d: malformed descriptor at off=%u (len=%u, remaining=%u)\n",
                  slot_id, off, bLength, (unsigned)(len - off));
            break;
        }
        switch (bDescriptorType) {
        case USB_DT_CONFIGURATION:
            // Skip - the caller already logged the header before this walk.
            break;
        case USB_DT_INTERFACE: {
            struct usb_interface_descriptor *iface = (struct usb_interface_descriptor *)(buf + off);
            INFO("slot %d: iface %u alt %u: class=%u sub=%u proto=%u eps=%u\n",
                 slot_id,
                 iface->bInterfaceNumber, iface->bAlternateSetting,
                 iface->bInterfaceClass, iface->bInterfaceSubClass,
                 iface->bInterfaceProtocol, iface->bNumEndpoints);
            break;
        }
        case USB_DT_ENDPOINT: {
            struct usb_endpoint_descriptor *ep = (struct usb_endpoint_descriptor *)(buf + off);
            uint8_t xfer = ep->bmAttributes & USB_EP_XFER_MASK;
            INFO("slot %d:   ep%u %s %s max_pkt=%u interval=%u\n",
                 slot_id,
                 USB_EP_NUM(ep->bEndpointAddress),
                 USB_EP_DIR_IN(ep->bEndpointAddress) ? "IN " : "OUT", xfer_names[xfer], ep->wMaxPacketSize, ep->bInterval);
            break;
        }
        default:
            DEBUG("slot %d: skip descriptor type=%u len=%u\n",
                  slot_id, bDescriptorType, bLength);
            break;
        }
        off += bLength;
    }
}

static int xhci_enumerate_port(struct xhci_hc *hc, uint32_t port) {
    uint32_t psc = xhci_port_read(hc, port);
    uint32_t speed = (psc & XHCI_PORTSC_SPEED_MASK) >> XHCI_PORTSC_SPEED_SHIFT;
    if (speed == 0) {
        ERROR("port %u: speed=0 after reset, cannot enumerate\n", port);
        return -1;
    }

    int slot_id = xhci_enable_slot(hc);
    if (slot_id < 0) return -1;

    if (xhci_alloc_device_ctx(hc, slot_id) < 0) return -1;
    if (xhci_alloc_input_ctx(hc, slot_id, port, speed) < 0) return -1;

    // hand the input context to the controller
    if (xhci_address_device(hc, slot_id, 1) < 0) return -1;

    // read the first 8 bytes of the device descriptor - standard "discover bMaxPacketSize0" probe
    uint8_t *desc8 = kmem_mallocz(8);
    if (!desc8) {
        ERROR("slot %d: cannot allocate descriptor scratch\n", slot_id);
        return -1;
    }
    int desc = xhci_get_device_descriptor(hc, slot_id, desc8, 8);
    if (desc < 0) {
        kmem_free(desc8);
        return -1;
    }
    INFO("slot %d: dev desc (first %d B) bLength=%u type=%u bcdUSB=0x%04x "
         "class=%u sub=%u proto=%u maxpkt0=%u\n",
         slot_id, desc,
         desc8[0], desc8[1],
         (uint16_t)(desc8[2] | (desc8[3] << 8)),
         desc8[4], desc8[5], desc8[6], desc8[7]);
    uint8_t bMaxPacketSize0 = desc8[7];
    kmem_free(desc8);

    // if the device's bMaxPacketSize0 doesn't match the
    // default, push the corrected value into the EP0 context
    uint16_t reported = xhci_decode_max_pkt0(speed, bMaxPacketSize0);
    uint16_t current  = xhci_default_ep0_pkt_size(speed);
    if (reported != current && reported >= 8 && reported <= 1024) {
        DEBUG("slot %d: EP0 max_pkt mismatch (default=%u, device=%u); evaluating\n",
              slot_id, current, reported);
        if (xhci_evaluate_ep0_max_pkt(hc, slot_id, reported) < 0) {
            return -1;
        }
    }

    if (xhci_address_device(hc, slot_id, 0) < 0) return -1;

    // the controller writes the USB address into the output slot context
    uint8_t usb_addr = hc->device_ctxs[slot_id]->slot.fields[3] & 0xff;
    INFO("slot %d: addressed (USB addr=%u, speed=%u)\n", slot_id, usb_addr, speed);

    // now that the device is Addressed and EP0 max_pkt is
    // correct, read the full device descriptor for vendor/product
    // then walk the configuration descriptor for interface and
    // endpoint topology.

    struct usb_device_descriptor *dev_desc = kmem_mallocz(sizeof(struct usb_device_descriptor));
    if (!dev_desc) {
        ERROR("slot %d: cannot allocate device descriptor buf\n", slot_id);
        return -1;
    }
    int got = xhci_get_device_descriptor(hc, slot_id, dev_desc, sizeof(struct usb_device_descriptor));
    if (got < (int)sizeof(struct usb_device_descriptor)) {
        ERROR("slot %d: full device descriptor read returned %d\n", slot_id, got);
        kmem_free(dev_desc);
        return -1;
    }
    INFO("slot %d: vendor=0x%04x product=0x%04x bcdDev=0x%04x "
         "class=%u sub=%u proto=%u configs=%u\n",
         slot_id,
         dev_desc->idVendor, dev_desc->idProduct, dev_desc->bcdDevice,
         dev_desc->bDeviceClass, dev_desc->bDeviceSubClass,
         dev_desc->bDeviceProtocol, dev_desc->bNumConfigurations);
    uint16_t vendor  = dev_desc->idVendor;
    uint16_t product = dev_desc->idProduct;
    kmem_free(dev_desc);

    // Two-pass config descriptor read: header first (gives wTotalLength), then the full bundle
    struct usb_config_descriptor *cfg_hdr = kmem_mallocz(sizeof(struct usb_config_descriptor));
    if (!cfg_hdr) {
        ERROR("slot %d: cannot allocate config header buf\n", slot_id);
        return -1;
    }
    got = xhci_get_descriptor(hc, slot_id, USB_DT_CONFIGURATION, 0, cfg_hdr, sizeof(struct usb_config_descriptor));
    if (got < (int)sizeof(struct usb_config_descriptor)) {
        ERROR("slot %d: config header read returned %d\n", slot_id, got);
        kmem_free(cfg_hdr);
        return -1;
    }
    uint16_t total = cfg_hdr->wTotalLength;
    INFO("slot %d: config %u: %u iface(s), total=%u B, attr=0x%02x maxpower=%u mA\n",
         slot_id,
         cfg_hdr->bConfigurationValue, cfg_hdr->bNumInterfaces,
         total, cfg_hdr->bmAttributes, cfg_hdr->bMaxPower * 2);
    kmem_free(cfg_hdr);

    if (total < sizeof(struct usb_config_descriptor) || total > 4096) {
        ERROR("slot %d: implausible config total length %u\n", slot_id, total);
        return -1;
    }

    uint8_t *cfg = kmem_mallocz(total);
    if (!cfg) {
        ERROR("slot %d: cannot allocate %u-byte config buf\n", slot_id, total);
        return -1;
    }
    got = xhci_get_descriptor(hc, slot_id, USB_DT_CONFIGURATION, 0, cfg, total);
    if (got < (int)total) {
        ERROR("slot %d: full config read returned %d (expected %u)\n",
              slot_id, got, total);
        kmem_free(cfg);
        return -1;
    }
    xhci_parse_config_descriptor(slot_id, cfg, total);
    kmem_free(cfg);

    INFO("port %u enumerated as slot %d (vendor=0x%04x product=0x%04x); awaiting Phase 6 CONFIGURE_ENDPOINT\n",
         port, slot_id, vendor, product);
    return slot_id;
}


//
// MSI-X IRQ handler stub
//
// acknowledge the interrupt so the controller will fire again later; to be fleshed out later
//
//   1. Clear USBSTS.EINT (event interrupt) (W1C) -- controller-wide event indicator. something new has been placed on the event ring
//   2. Clear IMAN.IP (interrupt management - interrupt pending) (W1C) on the primary interrupter so a future event can raise the line again. each interruptor gets its own IMAN reg
//   3. Send EOI (end of interrupt) to the LAPIC.
//
static int xhci_irq_handler(excp_entry_t *e, excp_vec_t v, void *priv) {
    struct xhci_hc *hc = (struct xhci_hc *)priv;
    uint8_t *op = (uint8_t *)hc->op_base;
    uint8_t *ir = (uint8_t *)hc->rt_base + XHCI_RT_IR_BASE;

    DEBUG("IRQ vec=%lu USBSTS=0x%08x IMAN=0x%08x\n",
          (ulong_t)v,
          xhci_readl(op + XHCI_OP_USBSTS),
          xhci_readl(ir + XHCI_IR_IMAN));

    // Ack global event indicator.
    // EINT is W1C so writing causes the interrupt flag to go low
    xhci_writel(op + XHCI_OP_USBSTS, XHCI_STS_EINT);

    // Ack interrupter pending bit. Clear the pending bit, also W1C. An xHCI controller can have many interruptors so this like the local flag
    uint32_t iman = xhci_readl(ir + XHCI_IR_IMAN);
    xhci_writel(ir + XHCI_IR_IMAN, iman | XHCI_IMAN_IP);

    // Drain event ring, dispatch by TRB type, advance ERDP.
    xhci_drain_event_ring(hc);

    apic_do_eoi();
    return 0;
}


//
// Initialize the event ring and ERST
//
//   1. Allocate the ERST
//   2. Allocate the event ring TRB array (one segment, RING_SIZE TRBs).
//   3. Fill ERST[0] = { base = event_ring_phys, size = RING_SIZE }.
//   4. Program interrupter 0:
//        ERSTSZ = 1 (size)
//        ERDP   = event_ring_phys (dequeue pointer) tells the hardware where the driver is at on the ring
//        ERSTBA = erst_phys (base address) (must be written last as writing this tells the controller to start using the erst)
//        IMOD   = moderation interval (units of 250 ns; 4000 = ~1 ms).
//        IMAN   = interrupt enable bit set.
//
// no LINK TRB inside  the segment -- segments chain via the ERST itself.
//
static int xhci_setup_event_ring(struct xhci_hc *hc) {

    // ERST must be 64-byte aligned. kmem_mallocz only guarantees 16-byte
    // alignment, but the buddy allocator returns blocks aligned to their
    // (rounded-up) size, so request at least XHCI_ERST_ALIGN bytes.
    size_t erst_size = sizeof(struct xhci_erst_entry);
    if (erst_size < XHCI_ERST_ALIGN) erst_size = XHCI_ERST_ALIGN;
    hc->erst = kmem_mallocz(erst_size);
    if (!hc->erst) {
        ERROR("cannot allocate ERST\n");
        return -1;
    }
    hc->erst_phys    = (uint64_t)hc->erst;
    hc->erst_entries = 1;

    if (hc->erst_phys & (XHCI_ERST_ALIGN - 1)) {
        ERROR("ERST not %u-byte aligned (got 0x%lx)\n",
              XHCI_ERST_ALIGN, hc->erst_phys);
        kmem_free(hc->erst);
        hc->erst = NULL;
        return -1;
    }

    size_t ring_bytes = XHCI_RING_SIZE * sizeof(struct xhci_trb);
    hc->event_ring.trbs = kmem_mallocz(ring_bytes);
    if (!hc->event_ring.trbs) {
        ERROR("cannot allocate event ring (%lu bytes)\n", ring_bytes);
        return -1;
    }
    hc->event_ring.trbs_phys = (uint64_t)hc->event_ring.trbs;

    if (hc->event_ring.trbs_phys & (XHCI_RING_ALIGN - 1)) {
        ERROR("event ring not %u-byte aligned (got 0x%lx)\n",
              XHCI_RING_ALIGN, hc->event_ring.trbs_phys);
        return -1;
    }

    // The controller's producer cycle on the event ring starts at 1 after reset
    hc->event_ring.enq   = 0;
    hc->event_ring.deq   = 0;
    hc->event_ring.cycle = 1;

    // Fill ERST[0]
    hc->erst[0].base  = hc->event_ring.trbs_phys;
    hc->erst[0].size  = XHCI_RING_SIZE;
    hc->erst[0].rsvd1 = 0;
    hc->erst[0].rsvd2 = 0;

    // Program interrupter 0. Order: ERSTSZ -> ERDP -> ERSTBA last.
    uint8_t *ir = (uint8_t *)hc->rt_base + XHCI_RT_IR_BASE;

    xhci_writel(ir + XHCI_IR_ERSTSZ, hc->erst_entries);
    xhci_writeq(ir + XHCI_IR_ERDP,   hc->event_ring.trbs_phys);
    xhci_writeq(ir + XHCI_IR_ERSTBA, hc->erst_phys);

    // Moderation: ~1 ms between back-to-back interrupts (4000 * 250 ns).
    xhci_writel(ir + XHCI_IR_IMOD, 4000);

    // Enable the interrupter
    xhci_writel(ir + XHCI_IR_IMAN, XHCI_IMAN_IE);

    INFO("event ring at phys=0x%lx (%u TRBs); ERST at 0x%lx\n",
         hc->event_ring.trbs_phys, XHCI_RING_SIZE, hc->erst_phys);
    return 0;
}


//
// Workaround for NK's MSI-X auto-detect on 64-bit BARs
//
// pci_msi_x_detect (run during pci_init) bails out if the MSI-X table sits
// on a 64-bit memory BAR - pci_msi_x_get_bar_start only handles
// 32-bit BARs, so it returns NULL and the detect function never sets
// msix.type = PCI_MSI_X (even though it did find the capability
// and read its size). xHCI's BAR0 is 64-bit so we hit this on every probe.
//
// Wrote a workaround here to prevent messing with the PCI code.
// if the auto-detect found the capability (msix.co != 0) but failed to set the
// type, we re-read the table/PBA pointers using pci_dev_get_bar_addr 
// (which already handles 64-bit BARs), patch the four
// fields the auto-detect would have set, and proceed.
//
// Returns 0 if MSI-X is now usable, -1 otherwise.
//
static int xhci_fixup_msix(struct xhci_hc *hc) {
    struct pci_dev *pdev = hc->pci_dev;

    if (pdev->msix.type == PCI_MSI_X) {
        return 0;   // auto-detect already worked
    }
    if (pdev->msix.co == 0) {
        return -1;  // device truly has no MSI-X capability
    }

    // Re-read the MSI-X capability layout from PCI config space
    uint8_t co = pdev->msix.co;
    uint32_t table = pci_dev_cfg_readl(pdev, co + 4);
    uint32_t pba   = pci_dev_cfg_readl(pdev, co + 8);
    uint8_t  table_bar = table & 0x7;
    uint32_t table_off = table & ~0x7u;
    uint8_t  pba_bar   = pba & 0x7;
    uint32_t pba_off   = pba & ~0x7u;

    DEBUG("MSI-X re-detect: table BAR=%u off=0x%x, PBA BAR=%u off=0x%x\n",
          table_bar, table_off, pba_bar, pba_off);

    if (table_bar > 5 || pba_bar > 5) {
        ERROR("invalid MSI-X BAR indicators\n");
        return -1;
    }

    uint64_t table_phys = pci_dev_get_bar_addr(pdev, table_bar);
    uint64_t pba_phys   = pci_dev_get_bar_addr(pdev, pba_bar);
    if (!table_phys || !pba_phys) {
        ERROR("MSI-X table/PBA BAR maps to zero (table=0x%lx pba=0x%lx)\n",
              table_phys, pba_phys);
        return -1;
    }

    // Patch what pci_msi_x_detect should have written
    pdev->msix.table   = (pci_msi_x_table_entry_t *)(table_phys + table_off);
    pdev->msix.pending = (uint64_t *)(pba_phys + pba_off);
    pdev->msix.type    = PCI_MSI_X;

    DEBUG("MSI-X fixup: table at %p, pending at %p, %u entries\n",
          pdev->msix.table, pdev->msix.pending, pdev->msix.size);
    return 0;
}


//
// Allocate an MSI-X vector and route the primary interrupter to it
//
static int xhci_setup_irq(struct xhci_hc *hc) {
    if (xhci_fixup_msix(hc) != 0) {
        ERROR("device does not advertise usable MSI-X\n");
        return -1;
    }

    ulong_t vec; // the interrupt number
    if (idt_find_and_reserve_range(1, 0, &vec) != 0) {
        ERROR("cannot reserve an IDT vector\n");
        return -1;
    }
    hc->irq_vec = (int)vec;

    if (register_int_handler((uint16_t)vec, xhci_irq_handler, hc) != 0) {
        ERROR("register_int_handler(vec=%lu) failed\n", vec);
        return -1;
    }

    if (pci_dev_enable_msi_x(hc->pci_dev) != 0) {
        ERROR("pci_dev_enable_msi_x failed\n");
        return -1;
    }
    if (pci_dev_set_msi_x_entry(hc->pci_dev, 0, (int)vec, 0) != 0) {
        ERROR("pci_dev_set_msi_x_entry failed\n");
        return -1;
    }
    if (pci_dev_unmask_msi_x_entry(hc->pci_dev, 0) != 0) { // interrupt enable
        ERROR("pci_dev_unmask_msi_x_entry failed\n");
        return -1;
    }

    INFO("MSI-X entry 0 -> vector %lu, handler installed\n", vec);
    return 0;
}

//
// Start the controller
// after reset / DCBAA / rings / interrupter are all set up, flip USBCMD to start running.
// After RS=1 the controller clears USBSTS.HCH, indicating it is no longer
// halted. From that point, port-status changes start producing events.
//
static int xhci_start(struct xhci_hc *hc) {
    uint8_t *op = (uint8_t *)hc->op_base;

    uint32_t cmd = xhci_readl(op + XHCI_OP_USBCMD);
    cmd |= XHCI_CMD_RS | XHCI_CMD_INTE | XHCI_CMD_HSEE; // run/stop, interrupt enable, raise system error on faults
    xhci_writel(op + XHCI_OP_USBCMD, cmd);

    // HCH clears once the controller transitions out of the halted state
    if (xhci_poll_until(op + XHCI_OP_USBSTS,
                        XHCI_STS_HCH, 0,
                        1000000, "USBSTS.HCH (running)") < 0) {
        return -1;
    }

    INFO("controller running (USBCMD=0x%08x USBSTS=0x%08x)\n",
         xhci_readl(op + XHCI_OP_USBCMD),
         xhci_readl(op + XHCI_OP_USBSTS));
    return 0;
}

//
// Free anything xhci_init may have allocated. call after a partial init failure
//
static void xhci_teardown(struct xhci_hc *hc) {
    if (hc->event_ring.trbs) {
        kmem_free(hc->event_ring.trbs);
        hc->event_ring.trbs = NULL;
    }
    if (hc->erst) {
        kmem_free(hc->erst);
        hc->erst = NULL;
    }
    if (hc->cmd_ring.trbs) {
        kmem_free(hc->cmd_ring.trbs);
        hc->cmd_ring.trbs = NULL;
    }
    if (hc->scratchpad_array) {
        for (uint32_t i = 0; i < hc->scratchpad_count; i++) {
            if (hc->scratchpad_array[i]) {
                kmem_free((void *)hc->scratchpad_array[i]);
                hc->scratchpad_array[i] = 0;
            }
        }
        kmem_free(hc->scratchpad_array);
        hc->scratchpad_array = NULL;
    }
    if (hc->dcbaa) {
        kmem_free(hc->dcbaa);
        hc->dcbaa = NULL;
    }
    
    // per-slot device/input contexts and EP0 rings.
    if (hc->device_ctxs) {
        for (uint32_t i = 1; i <= hc->max_slots; i++) {
            if (hc->device_ctxs[i]) {
                kmem_free(hc->device_ctxs[i]);
                hc->device_ctxs[i] = NULL;
            }
        }
        kmem_free(hc->device_ctxs);
        hc->device_ctxs = NULL;
    }
    if (hc->input_ctxs) {
        for (uint32_t i = 1; i <= hc->max_slots; i++) {
            if (hc->input_ctxs[i]) {
                kmem_free(hc->input_ctxs[i]);
                hc->input_ctxs[i] = NULL;
            }
        }
        kmem_free(hc->input_ctxs);
        hc->input_ctxs = NULL;
    }
    if (hc->ep0_rings) {
        for (uint32_t i = 1; i <= hc->max_slots; i++) {
            if (hc->ep0_rings[i].trbs) {
                kmem_free(hc->ep0_rings[i].trbs);
                hc->ep0_rings[i].trbs = NULL;
            }
        }
        kmem_free(hc->ep0_rings);
        hc->ep0_rings = NULL;
    }
}


//
// Per-controller initialization
//

static int xhci_init(struct xhci_hc *hc) {

    if (xhci_read_capabilities(hc) < 0) {
        return -1;
    }

    INFO("xHCI %x.%x at cap=%p op=%p rt=%p db=%p\n",
         (hc->hci_version >> 8) & 0xff,
         hc->hci_version & 0xff,
         hc->cap_base, hc->op_base, hc->rt_base, hc->db_base);
    INFO("  caplen=%u max_slots=%u max_ports=%u max_intrs=%u ctx_size=%uB\n",
         hc->cap_length, hc->max_slots, hc->max_ports,
         hc->max_intrs, hc->context_size);
    DEBUG("  HCSPARAMS1=0x%08x HCSPARAMS2=0x%08x HCCPARAMS1=0x%08x\n",
          hc->hcs_params1, hc->hcs_params2, hc->hcc_params1);

    if (xhci_reset(hc) < 0) {
        return -1;
    }

    if (xhci_set_max_slots(hc) < 0) {
        return -1;
    }

    if (hc->context_size != 32) {
        ERROR("64-byte context size not supported (CSZ=1)\n");
        return -1;
    }
    size_t n = (size_t)hc->max_slots + 1;
    hc->device_ctxs = kmem_mallocz(n * sizeof(struct xhci_device_ctx *));
    hc->input_ctxs  = kmem_mallocz(n * sizeof(struct xhci_input_ctx *));
    hc->ep0_rings   = kmem_mallocz(n * sizeof(struct xhci_ring));
    if (!hc->device_ctxs || !hc->input_ctxs || !hc->ep0_rings) {
        ERROR("cannot allocate slot tracking arrays\n");
        goto fail;
    }

    if (xhci_setup_dcbaa(hc) < 0) {
        goto fail;
    }

    if (xhci_setup_scratchpads(hc) < 0) {
        goto fail;
    }

    if (xhci_init_cmd_ring(hc) < 0) {
        goto fail;
    }

    if (xhci_setup_event_ring(hc) < 0) {
        goto fail;
    }

    if (xhci_setup_irq(hc) < 0) {
        goto fail;
    }

    if (xhci_start(hc) < 0) {
        goto fail;
    }

    // Phase 4: power on root hub ports and pick up any devices that were
    // already plugged in when the controller started.
    xhci_scan_ports(hc);

    return 0;

fail:
    xhci_teardown(hc);
    return -1;
}


//
// PCI probe
// ensure that the hardware and the driver are compatible
//

static int xhci_probe(struct pci_dev *pdev, void *state) {

    if (!xhci_match(pdev)) {
        return 0;
    }

    INFO("found xHCI controller at %02x:%02x.%x (vendor=%04x device=%04x)\n",
         pdev->bus->num, pdev->num, pdev->fun,
         pdev->cfg.vendor_id, pdev->cfg.device_id);

    if (pci_dev_get_bar_type(pdev, 0) != PCI_BAR_MEM) {
        ERROR("BAR0 is not memory-mapped\n");
        return -1;
    }

    uint64_t bar_phys = pci_dev_get_bar_addr(pdev, 0);
    if (!bar_phys) {
        ERROR("BAR0 is unmapped (zero)\n");
        return -1;
    }
    uint64_t bar_size = pci_dev_get_bar_size(pdev, 0);
    DEBUG("BAR0 phys=0x%lx size=0x%lx\n", bar_phys, bar_size);

    struct xhci_hc *hc = malloc(sizeof(*hc));
    if (!hc) {
        ERROR("cannot allocate xhci_hc\n");
        return -1;
    }
    memset(hc, 0, sizeof(*hc));
    spinlock_init(&hc->lock);
    INIT_LIST_HEAD(&hc->node);
    hc->pci_dev  = pdev; // back pointer

    // the BAR physical address is directly usable as a kernel-virtual MMIO pointer
    hc->cap_base = (void *)bar_phys;

    // MMIO must be enabled to make xHCI respond to reads/writes on its BAR address
    // Bus Master is required for xHCI to initiate its own transfers on the PCI bus
    pci_dev_enable_mmio(pdev);
    pci_dev_enable_master(pdev);

    if (xhci_init(hc) < 0) {
        ERROR("controller init failed\n");
        pci_dev_disable_master(pdev);
        pci_dev_disable_mmio(pdev);
        free(hc);
        return -1;
    }

    list_add_tail(&hc->node, &xhci_controllers);
    return 0;
}


//
// Public entry points
//

int xhci_pci_init(struct naut_info *naut) {
    INFO("scanning PCI for xHCI controllers (class=0x%02x sub=0x%02x progif=0x%02x)\n",
         XHCI_PCI_CLASS, XHCI_PCI_SUBCLASS, XHCI_PCI_PROGIF);

    if (!xhci_controllers_inited) {
        INIT_LIST_HEAD(&xhci_controllers);
        xhci_controllers_inited = 1;
    }

    // pci_map_over_devices uses 0xffff as "match any" for both vendor and device
    int rc = pci_map_over_devices(xhci_probe, 0xffff, 0xffff, NULL);
    if (rc < 0) {
        ERROR("PCI scan failed\n");
        return -1;
    }

    if (list_empty(&xhci_controllers)) {
        INFO("no xHCI controllers found\n");
    }
    return 0;
}

int xhci_pci_deinit(void) {
    return 0;
}
