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
// Phase 4: Port management
//
// PORTSC mixes plain RW bits (PR, PP, PLS, ...) with RW1C "change" bits
// (CSC, PEC, PRC, ...). Any read/modify/write of PORTSC must mask off the
// change bits, otherwise we'd inadvertently clear them just by writing back
// what we read. PED is also RW1C-to-clear (writing 1 disables the port),
// so we treat it the same way and never echo it back unintentionally.
//

// All RW1C bits in PORTSC. Mask these off in any read/modify/write so we
// don't clear them by accident. To clear specific change bits, use
// xhci_port_clear_changes().
#define XHCI_PORTSC_RW1C_ALL \
        (XHCI_PORTSC_W1C_MASK | XHCI_PORTSC_PED)

// Pointer to PORTSC for `port` (1-indexed; valid range 1..max_ports).
static inline volatile uint32_t *
xhci_portsc(struct xhci_hc *hc, uint32_t port) {
    return (volatile uint32_t *)((uint8_t *)hc->op_base + XHCI_OP_PORT_BASE
                                 + (port - 1) * XHCI_OP_PORT_STRIDE
                                 + XHCI_PORT_PORTSC);
}

static uint32_t xhci_port_read(struct xhci_hc *hc, uint32_t port) {
    return xhci_readl(xhci_portsc(hc, port));
}

// Write `val` to PORTSC, defensively stripping RW1C bits so a stale read
// can't accidentally clear them.
static void xhci_port_write(struct xhci_hc *hc, uint32_t port, uint32_t val) {
    xhci_writel(xhci_portsc(hc, port), val & ~XHCI_PORTSC_RW1C_ALL);
}

// Acknowledge the W1C change bits in `change_mask` by writing 1 to them.
// Other RW1C bits get 0 (preserved). Plain RW bits keep their current
// values from the read-back.
static void xhci_port_clear_changes(struct xhci_hc *hc, uint32_t port,
                                    uint32_t change_mask) {
    uint32_t cur = xhci_port_read(hc, port);
    uint32_t v = (cur & ~XHCI_PORTSC_RW1C_ALL) |
                 (change_mask & XHCI_PORTSC_W1C_MASK);
    xhci_writel(xhci_portsc(hc, port), v);
}

// Power on a port if HCCPARAMS1.PPC=1 and PORTSC.PP is currently 0.
// On controllers without per-port power control, ports come up powered and
// this is a no-op.
static int xhci_port_power_on(struct xhci_hc *hc, uint32_t port) {
    if (!XHCI_HCC1_PPC(hc->hcc_params1)) {
        return 0;
    }
    uint32_t v = xhci_port_read(hc, port);
    if (v & XHCI_PORTSC_PP) {
        return 0;
    }
    xhci_port_write(hc, port, v | XHCI_PORTSC_PP);
    udelay(20000);   // ~20ms power-on debounce per USB 2.0 §7.1.7.3
    v = xhci_port_read(hc, port);
    if (!(v & XHCI_PORTSC_PP)) {
        ERROR("port %u failed to power on (PORTSC=0x%08x)\n", port, v);
        return -1;
    }
    DEBUG("port %u powered on (PORTSC=0x%08x)\n", port, v);
    return 0;
}

// Trigger a port reset by setting PORTSC.PR. The controller asserts PRC
// when reset completes; the IRQ handler picks that up via
// xhci_handle_port_status_change(). Asynchronous - we do not poll here.
static int xhci_port_reset(struct xhci_hc *hc, uint32_t port) {
    uint32_t v = xhci_port_read(hc, port);
    if (!(v & XHCI_PORTSC_CCS)) {
        DEBUG("port %u: no device connected, skipping reset\n", port);
        return -1;
    }
    DEBUG("port %u: asserting PR (PORTSC=0x%08x)\n", port, v);
    xhci_port_write(hc, port, v | XHCI_PORTSC_PR);
    return 0;
}

// React to a PORT_STATUS_CHANGE_EVENT TRB. Decode the affected port from
// param[31:24], read PORTSC, and dispatch on the change bits that are set.
// Each branch acks its own change bit so the interrupter doesn't keep
// re-firing on it.
static void xhci_handle_port_status_change(struct xhci_hc *hc,
                                           struct xhci_trb *trb) {
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

    if (psc & XHCI_PORTSC_PRC) {
        // reset finished; port is ready for slot assignment (Phase 5)
        INFO("port %u: reset complete (speed=%u, PED=%u)\n",
             port, speed, !!(psc & XHCI_PORTSC_PED));
        xhci_port_clear_changes(hc, port, XHCI_PORTSC_PRC);
        // TODO Phase 5: ENABLE_SLOT + ADDRESS_DEVICE here
    }

    if (psc & XHCI_PORTSC_CSC) {
        if (psc & XHCI_PORTSC_CCS) {
            INFO("port %u: device connected\n", port);
            xhci_port_clear_changes(hc, port, XHCI_PORTSC_CSC);
            // Issue reset; PRC event comes back through this handler
            xhci_port_reset(hc, port);
        } else {
            INFO("port %u: device disconnected\n", port);
            xhci_port_clear_changes(hc, port, XHCI_PORTSC_CSC);
        }
    }

    // Ack any other change bits we don't specifically handle (PEC, OCC,
    // PLC, WRC, CEC) so they don't keep firing.
    uint32_t residual = psc & XHCI_PORTSC_W1C_MASK
                            & ~(XHCI_PORTSC_CSC | XHCI_PORTSC_PRC);
    if (residual) {
        DEBUG("port %u: acking residual change bits 0x%08x\n", port, residual);
        xhci_port_clear_changes(hc, port, residual);
    }
}

// Drain all completed TRBs from the event ring. Walks the ring while the
// next TRB's cycle bit matches our consumer cycle state, dispatching by
// TRB type. Always writes ERDP at the end (with EHB=1) to ack the event
// handler busy bit, even if zero events were processed.
static void xhci_drain_event_ring(struct xhci_hc *hc) {
    struct xhci_ring *er = &hc->event_ring;
    uint8_t *ir = (uint8_t *)hc->rt_base + XHCI_RT_IR_BASE;
    int n = 0;

    while (1) {
        struct xhci_trb *trb = &er->trbs[er->deq];
        // The xHC writes the cycle bit last; the rmb keeps the compiler
        // from hoisting later trb->* loads above the cycle check.
        xhci_rmb();
        uint32_t ctrl = trb->control;
        if ((ctrl & XHCI_TRB_CYCLE) != er->cycle) {
            break;   // hardware hasn't filled this slot yet
        }

        uint32_t type = XHCI_TRB_GET_TYPE(ctrl);
        switch (type) {
        case XHCI_TRB_PORT_STATUS_CHANGE:
            xhci_handle_port_status_change(hc, trb);
            break;
        case XHCI_TRB_COMMAND_COMPLETION:
            DEBUG("event: command completion (cc=%u slot=%u)\n",
                  XHCI_TRB_GET_COMP(trb->status),
                  (ctrl >> 24) & 0xff);
            // TODO Phase 5: signal command waiters
            break;
        case XHCI_TRB_TRANSFER_EVENT:
            DEBUG("event: transfer (cc=%u)\n",
                  XHCI_TRB_GET_COMP(trb->status));
            // TODO Phase 5+: route to endpoint
            break;
        default:
            DEBUG("event: type=%u (unhandled)\n", type);
            break;
        }

        er->deq++;
        if (er->deq == XHCI_RING_SIZE) {
            er->deq = 0;
            er->cycle ^= 1;
        }
        n++;
    }

    // Update ERDP to current dequeue position; EHB=1 acks event handler
    // busy (W1C). DESI=0 because we have a single ERST segment.
    uint64_t phys = er->trbs_phys + er->deq * sizeof(struct xhci_trb);
    xhci_writeq(ir + XHCI_IR_ERDP,
                (phys & XHCI_ERDP_PTR_MASK) | XHCI_ERDP_EHB);

    if (n > 0) {
        DEBUG("drained %d event(s); deq=%u cycle=%u\n", n, er->deq, er->cycle);
    }
}

// Power on every root hub port and pick up any devices already connected
// when the controller started. Ports populated before USBCMD.RS=1 do
// generally fire a CSC event (QEMU does), but powering up explicitly and
// kicking reset on any pre-connected port makes startup deterministic.
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

        if (v & XHCI_PORTSC_CCS) {
            INFO("port %u: device already connected at startup\n", p);
            xhci_port_reset(hc, p);
            // Wait for the controller to finish the reset and post PRC.
            // USB 2 reset is ~10ms on the wire; cap at 100ms.
            xhci_poll_until((void *)xhci_portsc(hc, p),
                            XHCI_PORTSC_PRC, XHCI_PORTSC_PRC,
                            100000, "PORTSC.PRC");
        }
    }

    // Bring-up sanity check: until MSI-X delivery is wired up, drain any
    // PORT_STATUS_CHANGE events the resets above generated. Exercises
    // xhci_drain_event_ring and xhci_handle_port_status_change end-to-end.
    xhci_drain_event_ring(hc);
}


//
// MSI-X IRQ handler stub
//
// acknowledge the interrupt so the controller will fire again later 
// to be fleshed out later
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
