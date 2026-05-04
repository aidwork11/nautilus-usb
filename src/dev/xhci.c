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
// MSI-X IRQ handler stub
//
// Phase 3.8 only wires the interrupt path up; full event-ring drain lives in
// Phase 7. For now we just acknowledge the interrupt so the controller will
// fire again later (otherwise IMAN.IP stays asserted and we'd be stuck).
//
//   1. Clear USBSTS.EINT (W1C) -- controller-wide event indicator.
//   2. Clear IMAN.IP (W1C) on the primary interrupter so a future event can
//      raise the line again.
//   3. Send EOI to the LAPIC.
//
// Note: writing back the read value of IMAN clears IP (W1C) while leaving IE
// untouched (since IE is RW and was already 1).
//
static int xhci_irq_handler(excp_entry_t *e, excp_vec_t v, void *priv) {
    struct xhci_hc *hc = (struct xhci_hc *)priv;
    uint8_t *op = (uint8_t *)hc->op_base;
    uint8_t *ir = (uint8_t *)hc->rt_base + XHCI_RT_IR_BASE;

    // Ack global event indicator.
    xhci_writel(op + XHCI_OP_USBSTS, XHCI_STS_EINT);

    // Ack interrupter pending bit. Read-then-write keeps IE intact.
    uint32_t iman = xhci_readl(ir + XHCI_IR_IMAN);
    xhci_writel(ir + XHCI_IR_IMAN, iman | XHCI_IMAN_IP);

    // TODO Phase 7: drain event ring, dispatch by TRB type, advance ERDP.

    apic_do_eoi();
    return 0;
}


//
// Initialize the event ring and ERST (Event Ring Segment Table)
// xHCI spec § 4.9.4 / § 6.5.
//
//   1. Allocate the ERST -- one entry, 16 bytes, 64-byte aligned.
//   2. Allocate the event ring TRB array (one segment, RING_SIZE TRBs).
//   3. Fill ERST[0] = { base = event_ring_phys, size = RING_SIZE }.
//   4. Program interrupter 0:
//        ERSTSZ = 1
//        ERDP   = event_ring_phys
//        ERSTBA = erst_phys           <-- must be written LAST; this triggers
//                                         the controller to start using the
//                                         ERST.
//        IMOD   = moderation interval (units of 250 ns; 4000 = ~1 ms).
//        IMAN   = IE bit set.
//
// Unlike the command/transfer rings, the event ring has no LINK TRB inside
// the segment -- segments chain via the ERST itself.
//
static int xhci_setup_event_ring(struct xhci_hc *hc) {
    // ERST: at least 1 entry. We allocate room for one entry; buddy will
    // round to a power-of-2 block, which gives us the 64-byte alignment
    // the spec requires.
    size_t erst_size = sizeof(struct xhci_erst_entry);
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

    // Event ring TRB array (one segment).
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

    // Consumer state. The controller's producer cycle on the event ring
    // starts at 1 after reset, so we expect to see cycle=1 on valid events.
    hc->event_ring.enq   = 0;   // unused on event rings (HW is producer)
    hc->event_ring.deq   = 0;
    hc->event_ring.cycle = 1;

    // Fill ERST[0].
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

    // Enable the interrupter (IE=1, IP starts at 0).
    xhci_writel(ir + XHCI_IR_IMAN, XHCI_IMAN_IE);

    INFO("event ring at phys=0x%lx (%u TRBs); ERST at 0x%lx\n",
         hc->event_ring.trbs_phys, XHCI_RING_SIZE, hc->erst_phys);
    return 0;
}


//
// Workaround for NK's MSI-X auto-detect on 64-bit BARs
//
// pci_msi_x_detect (run during pci_init) bails out if the MSI-X table sits
// on a 64-bit memory BAR -- its helper pci_msi_x_get_bar_start only handles
// 32-bit BARs, so it returns NULL and the detect function never sets
// msix.type = PCI_MSI_X (even though it did successfully find the capability
// and read its size). xHCI's BAR0 is 64-bit per spec, and that's typically
// where the MSI-X table lives, so we hit this on every probe.
//
// To stay scoped to the xhci driver we don't touch pci.c. Instead, if the
// auto-detect found the capability (msix.co != 0) but failed to set the
// type, we re-read the table/PBA pointers ourselves using
// pci_dev_get_bar_addr (which already handles 64-bit BARs), patch the four
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

    // Re-read the MSI-X capability layout from PCI config space.
    //   co + 4: Table  - low 3 bits = BAR indicator, rest = byte offset
    //   co + 8: PBA    - same encoding
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

    // pci_dev_get_bar_addr handles 64-bit BARs correctly (it reads the
    // upper dword of the pair when needed and masks off the type bits).
    uint64_t table_phys = pci_dev_get_bar_addr(pdev, table_bar);
    uint64_t pba_phys   = pci_dev_get_bar_addr(pdev, pba_bar);
    if (!table_phys || !pba_phys) {
        ERROR("MSI-X table/PBA BAR maps to zero (table=0x%lx pba=0x%lx)\n",
              table_phys, pba_phys);
        return -1;
    }

    // Patch what pci_msi_x_detect should have written. Identity-mapped, so
    // physical == kernel-virtual.
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
//   1. Reserve 1 IDT vector via idt_find_and_reserve_range.
//   2. register_int_handler(vec, xhci_irq_handler, hc) -- so an early-firing
//      interrupt finds a real handler.
//   3. pci_dev_enable_msi_x(pdev) -- disables legacy IRQ, enables MSI-X
//      (with all entries still masked).
//   4. pci_dev_set_msi_x_entry(pdev, 0, vec, target_cpu=0).
//   5. pci_dev_unmask_msi_x_entry(pdev, 0) -- now interrupts can fire.
//
static int xhci_setup_irq(struct xhci_hc *hc) {
    if (xhci_fixup_msix(hc) != 0) {
        ERROR("device does not advertise usable MSI-X\n");
        return -1;
    }

    ulong_t vec;
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
    if (pci_dev_unmask_msi_x_entry(hc->pci_dev, 0) != 0) {
        ERROR("pci_dev_unmask_msi_x_entry failed\n");
        return -1;
    }

    INFO("MSI-X entry 0 -> vector %lu, handler installed\n", vec);
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
