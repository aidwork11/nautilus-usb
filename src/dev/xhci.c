#include <nautilus/nautilus.h>
#include <nautilus/cpu.h>
#include <nautilus/mm.h>
#include <nautilus/list.h>
#include <nautilus/spinlock.h>
#include <nautilus/naut_string.h>
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
// timeout. Used for xHCI register handshakes (USBSTS.HCH, USBCMD.HCRST,
// USBSTS.CNR).
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
// Allocate and program the DCBAA (Device Context Base Address Array)
// xHCI spec § 4.5: a (MaxSlots+1)-entry array of 64-bit physical pointers,
// 64-byte aligned. Index 0 is reserved (scratchpad pointer or NULL); slots
// 1..MaxSlots will point to per-device output contexts later.
//
static int xhci_setup_dcbaa(struct xhci_hc *hc) {
    size_t size = (hc->max_slots + 1) * sizeof(uint64_t);

    // kmem_mallocz returns a buddy block; for size > 64B it's at least
    // 64-byte aligned (alignment == roundup_pow_of_two(size) >= 64).
    hc->dcbaa = kmem_mallocz(size);
    if (!hc->dcbaa) {
        ERROR("cannot allocate DCBAA (%lu bytes)\n", size);
        return -1;
    }
    // NK heap is identity-mapped: kernel-virtual == physical.
    hc->dcbaa_phys = (uint64_t)hc->dcbaa;

    if (hc->dcbaa_phys & (XHCI_DCBAA_ALIGN - 1)) {
        ERROR("DCBAA not %u-byte aligned (got 0x%lx)\n",
              XHCI_DCBAA_ALIGN, hc->dcbaa_phys);
        kmem_free(hc->dcbaa);
        hc->dcbaa = NULL;
        return -1;
    }

    // DCBAAP is a 64-bit operational register.
    uint8_t *op = (uint8_t *)hc->op_base;
    xhci_writeq(op + XHCI_OP_DCBAAP, hc->dcbaa_phys);

    INFO("DCBAA at phys=0x%lx (%lu bytes, %u entries)\n",
         hc->dcbaa_phys, size, hc->max_slots + 1);
    return 0;
}


//
// Allocate scratchpad buffers if the controller requires them
// xHCI spec § 4.20: HCSPARAMS2.MaxScratchpadBufs encodes how many 4 KB pages
// of scratch memory the controller wants. If nonzero, allocate an array of
// physical page addresses and store its phys in DCBAA[0].
//
static int xhci_setup_scratchpads(struct xhci_hc *hc) {
    uint32_t spb = XHCI_HCS2_SPB_MAX(hc->hcs_params2);
    hc->scratchpad_count = spb;

    if (spb == 0) {
        DEBUG("no scratchpad buffers required\n");
        return 0;
    }

    // Pointer array: one uint64_t per scratchpad page.
    size_t array_size = spb * sizeof(uint64_t);
    hc->scratchpad_array = kmem_mallocz(array_size);
    if (!hc->scratchpad_array) {
        ERROR("cannot allocate scratchpad pointer array\n");
        return -1;
    }
    hc->scratchpad_array_phys = (uint64_t)hc->scratchpad_array;

    // One 4 KB page per scratchpad slot. Buddy gives 4 KB-aligned blocks
    // for size==4096 since alignment matches the order.
    for (uint32_t i = 0; i < spb; i++) {
        void *page = kmem_mallocz(4096);
        if (!page) {
            ERROR("cannot allocate scratchpad %u\n", i);
            return -1;   // teardown handles partial cleanup
        }
        hc->scratchpad_array[i] = (uint64_t)page;
    }

    // DCBAA[0] points at the scratchpad pointer array (per spec § 6.1).
    hc->dcbaa[0] = hc->scratchpad_array_phys;

    INFO("allocated %u scratchpad buffer(s), array at 0x%lx\n",
         spb, hc->scratchpad_array_phys);
    return 0;
}


//
// Initialize the command ring and program CRCR
// xHCI spec § 4.9.3: the command ring is a contiguous TRB array. The last
// TRB is a LINK pointing back to TRB[0] with TC=1 (toggle cycle on traverse).
// CRCR holds the ring's physical address and the initial Ring Cycle State.
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

    // Producer state. Cycle starts at 1 (matches the controller's initial
    // CCS, which we set via CRCR.RCS=1 below).
    hc->cmd_ring.enq   = 0;
    hc->cmd_ring.deq   = 0;
    hc->cmd_ring.cycle = 1;

    // Last TRB: LINK back to TRB[0] with TC=1 so the controller's CCS
    // toggles each time it traverses the ring boundary. Cycle bit on the
    // LINK itself starts at 0 (controller's initial CCS=1, so it won't
    // traverse here until the driver flips this LINK's cycle bit on the
    // first ring wrap).
    struct xhci_trb *link = &hc->cmd_ring.trbs[XHCI_RING_SIZE - 1];
    link->param   = hc->cmd_ring.trbs_phys;
    link->status  = 0;
    link->control = XHCI_TRB_TYPE(XHCI_TRB_LINK) | XHCI_TRB_TC;

    // CRCR is 64-bit. Bits 6..63 = ring base (already 64B aligned), bit 0 =
    // RCS (set to 1 so the controller's CCS comes up as 1).
    uint8_t *op = (uint8_t *)hc->op_base;
    uint64_t crcr = (hc->cmd_ring.trbs_phys & ~0x3fULL) | XHCI_CRCR_RCS;
    xhci_writeq(op + XHCI_OP_CRCR, crcr);

    INFO("command ring at phys=0x%lx (%u TRBs, %lu bytes)\n",
         hc->cmd_ring.trbs_phys, XHCI_RING_SIZE, size);
    return 0;
}


//
// Free anything xhci_init may have allocated. Safe to call after a partial
// init failure -- every free is conditional on the pointer being non-NULL.
//
static void xhci_teardown(struct xhci_hc *hc) {
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
