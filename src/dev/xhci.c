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
    hc->hcs_params1 = xhci_readl(base + XHCI_CAP_HCSPARAMS1);
    hc->hcs_params2 = xhci_readl(base + XHCI_CAP_HCSPARAMS2);
    hc->hcc_params1 = xhci_readl(base + XHCI_CAP_HCCPARAMS1);

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

    return 0;
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
