# xHCI Port Plan for Nautilus Kernel

## Overview

This document outlines the steps required to implement an xHCI (eXtensible Host Controller
Interface) USB host controller driver for Nautilus, along with a testing strategy. The
implementation follows the xHCI specification (Intel, rev 1.2) and uses the kanawha kernel's
`drivers/usb/xhci/` as a structural reference, adapted to Nautilus's APIs.

Target files:
- `include/dev/xhci.h` — register offsets, TRB/context structs, constants
- `src/dev/xhci.c` — driver implementation
- `include/dev/usb.h` — minimal USB device/transfer abstraction
- `src/dev/usb.c` — USB core (descriptor parsing, device enumeration)

---

## Phase 0: Prerequisites and Infrastructure

Before any xHCI code is written, verify and extend NK's infrastructure where needed.

### 0.1 Physically Contiguous Memory

xHCI rings (command, event, transfer) and device context arrays must be physically contiguous.
Verify that `kmem_malloc()` returns physically contiguous memory in NK's identity-mapped kernel
heap. If not, a simple physically-contiguous allocator (allocating from a reserved region at
boot) will be needed.

- Check: allocate a buffer, walk its pages, confirm physical addresses are contiguous
- If needed: reserve a DMA pool at boot (e.g. 4MB below the 4GB boundary) and implement
  `dma_alloc(size_t size)` / `dma_free(void *ptr)`

### 0.2 MMIO Barrier Macros

Define read/write barrier macros for MMIO access. The xHCI spec requires that certain register
sequences not be reordered by the compiler or CPU.

```c
#define xhci_readl(addr)       (*((volatile uint32_t *)(addr)))
#define xhci_readq(addr)       (*((volatile uint64_t *)(addr)))
#define xhci_writel(addr, v)   ((*((volatile uint32_t *)(addr))) = (v))
#define xhci_writeq(addr, v)   ((*((volatile uint64_t *)(addr))) = (v))
#define xhci_mb()              __asm__ volatile("mfence" ::: "memory")
```

### 0.3 Virtual Interrupt Vector Allocation

MSI-X requires allocating free interrupt vectors. Confirm that NK has a mechanism for
allocating vectors (or pick fixed ones for initial bring-up and convert later).

---

## Phase 1: Register Layout and Data Structures (`include/dev/xhci.h`)

Define all register offsets and structures before writing any logic.

### 1.1 Capability Registers (base + 0x0)

```
CAPLENGTH       0x00    (8-bit)  — byte length of capability register set
HCIVERSION      0x02    (16-bit) — xHCI spec version (e.g. 0x0110 = 1.1)
HCSPARAMS1      0x04    — MaxSlots, MaxIntrs, MaxPorts
HCSPARAMS2      0x08    — IST, ERST max size, scratchpad count
HCSPARAMS3      0x0C    — U1/U2 exit latencies
HCCPARAMS1      0x10    — 64-bit addressing, context size (32/64 byte), etc.
DBOFF           0x14    — doorbell array offset from base
RTSOFF          0x18    — runtime register space offset from base
HCCPARAMS2      0x1C
```

### 1.2 Operational Registers (base + CAPLENGTH)

```
USBCMD          0x00    — run/stop, host reset, interrupter enable
USBSTS          0x04    — halted, event interrupt, port change detect
PAGESIZE        0x08    — supported page sizes (bit N = 2^(N+12) bytes)
DNCTRL          0x14    — device notification control
CRCR            0x18    — command ring control (64-bit)
DCBAAP          0x30    — device context base address array pointer (64-bit)
CONFIG          0x38    — MaxSlotsEn
PORT_BASE       0x400   — per-port registers (PORTSC, PORTPMSC, PORTLI, PORTHLPMC)
                          each port: 4 × 32-bit registers, stride 0x10
```

### 1.3 Runtime Registers (base + RTSOFF)

```
MFINDEX         0x00    — microframe index
IR_BASE         0x20    — interrupter register sets (stride 0x20 each)
  IMAN          +0x00   — interrupt management (enable, pending)
  IMOD          +0x04   — interrupt moderation
  ERSTSZ        +0x08   — event ring segment table size
  ERSTBA        +0x10   — event ring segment table base address (64-bit)
  ERDP          +0x18   — event ring dequeue pointer (64-bit)
```

### 1.4 Doorbell Registers (base + DBOFF)

Array of 256 × 32-bit registers. Index 0 = host controller (command ring). Index 1–255 = device
slots. DB value encodes target endpoint + stream ID.

### 1.5 TRB Structures (16 bytes each)

All TRBs share a common layout:

```c
struct xhci_trb {
    uint64_t param;     // address or data field
    uint32_t status;    // transfer length, completion code, etc.
    uint32_t control;   // TRB type (bits 10:15), flags, cycle bit (bit 0)
};
```

Define TRB type constants (LINK, ENABLE_SLOT, ADDRESS_DEVICE, CONFIGURE_ENDPOINT,
EVALUATE_CONTEXT, TRANSFER_NORMAL, TRANSFER_SETUP, TRANSFER_DATA, TRANSFER_STATUS,
PORT_STATUS_CHANGE_EVENT, COMMAND_COMPLETION_EVENT, TRANSFER_EVENT, etc.).

### 1.6 Context Structures

The xHCI spec defines 32-byte or 64-byte contexts (selected by CSZ bit in HCCPARAMS1).

```c
struct xhci_slot_ctx    { uint32_t fields[8];  };   // 32-byte variant
struct xhci_ep_ctx      { uint32_t fields[8];  };
struct xhci_device_ctx  {
    struct xhci_slot_ctx slot;
    struct xhci_ep_ctx   ep[31];   // EP0 in/out + EP1..15 in/out
};
struct xhci_input_ctx   {
    uint32_t             drop_flags;
    uint32_t             add_flags;
    uint32_t             rsvd[6];
    struct xhci_device_ctx device;
};
```

### 1.7 Ring Structures

```c
#define XHCI_RING_SIZE  256   // TRBs per ring segment

struct xhci_ring {
    struct xhci_trb *trbs;      // physically contiguous array of RING_SIZE TRBs
    uint32_t        enq;        // enqueue index
    uint32_t        deq;        // dequeue index (event ring only)
    uint8_t         cycle;      // current cycle state bit
};
```

The last TRB in each segment must be a LINK TRB pointing back to index 0 (for command/transfer
rings) or a link to the next segment (for event rings, which are segmented).

### 1.8 Per-Controller State

```c
struct xhci_hc {
    // MMIO regions
    void    *cap_base;      // capability registers
    void    *op_base;       // operational registers
    void    *rt_base;       // runtime registers
    uint32_t *db_base;      // doorbell array

    // Capabilities
    uint8_t  cap_length;
    uint8_t  context_size;  // 32 or 64 bytes
    uint8_t  max_slots;
    uint8_t  max_ports;
    uint16_t max_intrs;

    // Data structures
    uint64_t           *dcbaa;          // device context base address array
    struct xhci_ring    cmd_ring;
    struct xhci_ring    event_ring;
    struct xhci_trb    *erst_table;     // event ring segment table

    // Slot tracking
    struct xhci_device_ctx **device_ctxs;   // indexed by slot ID (1-based)
    struct xhci_input_ctx  **input_ctxs;

    // Synchronization
    // (NK uses spinlocks; add appropriate lock here)

    struct pci_dev *pci_dev;
};
```

---

## Phase 2: PCI Probe and MMIO Setup (`src/dev/xhci.c`)

### 2.1 PCI Device Discovery

xHCI PCI class: `class=0x0C`, `subclass=0x03`, `progif=0x30`.

```c
static int xhci_probe(struct pci_dev *dev, void *state) {
    // verify class/subclass/progif
    // allocate struct xhci_hc
    // call xhci_init(hc, dev)
    return 0;
}

int xhci_pci_init(struct naut_info *naut) {
    pci_map_over_devices(xhci_probe, /* vendor= */ -1, /* device= */ -1, NULL);
    return 0;
}
```

Register `xhci_pci_init` in `src/arch/hrt/init.c` after `pci_init()`.

### 2.2 BAR Mapping

xHCI uses a single 64-bit MMIO BAR (BAR 0). Read its address with `pci_dev_get_bar_addr()`.
The identity-mapped kernel virtual address space means no explicit `ioremap` is needed — use
the physical address directly.

### 2.3 Enable Bus Mastering and MMIO

Set the PCI command register: enable Memory Space (bit 1) and Bus Master (bit 2).

---

## Phase 3: Host Controller Initialization

### 3.1 Read Capabilities

Parse `CAPLENGTH`, `HCSPARAMS1` (MaxSlots, MaxPorts), `HCCPARAMS1` (context size, 64-bit
capability), `DBOFF`, `RTSOFF`.

### 3.2 Reset the Controller

1. Clear `USBCMD.RS` (run/stop = 0); poll `USBSTS.HCH` until set (controller halted).
2. Set `USBCMD.HCRST` = 1; poll until `USBCMD.HCRST` = 0 and `USBSTS.CNR` = 0.
3. After reset: re-read capabilities (some controllers change values after reset).

Timeout after ~1 second; return error if reset doesn't complete.

### 3.3 Program MaxSlotsEn

Write the number of device slots to enable into `CONFIG.MaxSlotsEn` (≤ `MaxSlots` from
HCSPARAMS1).

### 3.4 Allocate and Program the DCBAA

Allocate `(MaxSlots + 1) × 8` bytes, physically contiguous, 64-byte aligned, zeroed.
Write the physical address to `DCBAAP`. Index 0 is reserved (scratchpad buffer pointer if
scratchpads are needed; otherwise NULL).

### 3.5 Allocate Scratchpad Buffers (if required)

HCSPARAMS2 encodes `Max Scratchpad Buffers`. If nonzero: allocate an array of physical page
addresses and point `DCBAA[0]` at it.

### 3.6 Initialize the Command Ring

Allocate `RING_SIZE` TRBs (physically contiguous). Set the last TRB as a LINK TRB with TC=1
(toggle cycle). Write the physical base address + cycle bit to `CRCR`.

### 3.7 Initialize the Event Ring

1. Allocate the Event Ring Segment Table (ERST): at least one entry, each entry is a 64-bit
   base address + 16-bit size.
2. Allocate the event ring TRB array for that segment.
3. Write `ERSTSZ` = number of segments.
4. Write `ERSTBA` = physical address of the ERST.
5. Write `ERDP` = physical address of the first TRB in the ring.
6. Enable the interrupter: set `IMAN.IE` = 1, write an `IMOD` value (e.g. 4000 for 1ms
   moderation).

### 3.8 Register the Interrupt Handler

Use MSI-X (preferred) or MSI. Allocate one vector for the primary interrupter.

```c
pci_dev_enable_msi_x(dev);
pci_dev_set_msi_x_entry(dev, 0, vec, target_cpu);
pci_dev_unmask_msi_x_entry(dev, 0);
register_int_handler(vec, xhci_irq_handler, hc);
```

### 3.9 Start the Controller

Set `USBCMD.RS` = 1, `USBCMD.INTE` = 1 (interrupter enable), `USBCMD.HSEE` = 1.
Poll `USBSTS.HCH` until it clears (controller running).

---

## Phase 4: Port Management

### 4.1 Port Status Change Detection

On a port status change event (TRB type `PORT_STATUS_CHANGE_EVENT`), read `PORTSC` for the
affected port. Key PORTSC bits:

- `CCS` (bit 0) — current connect status
- `PED` (bit 1) — port enabled
- `OCA` (bit 3) — overcurrent active
- `PR`  (bit 4) — port reset
- `PLS` (bits 8:5) — port link state
- `PP`  (bit 9) — port power
- `PRC` (bit 21) — port reset change (W1C)
- `CSC` (bit 17) — connect status change (W1C)

### 4.2 Port Reset

For a USB 3 port with `CCS=1`: write `PORTSC.PR=1`. Wait for `PRC` interrupt. Then the port is
ready for slot assignment. For USB 2 ports: write `PORTSC.PR=1`, wait for `PRC`, then proceed.

Clear change bits by writing 1 to them (W1C).

---

## Phase 5: USB Device Enumeration

### 5.1 Enable Slot (ENABLE_SLOT command)

Issue an ENABLE_SLOT TRB to the command ring. Ring doorbell 0. Wait for
`COMMAND_COMPLETION_EVENT` with `CompletionCode = Success`. Extract the slot ID from the event
TRB.

**Ringing the command ring doorbell:** write 0 to `db_base[0]`.

### 5.2 Allocate Output Device Context

Allocate a zeroed `struct xhci_device_ctx` (physically contiguous). Store its physical address
in `DCBAA[slot_id]`.

### 5.3 Allocate Input Context and Initialize EP0

Allocate a zeroed `struct xhci_input_ctx`. Set:
- `add_flags` = bit 0 (slot) | bit 1 (EP0)
- Slot context: `route_string=0`, `root_hub_port_number`, `context_entries=1`,
  `speed` (from port speed field in PORTSC)
- EP0 context: `ep_type=4` (Control), `max_packet_size` (8/64/512 depending on speed),
  `tr_dequeue_pointer` = physical base of EP0 transfer ring | cycle bit, `error_count=3`

### 5.4 Allocate EP0 Transfer Ring

Allocate `RING_SIZE` TRBs for EP0. Set up the LINK TRB at the end.

### 5.5 ADDRESS_DEVICE Command

Issue ADDRESS_DEVICE TRB with BSR=0 (block set address request = false for the second call;
true for the first call to get port speed). Input context physical address goes in the TRB
param field. Ring doorbell 0. Wait for completion event.

### 5.6 GET_DESCRIPTOR (Device Descriptor)

Issue a USB control transfer on EP0:
1. SETUP TRB: `bmRequestType=0x80`, `bRequest=6` (GET_DESCRIPTOR), `wValue=0x0100`
   (device descriptor), `wIndex=0`, `wLength=18`
2. DATA TRB (IN): pointer to 18-byte buffer, length=18
3. STATUS TRB (OUT, direction bit set): zero length

Ring the device doorbell (`db_base[slot_id]`, target = EP0 = 1).
Wait for TRANSFER_EVENT for each TRB. Parse the returned `struct usb_device_descriptor` for
`idVendor`, `idProduct`, `bDeviceClass`, `bNumConfigurations`.

### 5.7 SET_ADDRESS

After the first GET_DESCRIPTOR (8 bytes to read bMaxPacketSize0), issue ADDRESS_DEVICE with
BSR=0 to assign the USB address. The controller handles the SET_ADDRESS request automatically.

### 5.8 GET_DESCRIPTOR (Configuration Descriptor)

Repeat control transfer for configuration descriptor (`wValue=0x0200`). Parse to find interface
descriptors, endpoint descriptors. Configure additional endpoints via CONFIGURE_ENDPOINT command.

---

## Phase 6: Minimal USB Core (`include/dev/usb.h`, `src/dev/usb.c`)

A thin USB layer above xHCI to allow future HCI drivers (EHCI, OHCI) and class drivers.

### 6.1 Device Abstraction

```c
struct usb_device {
    uint8_t  slot_id;
    uint8_t  address;
    uint8_t  speed;         // USB_SPEED_LS/FS/HS/SS
    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t  dev_class;
    struct xhci_hc *hc;
    // endpoint info, config descriptor cache
};
```

### 6.2 Transfer Interface

```c
int usb_control_transfer(struct usb_device *dev,
                         uint8_t request_type, uint8_t request,
                         uint16_t value, uint16_t index,
                         void *data, uint16_t length);

int usb_bulk_transfer(struct usb_device *dev, uint8_t ep,
                      void *data, size_t length, int dir);
```

Control transfer is fully implemented (delegates to `xhci_control_transfer`
on EP0). Bulk transfer is stubbed and returns -1 until 6.4 lands —
implementing it requires CONFIGURE_ENDPOINT and per-endpoint transfer rings.

A composite `usb_get_descriptor(dev, type, index, buf, len)` is layered on
top of `usb_control_transfer` so class drivers can read descriptors without
re-building the SETUP packet.

### 6.3 Device Class Hooks

A simple table of `(class, subclass, protocol) -> probe()` callbacks. After
each `usb_register_device`, the USB core walks the table, finds the first
matching entry (matching at the *interface* level — `bDeviceClass=0` is the
common "see-interfaces" sentinel), and invokes the probe. The probe can
read additional descriptors via `usb_get_descriptor`, bind a per-device
state pointer to the `usb_device`, and either return success (driver bound)
or failure (try next match).

```c
struct usb_driver {
    uint8_t  match_class;       // 0xff = wildcard
    uint8_t  match_subclass;    // 0xff = wildcard
    uint8_t  match_protocol;    // 0xff = wildcard
    const char *name;
    int (*probe)(struct usb_device *dev);
};

int usb_driver_register(const struct usb_driver *drv);
```

Class drivers are static (compiled in for now); a registration call at boot
adds them to the table. `usb_register_device` becomes the binding trigger.

### 6.4 Bulk and Interrupt Endpoints

Lifting the 6.2 stub. Requires:

1. **`xhci_alloc_ep_ring`** — allocate a transfer ring for one non-EP0
   endpoint and install it in the slot's input context.
2. **`xhci_configure_endpoint`** — issue the CONFIGURE_ENDPOINT command
   so the controller copies the new EP contexts into the output device
   context and transitions the slot to Configured.
3. **`xhci_normal_transfer`** — enqueue one NORMAL TRB on a non-EP0
   transfer ring, ring the doorbell with target=EP DCI.
4. **Endpoint cache on `usb_device`** — parsed table mapping `(ep_num, dir)`
   to DCI and transfer ring, populated by walking the configuration
   descriptor during enumeration.
5. **`usb_bulk_transfer`** and **`usb_interrupt_transfer`** — both call
   `xhci_normal_transfer` underneath; controller's EP_TYPE setting picks
   the wire-level behavior (best-effort bulk vs polled interrupt).

Note: 6.4 must also issue a `SET_CONFIGURATION` control transfer on the
device before CONFIGURE_ENDPOINT, so the device-side endpoint state moves
to active.

### 6.5 Isochronous Transfers — **implemented; descriptor path validated, transfer path not yet**

Structurally distinct from bulk/interrupt: uses `XHCI_TRB_ISOCH` (not
NORMAL), no retries, and needs different EP-context fields than
bulk/intr. Three deltas from `xhci_normal_transfer`:

1. **TRB type and scheduling.** `xhci_isoch_transfer` emits a TRB of
   type `XHCI_TRB_ISOCH (5)` with the `SIA` bit set ("Start Isoch
   ASAP"). With SIA=1 we don't pin a frame_id — the controller picks
   the next available microframe. Frame-id scheduling is the obvious
   extension for an isoch driver that needs precise A/V sync.

2. **CErr=0 in the EP context.** `xhci_init_ep_ctx` now reads the
   endpoint's transfer type and sets the CErr (retry count) field
   accordingly: 0 for isoch (spec mandate — isoch has no retries) and
   3 for everything else. Without this fix CONFIGURE_ENDPOINT would
   reject an isoch endpoint with `PARAMETER_ERROR`.

3. **Max ESIT Payload in DW4.** EP context dword 4 used to hold only
   Average TRB Length in its low 16 bits. The high 16 bits — Max ESIT
   Payload — were left zero. For periodic endpoints (isoch + intr) the
   controller uses this field for bandwidth reservation at
   CONFIGURE_ENDPOINT time; leaving it zero on real hardware would
   either be rejected or silently allocate no bandwidth. We set it to
   `max_packet * (mult+1) * (burst+1)` for periodic endpoints, with
   `mult=burst=0` by default (so it collapses to `max_packet`). A
   future SuperSpeed Endpoint Companion parser would refine the
   multipliers; for FS/HS single-burst endpoints the simple form is
   correct.

The same patch added `usb_isoch_transfer` to the USB core, mirroring
`usb_bulk_transfer` / `usb_interrupt_transfer`. Validation shape is
identical; dispatch goes to `xhci_isoch_transfer` instead of
`xhci_normal_transfer` since the underlying TRB type differs.

#### What's been validated

Adding `-device usb-audio,bus=xhci.0` to the QEMU smoke test confirms
that QEMU's emulated USB Audio Class 1.0 device enumerates as a
Full-Speed class=1 device on slot 3, and the config-descriptor walker
correctly identifies its isoch endpoint:

```
slot 3:   ep1 OUT isoch max_pkt=192 interval=1
```

So the **transfer-type decoding, attribute parsing, and EP-context
plumbing** for isoch are exercised on every smoke-test run. The
existing MSC and HID paths continue to pass with the audio device
attached — no regression from the EP-context CErr / Max ESIT Payload
changes.

#### What's not yet validated

The actual `xhci_isoch_transfer` / `usb_isoch_transfer` call path is
still untested end-to-end. The blocker is structural, not driver-side:
USB Audio Class declares its isoch endpoint on **interface 1,
alternate setting 1**, while interface 1 alternate setting 0 has zero
endpoints (the spec's "zero-bandwidth default" pattern that lets
plug-and-play coexist with audio playback). Two pieces of work are
needed to reach the endpoint:

1. **Multi-interface endpoint capture.** Today
   `xhci_parse_config_descriptor` only stores the first interface's
   endpoints — interface 1's endpoint is logged but discarded. The
   parser needs to either capture all interfaces or accept a
   target-interface argument from the caller.

2. **SET_INTERFACE control request.** Switching the audio device from
   alt 0 (no endpoints) to alt 1 (isoch endpoint active) requires
   issuing `SET_INTERFACE(intf=1, alt=1)`. This is a one-line standard
   request; the harder part is re-running CONFIGURE_ENDPOINT after the
   switch to install the newly-active endpoint into the slot's
   context.

Once those two land, the isoch path can be exercised by walking
`usb_for_each_device` for class=1 devices, doing the alt switch, and
calling `usb_isoch_transfer` with a small buffer.

#### Beyond-QEMU caveats

Even with the QEMU end-to-end test, real-hardware issues likely lurk
in:

- **SuperSpeed bandwidth reservation.** SS isoch needs Mult and
  MaxBurst from the Endpoint Companion descriptor, which we don't
  parse. Today we default them to 0; an SS isoch device will likely
  request more bandwidth than that allocates.
- **Frame scheduling.** SIA=1 hides the question of "when?" — fine for
  best-effort audio playback, not fine for tight A/V sync where the
  caller wants a specific frame.
- **Underrun / overrun handling.** Completion codes `RING_UNDERRUN`
  and `RING_OVERRUN` are routine for isoch (the device couldn't keep
  up for one frame) and aren't fatal. We currently log them as errors;
  a real audio driver would log + continue.

---

## Phase 7: Synchronization and Event Handling

### 7.1 IRQ Handler — **done**

Implemented in `xhci_irq_handler` (src/dev/xhci.c). On every MSI-X interrupt:

1. Acks `USBSTS.EINT` (W1C) — the controller-wide event indicator.
2. Acks `IMAN.IP` (W1C) on interrupter 0 — the per-interrupter pending flag.
3. Calls `xhci_drain_event_ring()` — which takes `hc->lock`, dequeues every TRB
   whose cycle matches the driver's consumer cycle, matches each completion
   against the waiter lists, advances ERDP, releases the lock, and wakes
   `hc->waitq` if any waiter was matched.
4. Calls `apic_do_eoi()` to release the LAPIC.

No softirq offload or per-CPU steering — fine for QEMU's emulated controller.

**Caveat:** in QEMU we observe MSI-X interrupts arriving inconsistently for this
controller's vector. The wait path in 7.2 handles this by also re-draining on
its periodic wakeups, so completions never get stuck behind a missed interrupt.
The IRQ-driven path stays correct on hardware where MSI-X delivery is reliable.

### 7.2 Command / Transfer Synchronization — **done**

Waiters live in two lists hanging off `struct xhci_hc`:

```c
struct list_head cmd_waiters;     // xhci_cmd_wait nodes
struct list_head xfer_waiters;    // xhci_xfer_wait nodes
```

Each waiter is stack-allocated by its issuer, linked into the appropriate list
under `hc->lock`, and matched by the drain code:

- commands by `cmd_trb_phys` (the COMMAND_COMPLETION event's param field),
- transfers by `(slot_id, ep_id, last_trb_phys)`.

The match key uses the last TRB's physical address so concurrent TDs on the
same endpoint never collide.

Issuers run a common helper, `xhci_wait_completion(hc, &wait.completed,
timeout_ms)`, which uses `nk_wait_queue_sleep_extended_multiple` on
`(hc->waitq, per-thread timer waitq)`. Wake paths:

- **IRQ-driven:** `xhci_drain_event_ring` from `xhci_irq_handler` sets
  `wait.completed`, releases the lock, and calls `nk_wait_queue_wake_all`. The
  sleeping issuer's cond-check sees the flag and the multi-wait returns.
- **Timer fallback:** if no IRQ fires within a slice (currently 20 ms), the
  timer's waitq wakes the issuer; it explicitly drains the ring, re-checks,
  and sleeps again. This bounds latency to `slice_ms` even when MSI-X delivery
  is broken.
- **Timeout:** the total elapsed time across slices caps each call at 1 s
  (commands, EP0) or 5 s (bulk/intr).

After waking, the issuer takes `hc->lock` to unlink its waiter — necessary
because a late-arriving IRQ might otherwise touch a freed stack frame.

Locking discipline:
- `hc->lock` guards the cmd ring's enq/cycle, every transfer ring's enq/cycle,
  the event ring's deq/cycle, and both waiter lists.
- Issuers take it `spin_lock_irq_save` before enqueueing a TRB + linking the
  waiter + ringing the doorbell, and release it before going to sleep.
- The IRQ handler also goes through `spin_lock_irq_save` (no-op from interrupt
  context but cheap and correct); `nk_wait_queue_wake_all` is called outside
  the lock to minimize hold time.

### 7.3 Event Ring Advance — **done**

`xhci_drain_locked()` walks events while the cycle bit matches the driver's
expected state, processes each, then writes ERDP at the end:

```c
uint64_t phys = er->trbs_phys + er->deq * sizeof(struct xhci_trb);
xhci_writeq(ir + XHCI_IR_ERDP, (phys & XHCI_ERDP_PTR_MASK) | XHCI_ERDP_EHB);
```

`XHCI_ERDP_EHB` ("Event Handler Busy") is W1C — writing 1 acknowledges to the
controller that the driver has caught up. The controller can then re-arm
interrupts. Wraparound at `XHCI_RING_SIZE` toggles the driver's cycle bit.

### Boot-time bootstrap

`xhci_pci_init` starts a fallback `idle` thread bound to CPU 0 before probing.
Without it, the boot thread (the only runnable thread on CPU 0 at that point)
would call `nk_sched_sleep` from `xhci_wait_completion` and panic the scheduler
with "APERIODIC QUEUE IS EMPTY". The test harness (`nk_run_tests`) uses the
same trick.

### Current state summary

| Sub-phase | Status | Notes                                                              |
|-----------|--------|--------------------------------------------------------------------|
| 7.1       | done   | Drain-and-wake handler; no softirq offload                         |
| 7.2       | done   | Sleep + IRQ-wake + 20ms re-drain fallback; multi-in-flight via list|
| 7.3       | done   | ERDP advance with EHB ack on every drain                           |

Mass storage and other class drivers that want concurrent endpoints (e.g.
bulk-IN and bulk-OUT in flight simultaneously) can now do so safely.

---

## Phase 8: Kconfig and Makefile Integration

### 8.1 Kconfig

Add to `Kconfig` (or the relevant drivers Kconfig):

```
config XHCI
    bool "xHCI USB Host Controller driver"
    depends on PCI
    help
      Enables the xHCI (USB 3.x) host controller driver.

config USB
    bool "USB core"
    depends on XHCI
```

### 8.2 Makefile

Add to `src/dev/Makefile` (or equivalent):

```makefile
obj-$(CONFIG_XHCI) += xhci.o
obj-$(CONFIG_USB)  += usb.o
```

---

## Testing Strategy

### T1: Static Analysis / Build Verification

- Compile with `-Wall -Wextra`; zero warnings policy
- Check struct sizes with `static_assert` for 16-byte TRBs, correct context sizes
- Verify all register offset macros against the xHCI 1.2 spec table

### T2: QEMU Emulated xHCI

QEMU emulates xHCI via `-device qemu-xhci`. This is the primary development target.
NK boots from an ISO image (built via `make isoimage`), so we use `-cdrom` rather than
`-kernel` and capture serial to a file we can grep after the run.

#### Build

```sh
make isoimage          # builds nautilus.bin and produces nautilus.iso
```

If make doesn't pick up source changes (the linker step or ISO step gets skipped),
delete the bin/iso and rebuild:

```sh
rm -f nautilus.bin nautilus.iso && make isoimage
```

#### Run in QEMU

```sh
# Pre-step: kill any lingering QEMU instance that might still hold the test image lock
pkill -9 qemu-system-x86_64 2>/dev/null; sleep 1

# Reset the test files
rm -f /tmp/xhci.log /tmp/usbtest.img && truncate -s 16M /tmp/usbtest.img

# Run with the standard "USB 3 mouse + USB 2 storage" device set we use for Phase 4/5
qemu-system-x86_64 \
  -cdrom /home/aidan-workman/nautilus/nautilus.iso \
  -m 2048 -smp 2 \
  -serial file:/tmp/xhci.log \
  -display none \
  -device qemu-xhci,id=xhci \
  -device usb-mouse,bus=xhci.0 \
  -drive if=none,id=usbdisk,file=/tmp/usbtest.img,format=raw \
  -device usb-storage,bus=xhci.0,drive=usbdisk \
  -device usb-audio,bus=xhci.0 \
  -no-reboot &
QPID=$!
sleep 12
kill $QPID 2>/dev/null
wait 2>/dev/null
```

12 seconds is enough for boot + xHCI init + Phase 4 port scan + Phase 5 enumeration.
Bump it if you're testing something that takes longer (e.g. a full descriptor read).

#### Inspect the log

```sh
# Quick xHCI-only summary
grep -E "xhci|port [0-9]|status change|reset complete|drained|ENABLE_SLOT|enumerated" /tmp/xhci.log

# Just see errors
grep -E "ERROR|timeout" /tmp/xhci.log

# Full output
cat /tmp/xhci.log
```

QEMU xHCI implements the full register interface including PORTSC, command/event rings, and
TRB completion. It will generate proper port status change events on attach.

**Checkpoints:**
1. Controller reset completes without timeout
2. DCBAAP, CRCR, ERST registers read back the written values
3. `USBSTS.HCH` clears after `USBCMD.RS = 1`
4. Port status change event fires for the emulated USB mouse/storage
5. ENABLE_SLOT returns slot ID 1
6. ADDRESS_DEVICE completes with `CompletionCode = 1` (Success)
7. GET_DESCRIPTOR returns `0x0412` (QEMU's vendor ID) in the device descriptor

**QEMU port layout to expect:** `qemu-xhci` exposes 8 root hub ports. Ports 1-4 are USB 3
(SuperSpeed, speed=4); ports 5-8 are USB 2 (HighSpeed, speed=3). With the device set above,
the USB 3 mouse lands on port 2 and the USB 2 storage lands on port 5.

### T3: NK Shell Commands

If NK has a debug/shell interface, add commands:

```
xhci_dump    — print all controller registers and capability fields
xhci_ports   — print PORTSC for each port
xhci_slots   — print slot/EP context for each active slot
usb_list     — list enumerated USB devices
```

These make it possible to inspect state without a debugger.

### T4: GDB + QEMU Remote Debug

NK already documents GDB usage (see `USING-GDB.md`). Set breakpoints at:
- `xhci_init` — verify register reads
- `xhci_irq_handler` — verify events are dequeued correctly
- `xhci_enable_slot` — verify command ring mechanics
- `xhci_address_device` — verify input context is populated correctly

Use `x/4xw` to read raw MMIO and compare against expected register values from the spec.

### T5: USB Mass Storage (Functional Test)

Once enumeration works, implement the USB Mass Storage class driver (Bulk-Only Transport, SCSI
command set). Test:
- `INQUIRY` command returns device info
- `READ(10)` returns correct data from a known test image
- Compare against the raw image file content

This validates the full transfer path: control (enumeration) + bulk (data transfer).

### T6: USB HID (Human Interface Device)

Attach a USB keyboard or mouse in QEMU (`-device usb-kbd`). Implement the HID boot protocol
(single interrupt IN endpoint, 8-byte keyboard report). Test that key presses generate
interrupt transfers with correct keycodes.

### T7: Real Hardware

After QEMU validation, test on physical x86_64 hardware. Expected additional challenges:
- Power management sequences (U1/U2 link power states)
- Root hub port speed detection across USB 2 and USB 3 ports
- BIOS handoff (XHCI BIOS ownership bit in HCCPARAMS: write `USBLEGSUP.OS_OWNED`, poll until
  `BIOS_OWNED` clears)
- More complex port topology (hubs, TT, etc.)

The BIOS handoff step (often missed) is critical on real hardware — many xHCI controllers start
with BIOS ownership and will not respond until ownership is transferred.

---

## Implementation Order

| Phase | Description                        | Depends On    |
|-------|------------------------------------|---------------|
| 0     | Infrastructure (DMA, barriers)     | —             |
| 1     | Header file (all structs/offsets)  | —             |
| 2     | PCI probe + MMIO setup             | 1             |
| 3     | HC reset + initialization          | 2             |
| 4     | Port management                    | 3             |
| 5     | Device enumeration                 | 3, 4          |
| 6     | USB core abstraction               | 5             |
| 7     | Sync / event handling              | 3, 5          |
| 8     | Kconfig / Makefile                 | all           |
| T1    | Build + static checks              | 1             |
| T2–T4 | QEMU + GDB testing                 | 3+            |
| T5–T6 | Functional class driver tests      | 5, 6          |
| T7    | Real hardware                      | T2–T6 passing |
