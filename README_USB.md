# xHCI USB Host Controller Driver for Nautilus

## Authors
- Said Aydin
- Aidan Workman

## Overview
This project implements a USB 3.0 xHCI (Extensible Host Controller Interface) 
host controller driver for the Nautilus Aerokernel. The driver enables Nautilus 
to discover, enumerate, and communicate with USB devices attached to an xHCI 
controller, and includes a USB Mass Storage Class (MSC) driver and USB Hub 
driver built on top of the xHCI layer.

## What We Built
- `src/dev/xhci.c` — the core xHCI host controller driver
- `src/dev/usb.c` — USB device registry and class driver framework  
- `src/dev/usb_msc.c` — USB Mass Storage Class driver (Bulk-Only Transport)
- `src/dev/usb_hub.c` — USB Hub class driver for downstream device enumeration
- `include/dev/xhci.h` — xHCI driver header and data structures
- `include/dev/usb.h` — USB core header

## How to Build

### Prerequisites (Ubuntu 24.04)
```bash
sudo apt install build-essential gcc nasm grub-common grub-pc-bin xorriso qemu-system-x86 git make libncurses-dev
```

### Build Steps
```bash
git clone https://github.com/aidwork11/nautilus-usb
cd nautilus-usb
make menuconfig  # Enable: Device Drivers -> xHCI, USB MSC, USB Hub
                 # Enable: Configuration -> Mirror virtual console output to serial
make -j$(nproc)
make isoimage
```

## How to Run

### Create a test disk image
```bash
dd if=/dev/zero of=test.img bs=1M count=16
```

### Boot in QEMU with a USB mass storage device
```bash
qemu-system-x86_64 \
  -machine q35 \
  -m 2G \
  -cdrom nautilus.iso \
  -device qemu-xhci,id=xhci \
  -drive if=none,id=usbdisk,file=test.img,format=raw \
  -device usb-storage,bus=xhci.0,drive=usbdisk \
  -no-reboot
```

When Nautilus boots, press `b` at the virtual console selector to view the 
system log and see the xHCI driver output.

## Expected Output
A successful run produces the following in the system log:
xhci: found xHCI controller
xhci: controller reset complete
xhci: controller running
xhci: port 1: device already connected at startup
xhci: ENABLE_SLOT -> slot 1
xhci: slot 1: addressed (USB addr=1, speed=3)
xhci: slot 1: vendor=0x46f4 product=0x0001
xhci: CONFIGURE_ENDPOINT complete
usb_msc: slot 1: INQUIRY vendor='QEMU' product='QEMU HARDDISK'
usb_msc: slot 1: capacity 32768 blocks x 512 B (16 MiB)
usb_msc: registered block device 'usb-msc0'
usb: bound driver 'usb-msc' to slot 1
xhci: port 1 enumerated as slot 1

## Driver Architecture
usb_msc.c / usb_hub.c      (class drivers)
|
usb.c                 (device registry, driver matching)
|
xhci.c                (host controller driver)
|
hardware / QEMU

The xHCI driver handles all low-level hardware interaction — command rings, 
event rings, TRB management, doorbell registers, and MSI-X interrupts. Class 
drivers sit on top and only call three functions: xhci_control_transfer(), 
xhci_normal_transfer(), and xhci_isoch_transfer().

## Key Design Decisions
- **FreeRTOS-style wait queues** for sleeping on command and transfer completions
- **Bitmap-based hot-plug tracking** for pending port enumerations and disconnects
- **Dedicated port worker thread** for handling hot-plug events outside IRQ context
- **3-phase disconnect teardown** to safely tear down slots without race conditions
- **MSI-X fixup** to work around Nautilus's 32-bit BAR limitation in PCI MSI-X detection

## Known Limitations
- Port bitmap only supports up to 63 ports (sufficient for all real hardware)
- No USB 3.x Link Power Management (U1/U2/U3 states)
- No stream support for SuperSpeed bulk endpoints
- `xhci_pci_deinit` is a stub — no graceful controller shutdown on unload
- Serial console output requires virtual console mirror to be enabled in menuconfig

## References
- [xHCI Specification](https://www.intel.com/content/dam/www/public/us/en/documents/technical-specifications/extensible-host-controler-interface-usb-xhci.pdf)
- [Nautilus Aerokernel](https://github.com/HExSA-Lab/nautilus)
