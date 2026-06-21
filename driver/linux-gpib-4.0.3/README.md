# Patched ni_usb driver — linux-gpib 4.0.3 code base

Drop-in replacement for `drivers/gpib/ni_usb/ni_usb_gpib.{c,h}` in a
linux-gpib 4.0.3 tree (as patched for HPDir by Ansgar Kueckes — but the fixes
themselves do not depend on that patch).

**Status: fully validated** — see `docs/FIRMWARE_QUIRKS.md`. Byte-perfect
HPDir duplication of a 14.5 MB AMIGO disc, identical results on kernels
3.13.0-170 and 4.4.0-148 (Ubuntu 14.04).

Note: this build still contains a few `NIDBG`-prefixed diagnostic printk lines
(low volume except during data transfers) — harmless, useful when reporting
issues, and easy to grep out if they bother you.

Build: from the linux-gpib `drivers/` directory, `sudo make`
(optionally `LINUX_SRCDIR=/usr/src/linux-headers-<version>` to cross-build for
a non-running kernel), then copy the `.ko` files into
`/lib/modules/<version>/gpib/...` and run `depmod -a`.
