# Patched ni_usb driver — modern code base

The same fixes forward-ported onto the current mainline-style ni_usb code
(`drivers/staging/gpib/ni_usb` / recent linux-gpib releases). Quirk 1's
`addressed_transfer_lock` already exists upstream; this port adds quirks 2-5,
the `rpp_force` skip-ppoll parameter, the bulk-in pipe hardening
(stop-on-timeout, late-response scoop, attach drain, `pipe_dirty` resync),
and the lock the parallel-poll request pair was missing even upstream.

**Status: VALIDATED** (2026-06-12) on Ubuntu 26.04 LTS, kernel 7.0.0-14,
linux-gpib 4.3.7: drop-in replacement for
`linux-gpib-kernel-4.3.7/drivers/gpib/ni_usb/ni_usb_gpib.{c,h}`, compiled
first try with no source changes. Full AMIGO protocol exchange, HPDir
identify/info, and an HPDir partial duplication byte-identical to the
reference image produced on the linux-gpib 4.0.3 bench. The quirk fixes are
independent of kernel and linux-gpib version, as expected: the quirks live in
the adapter's firmware.
