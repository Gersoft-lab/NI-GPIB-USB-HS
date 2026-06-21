# NI GPIB-USB-HS recent-firmware quirks — symptoms, diagnosis, fixes

This document describes the firmware behaviours of recent National Instruments
GPIB-USB-HS adapters (USB ID `3923:709b`) that break linux-gpib, how each one
manifests, how it was diagnosed, and how the drivers in this repository fix it.

**Test setup**: HP 9133XV disc unit (AMIGO protocol, hard disc emulated at the
MFM level by a [David Gesswein emulator](https://www.pdp8online.com/mfm/mfm.shtml),
so the drive side is healthy and reproducible by construction), linux-gpib 4.0.3
and the mainline-derived code base, kernels 3.13 / 4.4, HPDir 3.0 RC1.
A National Instruments TNT4882 PCI board running the same software stack served
as the known-good reference throughout.

A note on scope: everything below was diagnosed against HP-IB era peripherals,
which exercise corners of IEEE-488 (secondary addressing, parallel poll,
controller/talker dances) that modern instruments rarely touch. Plain
read/write instrument control may only ever hit quirks 1 and 2.

---

## Quirk 1 — bulk-in pipe desynchronization after an abandoned response

**Symptoms.** Random `EFAULT` on `ibcmd`; status blocks with impossible values
(`count=65227`); a register read returning what is recognizably the response to
an earlier data-read request; everything failing in cascade after one timeout;
in dmesg: `parse_register_read_block: parse error: wrong start id`.

**Mechanism.** Every bulk-out request gets exactly one bulk-in response. If the
host abandons a response (URB killed after a timeout — e.g. a read addressed to
nobody), the firmware still queues it. From then on **every request receives
the previous request's response**, shifted by one, forever. Nothing in the
protocol resynchronizes by itself; only re-enumeration used to clear it.

**Fix (multi-layered).**
- serialize each request/response pair under a mutex
  (`addressed_transfer_lock` — already present in modern code bases, backported
  to 4.0.3; the parallel-poll pair was missing it everywhere and gets it here);
- on a receive timeout, send `NI_USB_STOP_REQUEST` on the control endpoint and
  *wait* for the aborted response instead of killing the URB;
- if the URB had to be killed anyway, synchronously scoop late responses off
  the bulk-in pipe;
- drain the pipe at attach time;
- a `pipe_dirty` flag is set whenever a response arrives with the wrong status
  id or an impossible count; the next outgoing request first performs a
  stop-and-drain resynchronization. The driver self-heals.

## Quirk 2 — data-read responses omit the register-write status block

**Symptoms.** A read fails with `EIO` although the device demonstrably answered
(the data is visible in a dmesg raw dump); in dmesg:
`parse_board_ibrd_readback: unexpected data: register write status id=0x4, expected 0x9`
followed by `received unexpected termination block`.

**Mechanism.** The driver appends two register writes (`AUX_HLDI`,
`AUX_CLEAR_END`) to every data-read request, and the parser expects the
response to echo a register-write status block (id `0x09`) between the read
status and the termination block. Recent firmware omits that block — the
termination (`04 00 00 00`) directly follows the padding. The parser overruns
into garbage, the byte count check fails, and a successful read is discarded.
**The parser in current mainline code has the same problem.**

**Fix.** Accept a termination block where the register-write status block was
expected and finish parsing normally.

## Quirk 3 — the IBGTS request irreversibly breaks the firmware's addressing engine

This was the big one: it made **every addressed write fail**, which user space
reports as the deeply misleading *"You have attempted to write data or command
bytes, but there are no listeners currently addressed."*

**Symptoms.** Writes to a device that provably acknowledges its address fail
forever (firmware error 3 — "not in TACS"); the firmware's reported `ibsta`
shows TACS set right after the addressing command and gone again by the time
of the write.

**Mechanism.** The library calls "go to standby" (IBGTS) after every
`send_setup`. On recent firmware, the IBGTS request not only drops ATN — it
**wipes the engine's addressing model, and the engine never latches TACS from
subsequent command bytes again** (not even after IFC). Once one IBGTS has been
issued, addressed writes are dead until re-enumeration. The same corruption is
triggered by out-of-band register writes to AUXMR or BCR — there is no safe
way to manipulate ATN behind the engine's back.

**Fix.** `ni_usb_go_to_standby()` is a no-op. This is safe: the firmware's own
data-read and data-write requests manage ATN themselves. (Verified end to end:
addressing, command writes, data reads and parallel polls all behave with gts
disabled; the TNT4882-PCI reference behaviour is reproduced.)

## Quirk 4 — garbage byte count on reads that end without EOI

**Symptoms.** User-space crash (segfault) after a short read; in dmesg:
`bug: discarded data. actual_bytes_read=170, j=4` — 170 being `0xaa`.

**Mechanism.** The read response carries a "bytes in last data block" byte.
When the read terminates without END/EOI, recent firmware leaves `0xaa` filler
there instead of the count. The driver believed it and reported e.g. 170 bytes
read into a 4-byte user buffer. **Mainline has the same problem.**

**Fix.** Clamp the reported count to the number of bytes actually parsed out
of the response's data blocks.

## Quirk 5 — parallel-poll results are raw active-low line levels

**Symptoms.** `ibrpp` returns values like `0x5f` or `0x7f` where the expected
response byte is e.g. `0x20`; HPDir loops or times out on its ppoll gates.

**Mechanism.** The poll executes correctly on the bus, but the result byte in
the response contains the raw DIO line levels — active low, i.e. a responding
device reads as a **0** bit — in the HP `DIO(8-A)` orientation. Every
register-level driver (and older firmware) returns the active-high logical
byte. Example on a bus with responding drives at addresses 0 and 2:
raw `0x5f` instead of logical `0xa0`.

**Fix.** Invert the byte (`~raw`). The bit orientation is left as-is — it is
already the `DIO(8-A)` orientation register-level drivers produce; display
layers that remap by address (e.g. HPDir's `-query`) keep working.

**Two hard warnings discovered while working on this path:**
- The second byte of the rpp request is a poll timeout code in principle, but
  recent firmware only accepts `0xf0` there. Any other value (we tried `0xf3`)
  wedges the firmware **beyond the reach of a USB reset** — only a physical
  power cycle of the port recovers it.
- The rpp request leaves ATN+EOI asserted (IDY state) until the next request
  arrives. Do not try to release them out of band — see quirk 3. The lingering
  IDY is mostly harmless… except for the race below.

## Quirk 6 — data writes without EOI never get their completion response

**Symptoms.** `ibwrt` of a data payload fails with a timeout (driver log:
`ni_usb_write: ni_usb_receive_bulk_msg returned -110, usb_bytes_read=0`),
the device reports a status error afterwards — **yet the data actually
arrived on the device**. Small command writes (a few bytes) work fine.

**Mechanism.** When a device data write is requested **without EOI
termination** (`send_eoi = 0`, no EOS), recent firmware executes the GPIB
transfer — the bytes demonstrably reach the listener — but **never sends its
completion/status block back on the bulk-in pipe**. The driver waits for a
response that will never come and times out. Payload size is irrelevant
(isolated with 64-byte and 256-byte writes; the discriminating variable is
the EOI flag). With EOI set, the same write completes normally and the
status block arrives immediately.

**Driver-side fix (in this repository's drivers).** Since the transfer
itself does execute, the driver emulates the missing acknowledgement:
the first EOI-less data write waits at most 500 ms for the status block
(healthy firmware answers within milliseconds); if nothing comes back, the
unit is flagged and all subsequent EOI-less writes return success
immediately without waiting. Any straggler response is absorbed by the
quirk-1 `pipe_dirty` resync machinery. This is what makes **streamed
writes** usable: HPDir's default streaming `-dup <image> <msus>` restore
runs at ~90 records/s — about twice the read speed, since writes no longer
pay the USB round-trip.

**Tool-side alternative (confirmed writes).** Terminating each data write
with EOI sidesteps the quirk entirely and gets you a real per-write
acknowledgement:
- device-level code: open the data connection with `eot = 1`
  (e.g. `gpib.dev(board, pad, sad, tmo, 1)` in Python);
- board-level tools: add **`set-eot = yes`** to the interface block of
  `/etc/gpib.conf`.

Trade-off: EOI-terminated writes are individually confirmed but pay one
USB round-trip each (~29 sectors/s on the reference bench); emulated-ack
streaming is unconfirmed at the GPIB layer but ~3× faster — protocols with
their own status checks (e.g. AMIGO's DSJ) lose nothing.

With the quirk-1 pipe hardening in this driver (stop-on-timeout, late
response scoop, `pipe_dirty` resync), a non-EOI write on an unpatched-aware
setup no longer wedges the adapter either way: the pipe self-heals on the
next transaction.

---

## Not a firmware bug, but you will hit them anyway

### `set-reos = yes` truncates board-level reads (the default gpib.conf trap)

The stock `/etc/gpib.conf` enables REOS (terminate reads on the EOS character).
Board-level reads — which is what HPDir issues — then silently stop at the
first data byte equal to the EOS character. Binary HP-IB data is full of
`0x00`/`0x0a` bytes, so e.g. a 4-byte AMIGO status read returns 1 byte and the
application reports "AMIGO status failed". Device-level reads through the
library bindings disable EOS per descriptor, which makes the failure look
maddeningly tool-dependent.

**Fix: `set-reos = no`** in `/etc/gpib.conf`.

### The ppoll-after-addressing race (why HPDir needs `rpp_force` over USB)

HPDir samples a drive's parallel-poll line immediately after addressing it.
The drive's controller releases its PP line upon being addressed — but it
takes it a little while (firmware on a Z80-class CPU). Over PCI the poll
lands ~2 µs after the addressing command, *before* the release: the gate
passes. Over USB the poll lands one USB round trip later (~4 ms): the line is
already released and the gate can never pass. This race is structural; no
driver change can win it.

**Workaround:** the `rpp_force` module parameter (the "skip ppoll" idea
suggested by Anders on the VintHPcom list, implemented driver-side so
applications need no change):

```sh
echo 32 | sudo tee /sys/module/ni_usb_gpib/parameters/rpp_force  # 0x80 >> drive address
echo 0  | sudo tee /sys/module/ni_usb_gpib/parameters/rpp_force  # back to real polls
```

Against an instantly-ready drive (an MFM-emulated unit, or any healthy drive
in a read workflow) a forced "ready" answer is semantically sound.
One operational note for AMIGO drives: clear the drive's DSJ before starting
HPDir (a single Request Status exchange does it) — with a pending DSJ=1,
HPDir takes its "Amigo clear" path whose gate waits for a line *release*,
which a forced constant can never satisfy.

### Software replug

Most wedge states (including everything quirk 1 produces, and the
rmmod/modprobe reload dance) are fully cleared by a USB port reset — the
`USBDEVFS_RESET` ioctl, no cable touching required. The reliable sequence is
rmmod → `USBDEVFS_RESET` ioctl on the adapter's `/dev/bus/usb/...` node →
modprobe → `gpib_config`. The only state it cannot clear is the deep wedge
caused by a non-`0xf0` rpp timeout byte (see quirk 5).

---

## Reproduction / validation guide

1. Build linux-gpib with the patched `ni_usb` driver
   ([`driver/linux-gpib-4.0.3/`](../driver/linux-gpib-4.0.3) for the 2015 code
   base, [`driver/modern/`](../driver/modern) for current trees), install,
   `depmod -a`.
2. `/etc/gpib.conf`: controller `pad = 21`, `master = yes`, `set-reos = no`,
   `set-eot = yes`.
3. Software replug (the `USBDEVFS_RESET` sequence above) to initialize cleanly.
4. Sanity-check the AMIGO drive with a DSJ + Request Status exchange (and purge
   a pending power-on DSJ) before anything board-level.
5. Read: a buffered-read loop (seek then auto-incrementing reads, one DSJ per
   sector) images the full disc; or `hpdir -dup 702: disc.hpi`.
6. HPDir: `rpp_force` as above, then `hpdir -info 702:`,
   `hpdir -dup 702: disc.hpi` (note: plain target filename — an absolute path
   is parsed as an msus).

**Validation result on the reference bench:** three fully independent read
paths produce bit-identical 14.5 MB images (same SHA-256) — a Python AMIGO
buffered-read tool over USB, `hpdir -dup` over USB, and an `mfm_util` extraction
of the MFM-level image the emulated drive serves. Identical results on kernels
3.13 and 4.4.

**Write-path validation:** the full write chain is validated at scale on the
emulated drive, twice over:
- EOI-terminated buffered writes (per-sector DSJ): complete 14.5 MB image
  restored, 56,730 sectors, zero failures, ~29 sect/s — read-back SHA-256
  identical to the source;
- **HPDir streaming restore** (`hpdir -y -dup image.hpi 702:`, EOI-less
  writes relying on the driver's quirk-6 ack emulation): same image, ~10
  minutes (~90 records/s), read-back SHA-256 identical to the source.

Single-sector write/read-back with a known pattern confirms reversibility on a
real mechanical drive.

Two HPDir-specific caveats on the write side:
- `hpdir -dup <file> <msus> -r first,last` does **not** write that block
  range from a file source — the output bears no resemblance to the file
  content (this cost us hours; whatever `-r` means with a file source, it is
  not "raw range copy"). A full `-dup file msus` without `-r` is a faithful
  raw copy.
- `-nostream` (sector-at-a-time) writes through HPDir remain impractically
  slow over USB (~0.2-0.4 records/s): its per-record pacing waits on
  parallel-poll behaviour a constant `rpp_force` response cannot satisfy
  (an experimental `rpp_force_transition` parameter in the 4.0.3 driver,
  emulating a busy→ready transition after each write, did not unblock it).
  Use the default streaming mode — it is both correct and fast with the
  quirk-6 fix.

**Real-hardware validation:** the same protocol run against the actual 1983
MFM mechanism (emulator removed, original drive reconnected) gives the same
result — full AMIGO exchange, HPDir identify/info, and complete dumps via both
the Python tools and `hpdir -dup`, byte-identical to each other and to an
independent MFM-level capture of the same disc taken weeks earlier (the only
difference being a 4-byte boot timestamp the OS itself had updated in sector 0).
Zero read errors over 56,730 sectors. Behavioural note: the real drive asserts
its parallel-poll line only while a response is pending — it does not hold it
at idle the way the emulator does — but the post-addressing race that motivates
`rpp_force` (quirk-independent, USB latency) is identical on both, so the
workaround is needed for real and emulated drives alike.

## How the quirks were isolated

The diagnosis relied on a few simple techniques, useful to anyone chasing
similar firmware behaviour:

- **Step-by-step AMIGO replay**, sampling the parallel poll and the bus lines
  (`iblines`) after every command/read/write. Watching `TACS`/`LACS` and the
  DIO levels at each stage is what exposed quirks 3 and 5.
- **Command-sequence bisection** (byte ordering, bit-8 parity variants,
  with/without interleaved polls) to pinpoint which single step disturbs a
  device.
- **Listen-secondary mapping** (which secondaries a device acknowledges, via
  `ibln`'s NDAC check) to confirm addressing was intact.
- Driver-side, the `pipe_dirty` machinery doubles as a desync detector — its
  dmesg lines tell you a response shift happened and where.
