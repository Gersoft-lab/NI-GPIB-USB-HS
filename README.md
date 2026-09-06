# NI GPIB-USB-HS firmware quirks — driver fixes & documentation

Recent firmware revisions of the National Instruments **GPIB-USB-HS** adapter silently
break communication with vintage HP-IB peripherals — desynchronized transfer pipes,
malformed responses, broken addressing state and inverted parallel-poll results, none
of it documented anywhere. This repository documents these firmware quirks and
provides the patched drivers, with the diagnosis behind each fix.

Originally developed to read an **HP 9133XV hard disc from 1983** over USB: HPDir
identify, info and full byte-perfect duplication now work, validated against a
known-good TNT4882 PCI setup. Linux is supported today; Windows is being investigated.

## Does this sound familiar?

If you are using a GPIB-USB-HS with linux-gpib against HP-IB era equipment and see:

- `ibcmd` failing with **EFAULT / Bad address** out of nowhere
- reads failing with **EIO although the device answered** (visible in `dmesg` parse errors)
- every write failing with **"no listeners currently addressed"** while the device
  demonstrably acknowledges its address
- reads **truncated to a single byte**
- parallel poll returning nonsense like `0x5f` / `0x7f` instead of your device's bit
- data **writes timing out although the bytes demonstrably reached the device**
- the adapter **wedging** until physically replugged
- HPDir reporting `AMIGO identify returns $FFFF`, `ppoll timeout` or
  `AMIGO status failed`

…then you have a recent-firmware adapter (tested: S/N era of `0x709b` devices) and
this repository is for you. None of these are bugs in your code or your
40-year-old drive — the root cause is the adapter's recent firmware, which the
driver simply doesn't handle yet (a couple of these even bite current mainline
linux-gpib).

## What's inside

| Path | Content |
|---|---|
| `docs/FIRMWARE_QUIRKS.md` | Full write-up: each quirk, its symptoms, diagnosis and fix |
| `driver/linux-gpib-4.0.3/` | Patched `ni_usb_gpib.c/.h` for linux-gpib 4.0.3 (validated, kernels 3.13.0-170 and 4.4.0-148) |
| `driver/modern/` | Same fixes ported to the current mainline/linux-gpib code base (kernels 7.0.0-14 and 7.0.0-22) |

## Quick start (Linux)

1. Build linux-gpib with the patched `ni_usb` driver from `driver/`, install the
   modules, `depmod -a`.
2. `/etc/gpib.conf`: set your controller address (`pad = 21` for HP hosts),
   `master = yes`, and — important — **`set-reos = no`** (REOS silently truncates
   board-level reads at any data byte matching the EOS character; HP-IB data is
   full of them) and **`set-eot = yes`** (recent firmware never acknowledges data
   writes that don't end with EOI — quirk 6 — although the data does land).
3. (Re)initialize the adapter cleanly — a software replug via
   `USBDEVFS_RESET` (rmmod → USB reset ioctl → modprobe → `gpib_config`) avoids
   the physical cable ritual.
4. Sanity-check an AMIGO drive at address 2 (a DSJ + Request Status exchange);
   this also clears any pending power-on DSJ.
5. For HPDir: clear the drive's DSJ first (step 4 does it), then enable the
   skip-ppoll workaround and go:

   ```sh
   echo 32 | sudo tee /sys/module/ni_usb_gpib/parameters/rpp_force   # 0x80 >> address
   hpdir -info 702:
   cd /tmp && hpdir -dup 702: mydisc.hpi
   ```

   `rpp_force` exists because HPDir samples the drive's parallel-poll line
   microseconds after addressing it — a race that PCI adapters win and USB
   round-trips (~4 ms) structurally cannot. Against an instantly-ready drive
   (e.g. an MFM-emulated one) forcing the response is semantically sound.
   Set it back to `0` for real polls.

## The quirks, in one breath

Orphaned responses shift the firmware's bulk-in queue so every request receives the
previous request's answer; read responses omit a block the parser expects, so
successful reads get discarded; a go-to-standby request irreversibly breaks the
firmware's addressing engine, killing every subsequent write; reads that end without
EOI report a garbage byte count that crashes user space; parallel-poll results
come back as raw active-low line levels instead of the logical byte every other
driver returns; and data writes without a terminating EOI are executed on the bus
but never acknowledged, so working writes look like timeouts. Full details,
symptoms and diagnosis in [`docs/FIRMWARE_QUIRKS.md`](docs/FIRMWARE_QUIRKS.md).

## Validation

Three fully independent read paths produce **bit-identical images** (same SHA-256)
of the 14.5 MB hard-disc volume of an HP 9133XV: a Python AMIGO read path over USB,
`hpdir -dup` over USB with the patched driver, and an `mfm_util` extraction of the
MFM-level source image served by a
[David Gesswein MFM emulator](https://www.pdp8online.com/mfm/mfm.shtml).
Identical behaviour confirmed across Ubuntu 14.04 (kernels 3.13 and 4.4,
linux-gpib 4.0.3) and Ubuntu 26.04 (kernel 7.0, linux-gpib 4.3.7 with the
`driver/modern` port).

Also validated against the **real 1983 MFM mechanism** (not just the emulator):
full AMIGO exchange, HPDir identify/info, and complete Python and `hpdir -dup`
dumps — both byte-identical to each other and to an independent MFM-level capture
of the same disc, zero read errors. One behavioural difference worth knowing:
unlike the emulator, the real drive only asserts its parallel-poll line while a
response is pending (it does not hold it at idle), but the post-addressing race
that motivates `rpp_force` is identical on both.

The **write path** is validated at scale too (on the emulated drive), via two
independent routes: a full 14.5 MB image restored with EOI-terminated buffered
writes (56,730 sectors, zero failures), and the same image restored with HPDir's
standard streaming `-dup` — which the driver's quirk-6 ack emulation takes from
unusable to **~90 records/s, about twice the read speed** (~10 minutes for the
full disc). Both read back byte-identical to the source. See the quirk 6
write-up for the mechanism.

## Status / roadmap

- [x] linux-gpib 4.0.3 driver — validated end to end
- [x] Port to modern code base (linux-gpib 4.3.7 / staging style)
- [x] Validated on current Ubuntu (26.04, kernel 7.0, linux-gpib 4.3.7)
- [ ] Upstream submission (linux-gpib / kernel staging)
- [ ] Windows (NI 488.2 path) — under investigation

## Credits

- **Ansgar Kueckes** — [HPDir & HPDrive](https://www.hp9845.net/9845/projects/hpdir/),
  and the linux-gpib patches this work builds on
- **Christian Grosz** — [lif80utils](https://codeberg.org/Boeingflieger/lif80utils),
  whose ppoll-free AMIGO protocol approach showed the way
- **Anders** (VintHPcom) — the skip-ppoll insight behind `rpp_force`
- **David Gesswein** — the [MFM emulator](https://www.pdp8online.com/mfm/mfm.shtml)
  that provided a healthy, reproducible test drive
- **Frank Mori Hess** and the [linux-gpib](https://linux-gpib.sourceforge.io/)
  maintainers

## License

GPL-2.0 (same as the original linux-gpib driver this work derives from).
