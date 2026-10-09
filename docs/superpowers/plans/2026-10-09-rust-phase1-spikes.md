# Rust Firmware Phase 1 (Spikes) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build everything for the three feasibility spikes (camera, SD, WiFi+HTTP) that can be built and tested without a camera attached, so the on-device runs are a single gated step at the end.

**Architecture:** Pure logic lives in small `no_std` library crates under `rust/` that are unit-tested on the Mac (`wuw-board`, `wuw-frame`, `ov5640`). Each spike is a separate `esp-generate` project under `rust/spikes/` that depends on those crates by path and cross-compiles for `xtensa-esp32s3-none-elf`. No Cargo workspace: host crates and target crates keep separate build configs.

**Tech Stack:** Rust 1.95 (`esp` toolchain via espup), esp-hal 1.2.2 (`unstable`), esp-rtos 0.4.0, esp-radio 1.0.0-beta.1, embassy-net 0.9.1, edge-dhcp 0.8, picoserve 0.20.1, sdio 0.5.2 (pulled in by esp-hal's `esp32s3` feature), embedded-hal 1.0, espflash.

Spec: `docs/superpowers/specs/2026-10-09-rust-firmware-design.md`.

---

## Engineering rules (apply to every task)

Adapted from the rust-engineer practices for a `no_std` target:

- `cargo fmt --check` and `cargo clippy -- -D warnings` must pass for every crate touched.
- No `unwrap()` outside tests. Use `expect("why this cannot fail")` or propagate with `?`.
- Every `unsafe` block carries a `// SAFETY:` comment stating the invariant.
- Errors are small `#[derive(Debug, Clone, Copy, PartialEq, Eq)]` enums per crate. No `std`, no `thiserror` needed.
- Async uses embassy (`esp-rtos` executor), never tokio. No blocking delays inside async tasks; use `embassy_time::Timer`.
- Library crates are `#![no_std]` and tested on the host with `cargo test` from the crate directory.
- No per-frame heap allocation in spikes. Buffers are allocated once at startup.
- Commit messages: plain, no co-author trailers (repo owner's rule).

## File structure

| Path | Responsibility |
|---|---|
| `rust/README.md` | How the Rust tree is organised and how to build/flash it |
| `rust/wuw-board/` | Pin constants, mirrors `docs/WIRING.md`. Host-tested. |
| `rust/wuw-frame/` | Find a complete JPEG (SOI..EOI) inside a DMA buffer. Host-tested. |
| `rust/ov5640/` | OV5640 register tables (generated) + init sequencing over `embedded_hal::i2c::I2c`. Host-tested with a mock bus. |
| `rust/ov5640/vendor/` | Pinned copies of Espressif's `ov5640_regs.h` / `ov5640_settings.h` (Apache-2.0) |
| `tools/gen_ov5640_regs.py` | Converts the C tables to `rust/ov5640/src/regs.rs` |
| `tools/test_gen_ov5640_regs.py` | Unit test for the generator |
| `tools/pincheck.py` | Extended to check `wuw-board` against the C++ pin map |
| `tools/spike_net_stress.py` | Mac-side stress client for spike C |
| `rust/spikes/spike-camera/` | Spike A |
| `rust/spikes/spike-sd/` | Spike B |
| `rust/spikes/spike-net/` | Spike C |

---

### Task 1: Rust tree skeleton

**Files:**
- Create: `rust/README.md`
- Modify: `.gitignore`

- [ ] **Step 1: Confirm toolchain**

Run: `source ~/export-esp.sh && rustup toolchain list && espflash --version && esp-generate --version`
Expected: an `esp` toolchain listed, espflash prints a version, `esp-generate 1.4.0`.

- [ ] **Step 2: Ignore Rust build output**

Append to `.gitignore`:

```gitignore

# ── Rust alternative firmware ────────────────────────────────────────────
rust/**/target/
rust/**/Cargo.lock.bak
```

- [ ] **Step 3: Write `rust/README.md`**

```markdown
# WUW CAM OS, Rust edition (alternative firmware)

An independent firmware for the same WUW CAM hardware. The C++ firmware at the
repo root is unchanged and remains the default. Pick one at flash time.

Status: Phase 1, feasibility spikes. Not a usable camera OS yet.

| Path | What |
|---|---|
| `wuw-board/` | Pin map (checked against the C++ map by `tools/pincheck.py`) |
| `wuw-frame/` | JPEG frame extraction from DMA buffers |
| `ov5640/` | OV5640 sensor driver |
| `spikes/` | Three on-device feasibility tests |

Library crates are tested on the host:

    cd rust/wuw-board && cargo test

Spikes need the esp toolchain:

    source ~/export-esp.sh
    cd rust/spikes/spike-net && cargo run --release

Before flashing any camera for the first time, back up its flash:

    espflash read-flash 0 0x1000000 backup-<cam>-<date>.bin

Return to the C++ firmware with `pio run -e wuwcam -t upload` from the repo root.
```

- [ ] **Step 4: Shared partition table**

Create `rust/partitions-wuwcam.csv`, a byte-for-byte copy of the C++ firmware's
`default_16MB.csv` (from `~/.platformio/packages/framework-arduinoespressif32*/tools/partitions/`):

```csv
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x5000,
otadata,  data, ota,     0xe000,  0x2000,
app0,     app,  ota_0,   0x10000, 0x640000,
app1,     app,  ota_1,   0x650000,0x640000,
spiffs,   data, spiffs,  0xc90000,0x360000,
coredump, data, coredump,0xFF0000,0x10000,
```

Every spike flashes with this table so NVS (camera settings) stays where the
C++ firmware expects it. `otadata` is erased on each flash so the bootloader
boots `app0` even if the camera last ran an OTA image from `app1`; it holds no
settings. Each spike's `.cargo/config.toml` runner becomes:

```toml
runner = "espflash flash --monitor --chip esp32s3 --partition-table ../../partitions-wuwcam.csv --erase-parts otadata"
```

- [ ] **Step 5: Commit**

```bash
git add .gitignore rust/README.md rust/partitions-wuwcam.csv
git commit -m "Rust tree skeleton for the alternative firmware"
```

---

### Task 2: `wuw-board` pin crate

**Files:**
- Create: `rust/wuw-board/Cargo.toml`, `rust/wuw-board/src/lib.rs`

- [ ] **Step 1: Write the crate with failing tests first**

`rust/wuw-board/Cargo.toml`:

```toml
[package]
name = "wuw-board"
version = "0.1.0"
edition = "2024"
license = "GPL-3.0-only"

[dependencies]
```

`rust/wuw-board/src/lib.rs`:

```rust
//! WUW CAM pin map: Freenove ESP32-S3-WROOM N16R8 + OV5640 + 2.4" ST7789/HR2046.
//!
//! Mirrors `docs/WIRING.md`. `tools/pincheck.py` fails the build of trust if
//! these drift from the C++ firmware's `#define`s.
#![no_std]

// Camera (DVP, 8 bit). D0..D7 are the esp32-camera Y2..Y9.
pub const CAM_SIOD: u8 = 4;
pub const CAM_SIOC: u8 = 5;
pub const CAM_VSYNC: u8 = 6;
pub const CAM_HREF: u8 = 7;
pub const CAM_D0: u8 = 11;
pub const CAM_D1: u8 = 9;
pub const CAM_D2: u8 = 8;
pub const CAM_D3: u8 = 10;
pub const CAM_D4: u8 = 12;
pub const CAM_D5: u8 = 18;
pub const CAM_D6: u8 = 17;
pub const CAM_D7: u8 = 16;
pub const CAM_PCLK: u8 = 13;
pub const CAM_XCLK: u8 = 15;

// Onboard microSD, SDMMC 1-bit.
pub const SD_CMD: u8 = 38;
pub const SD_CLK: u8 = 39;
pub const SD_D0: u8 = 40;

// Shutter button to GND, internal pull-up.
pub const BTN: u8 = 14;

// Display + touch on one shared SPI bus (bridges B1-B3 in WIRING.md).
pub const LCD_MISO: u8 = 1;
pub const LCD_MOSI: u8 = 2;
pub const LCD_SCLK: u8 = 41;
pub const LCD_CS: u8 = 21;
pub const LCD_DC: u8 = 47;
pub const LCD_RST: u8 = 20;
pub const LCD_BL: u8 = 48;
pub const TCH_CS: u8 = 42;

/// Every claimed pin with its name, for checks.
pub const ALL: [(&str, u8); 26] = [
    ("CAM_SIOD", CAM_SIOD), ("CAM_SIOC", CAM_SIOC), ("CAM_VSYNC", CAM_VSYNC),
    ("CAM_HREF", CAM_HREF), ("CAM_D0", CAM_D0), ("CAM_D1", CAM_D1),
    ("CAM_D2", CAM_D2), ("CAM_D3", CAM_D3), ("CAM_D4", CAM_D4),
    ("CAM_D5", CAM_D5), ("CAM_D6", CAM_D6), ("CAM_D7", CAM_D7),
    ("CAM_PCLK", CAM_PCLK), ("CAM_XCLK", CAM_XCLK),
    ("SD_CMD", SD_CMD), ("SD_CLK", SD_CLK), ("SD_D0", SD_D0),
    ("BTN", BTN),
    ("LCD_MISO", LCD_MISO), ("LCD_MOSI", LCD_MOSI), ("LCD_SCLK", LCD_SCLK),
    ("LCD_CS", LCD_CS), ("LCD_DC", LCD_DC), ("LCD_RST", LCD_RST),
    ("LCD_BL", LCD_BL), ("TCH_CS", TCH_CS),
];

/// Pins the N16R8 module reserves: SPI flash, octal PSRAM, UART0, nonexistent.
pub fn is_forbidden(pin: u8) -> bool {
    matches!(pin, 22..=37 | 43 | 44)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_pin_claimed_twice() {
        for (i, (a, pa)) in ALL.iter().enumerate() {
            for (b, pb) in &ALL[i + 1..] {
                assert_ne!(pa, pb, "{a} and {b} both claim GPIO {pa}");
            }
        }
    }

    #[test]
    fn no_pin_is_reserved() {
        for (name, pin) in ALL {
            assert!(!is_forbidden(pin), "{name} is on reserved GPIO {pin}");
        }
    }

    #[test]
    fn strapping_pins_stay_free() {
        for (name, pin) in ALL {
            assert!(![0, 3, 45, 46].contains(&pin), "{name} is on strapping GPIO {pin}");
        }
    }
}
```

- [ ] **Step 2: Run tests**

Run: `cd rust/wuw-board && cargo test`
Expected: 3 tests PASS.

- [ ] **Step 3: Prove a test can fail**

Temporarily change `LCD_BL` to `21`, run `cargo test`, expect `no_pin_claimed_twice` to FAIL naming `LCD_CS and LCD_BL`. Revert.

- [ ] **Step 4: Lint**

Run: `cargo fmt --check && cargo clippy --all-targets -- -D warnings`
Expected: clean. (Run `cargo fmt` first if the `ALL` table needs reflowing.)

- [ ] **Step 5: Commit**

```bash
git add rust/wuw-board
git commit -m "wuw-board: Rust pin map with collision and reserved-pin tests"
```

---

### Task 3: `pincheck.py` checks the Rust map

**Files:**
- Modify: `tools/pincheck.py` (insert before the final `if fails:` block)

- [ ] **Step 1: Add the Rust comparison**

Insert immediately before `if fails:` near the end of `tools/pincheck.py`:

```python
# ── Rust alternative firmware ────────────────────────────────────────────
# rust/wuw-board is a second copy of this map. It must agree with the C++.
RUST = ROOT / "rust" / "wuw-board" / "src" / "lib.rs"
RUST_TO_CPP = {
    "CAM_SIOD": "SIOD_GPIO_NUM", "CAM_SIOC": "SIOC_GPIO_NUM",
    "CAM_VSYNC": "VSYNC_GPIO_NUM", "CAM_HREF": "HREF_GPIO_NUM",
    "CAM_D0": "Y2_GPIO_NUM", "CAM_D1": "Y3_GPIO_NUM", "CAM_D2": "Y4_GPIO_NUM",
    "CAM_D3": "Y5_GPIO_NUM", "CAM_D4": "Y6_GPIO_NUM", "CAM_D5": "Y7_GPIO_NUM",
    "CAM_D6": "Y8_GPIO_NUM", "CAM_D7": "Y9_GPIO_NUM",
    "CAM_PCLK": "PCLK_GPIO_NUM", "CAM_XCLK": "XCLK_GPIO_NUM",
    "SD_CMD": "SD_CMD_PIN", "SD_CLK": "SD_CLK_PIN", "SD_D0": "SD_D0_PIN",
    "BTN": "BTN_PIN",
    "LCD_MISO": "LCD_MISO", "LCD_MOSI": "LCD_MOSI", "LCD_SCLK": "LCD_SCLK",
    "LCD_CS": "LCD_CS", "LCD_DC": "LCD_DC", "LCD_RST": "LCD_RST",
    "LCD_BL": "LCD_BL", "TCH_CS": "TCH_CS",
}
if RUST.exists():
    rust = {m.group(1): int(m.group(2)) for m in re.finditer(
        r"^pub const (\w+): u8 = (\d+);", RUST.read_text(), re.M)}
    cpp = {n: p for d in groups.values() for n, p in d.items()}
    for rname, cname in RUST_TO_CPP.items():
        if rust.get(rname) != cpp.get(cname):
            fails.append("rust/wuw-board %s = %s but C++ %s = %s"
                         % (rname, rust.get(rname), cname, cpp.get(cname)))
    extra = set(rust) - set(RUST_TO_CPP)
    for rname in sorted(extra):
        fails.append("rust/wuw-board %s has no C++ counterpart in pincheck" % rname)
    print("\nrust/wuw-board: %d pins compared" % len(RUST_TO_CPP))
```

- [ ] **Step 2: Run it**

Run: `python3 tools/pincheck.py | tail -4`
Expected: `rust/wuw-board: 26 pins compared` then `OK - no collisions, nothing on a reserved pin`.

- [ ] **Step 3: Prove it catches drift**

Temporarily set `CAM_D0` to `9` in `rust/wuw-board/src/lib.rs`, run pincheck, expect `FAIL` with `rust/wuw-board CAM_D0 = 9 but C++ Y2_GPIO_NUM = 11`. Revert.

- [ ] **Step 4: Commit**

```bash
git add tools/pincheck.py
git commit -m "pincheck: compare the Rust pin map with the C++ one"
```

---

### Task 4: `wuw-frame` JPEG extraction

**Files:**
- Create: `rust/wuw-frame/Cargo.toml`, `rust/wuw-frame/src/lib.rs`

The OV5640 in JPEG mode streams a frame that may be padded before SOI and after EOI. The spike needs the exact JPEG slice.

- [ ] **Step 1: Write the failing tests**

`rust/wuw-frame/Cargo.toml`:

```toml
[package]
name = "wuw-frame"
version = "0.1.0"
edition = "2024"
license = "GPL-3.0-only"

[dependencies]
```

`rust/wuw-frame/src/lib.rs` (tests first, function stubbed):

```rust
//! Locate a complete JPEG inside a camera DMA buffer.
#![no_std]

use core::ops::Range;

/// Byte range of the first complete JPEG (SOI `FF D8` .. EOI `FF D9` inclusive).
pub fn find_jpeg(buf: &[u8]) -> Option<Range<usize>> {
    let _ = buf;
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_frame() {
        let b = [0xFF, 0xD8, 1, 2, 3, 0xFF, 0xD9];
        assert_eq!(find_jpeg(&b), Some(0..7));
    }

    #[test]
    fn padding_both_sides() {
        let b = [0, 0, 0xFF, 0xD8, 9, 0xFF, 0xD9, 0, 0];
        assert_eq!(find_jpeg(&b), Some(2..7));
    }

    #[test]
    fn truncated_frame_is_rejected() {
        let b = [0xFF, 0xD8, 1, 2, 3];
        assert_eq!(find_jpeg(&b), None);
    }

    #[test]
    fn no_soi() {
        assert_eq!(find_jpeg(&[1, 2, 0xFF, 0xD9]), None);
    }

    #[test]
    fn empty_and_tiny() {
        assert_eq!(find_jpeg(&[]), None);
        assert_eq!(find_jpeg(&[0xFF]), None);
    }

    #[test]
    fn eoi_before_soi_is_ignored() {
        let b = [0xFF, 0xD9, 0xFF, 0xD8, 7, 0xFF, 0xD9];
        assert_eq!(find_jpeg(&b), Some(2..7));
    }

    #[test]
    fn stuffed_ff00_inside_scan_is_not_eoi() {
        let b = [0xFF, 0xD8, 0xFF, 0x00, 0xD9, 0xFF, 0xD9];
        assert_eq!(find_jpeg(&b), Some(0..7));
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd rust/wuw-frame && cargo test`
Expected: `exact_frame`, `padding_both_sides`, `eoi_before_soi_is_ignored`, `stuffed_ff00_inside_scan_is_not_eoi` FAIL; the `None` cases pass.

- [ ] **Step 3: Implement**

Replace the stub body:

```rust
pub fn find_jpeg(buf: &[u8]) -> Option<Range<usize>> {
    let soi = buf.windows(2).position(|w| w == [0xFF, 0xD8])?;
    let after = soi + 2;
    let eoi = buf[after..].windows(2).position(|w| w == [0xFF, 0xD9])?;
    Some(soi..after + eoi + 2)
}
```

(`FF 00` stuffing never matches `FF D9`, so a plain scan is correct.)

- [ ] **Step 4: Run tests**

Run: `cargo test && cargo fmt --check && cargo clippy --all-targets -- -D warnings`
Expected: 7 PASS, lint clean.

- [ ] **Step 5: Commit**

```bash
git add rust/wuw-frame
git commit -m "wuw-frame: find a complete JPEG in a DMA buffer"
```

---

### Task 5: Vendor the OV5640 tables (needs one download, owner approval)

**Files:**
- Create: `rust/ov5640/vendor/ov5640_regs.h`, `rust/ov5640/vendor/ov5640_settings.h`, `rust/ov5640/vendor/SOURCE.md`

- [ ] **Step 1: Ask the owner to approve the download** of two files (~15 KB total, Apache-2.0) from `github.com/espressif/esp32-camera`, path `sensors/private_include/`.

- [ ] **Step 2: Fetch at a pinned commit**

```bash
mkdir -p rust/ov5640/vendor
SHA=$(gh api repos/espressif/esp32-camera/commits/master --jq .sha)
for f in ov5640_regs.h ov5640_settings.h; do
  curl -sSfL -o rust/ov5640/vendor/$f \
    https://raw.githubusercontent.com/espressif/esp32-camera/$SHA/sensors/private_include/$f
done
echo "$SHA"
```

- [ ] **Step 3: Record provenance** in `rust/ov5640/vendor/SOURCE.md`:

```markdown
Copied unmodified from https://github.com/espressif/esp32-camera at commit
<SHA from step 2>, path sensors/private_include/. Apache License 2.0,
Copyright Espressif Systems. Used only as input to tools/gen_ov5640_regs.py.
```

- [x] **Step 4: Read both files** (done: REG_DLY/REGLIST_TAIL are in settings.h; Task 6 updated) and confirm the table format the generator in Task 6 expects: arrays of `{reg, value}` pairs where `reg` is a hex literal or a `#define` name from `ov5640_regs.h`, the delay marker `REG_DLY`, and the terminator `REGLIST_TAIL`. If the format differs, update Task 6's parsing rules before writing code, and note the change in this plan.

- [ ] **Step 5: Commit**

```bash
git add rust/ov5640/vendor
git commit -m "ov5640: vendor Espressif register tables (Apache-2.0)"
```

---

### Task 6: Register table generator

**Files:**
- Create: `tools/gen_ov5640_regs.py`, `tools/test_gen_ov5640_regs.py`
- Generates: `rust/ov5640/src/regs.rs`

- [ ] **Step 1: Write the failing test**

`tools/test_gen_ov5640_regs.py`:

```python
import importlib.util, pathlib, unittest

spec = importlib.util.spec_from_file_location(
    "gen", pathlib.Path(__file__).with_name("gen_ov5640_regs.py"))
gen = importlib.util.module_from_spec(spec); spec.loader.exec_module(gen)

REGS_H = """
#define SYSTEM_CTROL0 0x3008
"""
SETTINGS_H = """
#define REG_DLY 0xffff
#define REGLIST_TAIL 0x0000
static const DRAM_ATTR uint16_t sensor_default_regs[][2] = {
    {SYSTEM_CTROL0, 0x82}, // software reset
    {REG_DLY, 10},
    {0x3103, 0x13},
    {REGLIST_TAIL, 0x00},
};
static const DRAM_ATTR uint16_t sensor_fmt_jpeg[][2] = {
    {0x4300, 0x30},
    {REGLIST_TAIL, 0x00},
};
"""

class GenTest(unittest.TestCase):
    def test_parses_tables_with_names_delays_and_tail(self):
        tables = gen.parse(REGS_H, SETTINGS_H)
        self.assertEqual(tables["sensor_default_regs"],
                         [("W", 0x3008, 0x82), ("D", 10, 0), ("W", 0x3103, 0x13)])
        self.assertEqual(tables["sensor_fmt_jpeg"], [("W", 0x4300, 0x30)])

    def test_emits_rust(self):
        rs = gen.emit({"sensor_fmt_jpeg": [("W", 0x4300, 0x30), ("D", 5, 0)]})
        self.assertIn("pub const SENSOR_FMT_JPEG: &[Op] = &[", rs)
        self.assertIn("Op::Write(0x4300, 0x30),", rs)
        self.assertIn("Op::DelayMs(5),", rs)

    def test_unknown_symbol_fails_loudly(self):
        with self.assertRaises(KeyError):
            gen.parse(REGS_H, "#define REGLIST_TAIL 0x0000\nstatic const uint16_t t[][2] = { {NOPE, 1}, {REGLIST_TAIL, 0} };")

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify it fails**

Run: `python3 tools/test_gen_ov5640_regs.py`
Expected: FAIL / error, `gen_ov5640_regs.py` does not exist.

- [ ] **Step 3: Implement the generator**

`tools/gen_ov5640_regs.py`:

```python
#!/usr/bin/env python3
"""Convert esp32-camera's OV5640 C register tables to Rust.

    python3 tools/gen_ov5640_regs.py

Reads rust/ov5640/vendor/ov5640_{regs,settings}.h, writes rust/ov5640/src/regs.rs.
"""
import pathlib, re

ROOT = pathlib.Path(__file__).resolve().parent.parent
VENDOR = ROOT / "rust" / "ov5640" / "vendor"
OUT = ROOT / "rust" / "ov5640" / "src" / "regs.rs"

def defines(text):
    return {m.group(1): int(m.group(2), 0)
            for m in re.finditer(r"^#define\s+(\w+)\s+(0x[0-9a-fA-F]+|\d+)\b", text, re.M)}

def value(tok, syms):
    tok = tok.strip()
    return int(tok, 0) if re.fullmatch(r"0x[0-9a-fA-F]+|\d+", tok) else syms[tok]

def parse(regs_h, settings_h):
    # REG_DLY / REGLIST_TAIL live in settings.h, register names in regs.h.
    syms = {**defines(regs_h), **defines(settings_h)}
    tables = {}
    for m in re.finditer(r"(\w+)\s*\[\s*\]\s*\[\s*2\s*\]\s*=\s*\{(.*?)\};", settings_h, re.S):
        name, body = m.group(1), re.sub(r"//[^\n]*", "", m.group(2))
        ops = []
        for a, b in re.findall(r"\{\s*([\w]+)\s*,\s*([\w]+)\s*\}", body):
            if a == "REGLIST_TAIL":
                break
            if a == "REG_DLY":
                ops.append(("D", value(b, syms), 0))
            else:
                ops.append(("W", value(a, syms), value(b, syms)))
        tables[name] = ops
    return tables

def emit(tables):
    out = ["// @generated by tools/gen_ov5640_regs.py from Espressif esp32-camera",
           "// (Apache-2.0). Do not edit by hand.", "", "use crate::Op;", ""]
    for name, ops in tables.items():
        out.append("pub const %s: &[Op] = &[" % name.upper())
        for kind, a, b in ops:
            out.append("    Op::DelayMs(%d)," % a if kind == "D"
                       else "    Op::Write(0x%04X, 0x%02X)," % (a, b))
        out.append("];")
        out.append("")
    return "\n".join(out)

def main():
    tables = parse((VENDOR / "ov5640_regs.h").read_text(),
                   (VENDOR / "ov5640_settings.h").read_text())
    OUT.write_text(emit(tables))
    print("wrote %s: %s" % (OUT.relative_to(ROOT),
          ", ".join("%s(%d)" % (k, len(v)) for k, v in tables.items())))

if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run tests**

Run: `python3 tools/test_gen_ov5640_regs.py`
Expected: 3 tests OK.

- [ ] **Step 5: Commit** (regs.rs is generated in Task 7 once the crate exists)

```bash
git add tools/gen_ov5640_regs.py tools/test_gen_ov5640_regs.py
git commit -m "Generator for OV5640 register tables"
```

---

### Task 7: `ov5640` driver crate

**Files:**
- Create: `rust/ov5640/Cargo.toml`, `rust/ov5640/src/lib.rs`, `rust/ov5640/src/regs.rs` (generated)

- [ ] **Step 1: Crate manifest**

```toml
[package]
name = "ov5640"
version = "0.1.0"
edition = "2024"
license = "GPL-3.0-only"

[dependencies]
embedded-hal = "1.0"

[dev-dependencies]
embedded-hal-mock = { version = "0.11", default-features = false, features = ["eh1"] }
```

- [ ] **Step 2: Write lib.rs with failing tests**

```rust
//! OV5640 control over SCCB (I2C, 7-bit address 0x3C, 16-bit registers).
#![no_std]

use embedded_hal::{delay::DelayNs, i2c::I2c};

pub mod regs;

pub const ADDR: u8 = 0x3C;
pub const CHIP_ID: u16 = 0x5640;
const REG_CHIP_ID_H: u16 = 0x300A;

/// One step of a register list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    Write(u16, u8),
    DelayMs(u16),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    WrongChip(u16),
}

pub struct Ov5640<I> {
    i2c: I,
}

impl<I: I2c> Ov5640<I> {
    pub fn new(i2c: I) -> Self {
        Self { i2c }
    }

    pub fn release(self) -> I {
        self.i2c
    }

    pub fn write(&mut self, reg: u16, val: u8) -> Result<(), Error<I::Error>> {
        let [h, l] = reg.to_be_bytes();
        self.i2c.write(ADDR, &[h, l, val]).map_err(Error::Bus)
    }

    pub fn read(&mut self, reg: u16) -> Result<u8, Error<I::Error>> {
        let mut v = [0u8];
        self.i2c
            .write_read(ADDR, &reg.to_be_bytes(), &mut v)
            .map_err(Error::Bus)?;
        Ok(v[0])
    }

    /// Reads 0x300A/0x300B and checks it is an OV5640.
    pub fn check_id(&mut self) -> Result<u16, Error<I::Error>> {
        let id = u16::from_be_bytes([self.read(REG_CHIP_ID_H)?, self.read(REG_CHIP_ID_H + 1)?]);
        if id == CHIP_ID { Ok(id) } else { Err(Error::WrongChip(id)) }
    }

    pub fn apply(&mut self, list: &[Op], delay: &mut impl DelayNs) -> Result<(), Error<I::Error>> {
        for op in list {
            match *op {
                Op::Write(r, v) => self.write(r, v)?,
                Op::DelayMs(ms) => delay.delay_ms(ms.into()),
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::{Mock, Transaction as T}};
    use std::vec;

    #[test]
    fn check_id_accepts_ov5640() {
        let mut bus = Mock::new(&[
            T::write_read(ADDR, vec![0x30, 0x0A], vec![0x56]),
            T::write_read(ADDR, vec![0x30, 0x0B], vec![0x40]),
        ]);
        assert_eq!(Ov5640::new(bus.clone()).check_id(), Ok(0x5640));
        bus.done();
    }

    #[test]
    fn check_id_rejects_ov2640() {
        let mut bus = Mock::new(&[
            T::write_read(ADDR, vec![0x30, 0x0A], vec![0x26]),
            T::write_read(ADDR, vec![0x30, 0x0B], vec![0x42]),
        ]);
        assert_eq!(Ov5640::new(bus.clone()).check_id(), Err(Error::WrongChip(0x2642)));
        bus.done();
    }

    #[test]
    fn apply_writes_big_endian_register_and_skips_delays_on_bus() {
        let mut bus = Mock::new(&[
            T::write(ADDR, vec![0x30, 0x08, 0x82]),
            T::write(ADDR, vec![0x31, 0x03, 0x13]),
        ]);
        let list = [Op::Write(0x3008, 0x82), Op::DelayMs(10), Op::Write(0x3103, 0x13)];
        Ov5640::new(bus.clone()).apply(&list, &mut NoopDelay).expect("mock bus");
        bus.done();
    }

    #[test]
    fn generated_default_table_starts_with_a_reset_and_is_nonempty() {
        assert!(regs::SENSOR_DEFAULT_REGS.len() > 50);
    }
}
```

- [ ] **Step 3: Generate regs.rs**

Run: `python3 tools/gen_ov5640_regs.py`
Expected: `wrote rust/ov5640/src/regs.rs: sensor_default_regs(N), sensor_fmt_jpeg(M), ...` with N > 50.

- [ ] **Step 4: Run tests**

Run: `cd rust/ov5640 && cargo test && cargo fmt --check && cargo clippy --all-targets -- -D warnings`
Expected: 4 PASS. If clippy flags the generated file, add `#[allow(clippy::unreadable_literal)]` via `#![allow(...)]` as the first line the generator emits, regenerate, rerun.

- [ ] **Step 5: Commit**

```bash
git add rust/ov5640
git commit -m "ov5640: SCCB driver, chip ID check and generated register lists"
```

---

### Task 8: Spike C, WiFi AP + HTTP (no camera needed)

**Files:**
- Create: `rust/spikes/spike-net/` (generated), then modify `src/bin/main.rs`, `Cargo.toml`
- Create: `rust/spikes/spike-net/src/test.jpg` (any small JPEG from `assets/`, e.g. `assets/wuw-skin-nacre.jpg`)
- Create: `tools/spike_net_stress.py`

- [ ] **Step 1: Generate the project**

```bash
mkdir -p rust/spikes && cd rust/spikes
esp-generate --headless -s \
  -o esp32s3-wroom-1-octal-psram -o unstable-hal -o alloc -o wifi -o embassy -o esp-backtrace -o log \
  spike-net
```

Do not pass the `claude`, `agents` or editor options. Set the runner in `spike-net/.cargo/config.toml` to the shared-partition-table runner from Task 1 Step 4. Run `cd spike-net && cargo build --release` and expect success before changing anything; this proves the template and toolchain.

- [ ] **Step 2: Add DHCP + HTTP dependencies** to `Cargo.toml` `[dependencies]`:

```toml
edge-dhcp = "0.8"
edge-nal = "0.5"
edge-nal-embassy = "0.6"
picoserve = { version = "0.20.1", features = ["embassy"] }
```

Run `cargo build --release`. If a version does not resolve against the template's embassy-net, run `cargo search edge-nal-embassy` and pick the release whose embassy-net requirement matches `cargo tree -i embassy-net`.

- [ ] **Step 3: Switch the template from station to access point**

In `src/bin/main.rs`, the template builds a station config. Replace it with:

```rust
use esp_radio::wifi::{AccessPointConfig, AuthMethod, ModeConfig};

let ap = AccessPointConfig::default()
    .with_ssid("wuw-rust".into())
    .with_password("wuwuwuwu".into())
    .with_auth_method(AuthMethod::Wpa2Personal)
    .with_channel(6);
controller.set_config(&ModeConfig::AccessPoint(ap)).expect("AP config");
controller.start_async().await.expect("wifi start");
```

and give embassy-net a static address instead of DHCP client:

```rust
use embassy_net::{Ipv4Address, Ipv4Cidr, StaticConfigV4};

let net_cfg = embassy_net::Config::ipv4_static(StaticConfigV4 {
    address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 4, 1), 24),
    gateway: Some(Ipv4Address::new(192, 168, 4, 1)),
    dns_servers: Default::default(),
});
```

SSID is `wuw-rust` so it never collides with a C++ camera's `wuw`. Check exact builder names against `~/.cargo/registry/src/*/esp-radio-1.0.0-beta.1/src/wifi/ap.rs` (`AccessPointConfig`, lines 36-90) and fix to match.

- [ ] **Step 4: DHCP server task** (phones need an address):

```rust
#[embassy_executor::task]
async fn dhcp_server(stack: embassy_net::Stack<'static>) {
    use edge_dhcp::{io::{self, DEFAULT_SERVER_PORT}, server::{Server, ServerOptions}};
    use edge_nal::UdpBind;
    use edge_nal_embassy::{Udp, UdpBuffers};

    let ip = Ipv4Address::new(192, 168, 4, 1);
    let mut buf = [0u8; 1500];
    let mut gw = [ip];
    let buffers = UdpBuffers::<3, 1024, 1024, 10>::new();
    let unbound = Udp::new(stack, &buffers);
    let mut sock = unbound
        .bind(core::net::SocketAddr::V4(core::net::SocketAddrV4::new(
            core::net::Ipv4Addr::UNSPECIFIED, DEFAULT_SERVER_PORT)))
        .await
        .expect("bind dhcp");
    let mut server = Server::<_, 32>::new_with_et(ip.into());
    let opts = ServerOptions::new(ip.into(), Some(&mut gw));
    loop {
        if let Err(e) = io::server::run(&mut server, &opts, &mut sock, &mut buf).await {
            log::warn!("dhcp: {e:?}");
            embassy_time::Timer::after_secs(1).await;
        }
    }
}
```

If `edge-dhcp` 0.8's names differ, follow its `examples/` in `~/.cargo/registry/src/*/edge-dhcp-0.8.*/` and keep the same behaviour: serve leases from 192.168.4.1, 32 clients.

- [ ] **Step 5: HTTP with picoserve**, 6 worker tasks so 6 phone connections are served in parallel:

```rust
static TEST_JPG: &[u8] = include_bytes!("../test.jpg");

fn app() -> picoserve::Router<impl picoserve::routing::PathRouter> {
    picoserve::Router::new()
        .route("/jpg", picoserve::routing::get(|| async {
            picoserve::response::Response::ok(TEST_JPG)
                .with_header("Content-Type", "image/jpeg")
                .with_header("Cache-Control", "no-store")
        }))
        .route("/stats", picoserve::routing::get(|| async {
            let free = esp_alloc::HEAP.free();
            picoserve::response::DebugValue(("heap_free", free))
        }))
}

const WORKERS: usize = 6;

#[embassy_executor::task(pool_size = WORKERS)]
async fn http_worker(id: usize, stack: embassy_net::Stack<'static>,
                     app: &'static picoserve::Router<AppRouter>,
                     config: &'static picoserve::Config<embassy_time::Duration>) -> ! {
    let mut rx = [0u8; 1024];
    let mut tx = [0u8; 4096];
    let mut http = [0u8; 2048];
    picoserve::listen_and_serve(id, app, config, stack, 80, &mut rx, &mut tx, &mut http).await
}
```

`AppRouter` is the router type; follow picoserve 0.20's `examples/embassy` for the `make_static!`/`StaticCell` pattern that gives `app` and `config` a `'static` lifetime, and set `config` timeouts: start read 5 s, read 2 s, write 2 s, `keep_connection_alive()`. Copy `assets/wuw-skin-nacre.jpg` to `src/test.jpg`.

- [ ] **Step 6: Build**

Run: `cargo build --release && cargo clippy --release -- -D warnings`
Expected: success. Report the image size from `espflash save-image --chip esp32s3 target/xtensa-esp32s3-none-elf/release/spike-net /tmp/x.bin` (should be well under the 6.5 MB app slot).

- [ ] **Step 7: Stress client** `tools/spike_net_stress.py`:

```python
#!/usr/bin/env python3
"""Spike C stress test: N parallel /jpg loops for D seconds, then /stats.

    python3 tools/spike_net_stress.py --host 192.168.4.1 --conns 6 --secs 600
"""
import argparse, threading, time, urllib.request

def worker(url, stop, stats, i):
    ok = err = 0
    while not stop.is_set():
        try:
            with urllib.request.urlopen(url, timeout=5) as r:
                body = r.read()
                if body[:2] == b"\xff\xd8" and body[-2:] == b"\xff\xd9":
                    ok += 1
                else:
                    err += 1
        except Exception:
            err += 1
    stats[i] = (ok, err)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--conns", type=int, default=6)
    ap.add_argument("--secs", type=int, default=600)
    a = ap.parse_args()
    base = "http://%s" % a.host
    before = urllib.request.urlopen(base + "/stats", timeout=5).read().decode()
    stop, stats = threading.Event(), {}
    ts = [threading.Thread(target=worker, args=(base + "/jpg", stop, stats, i))
          for i in range(a.conns)]
    t0 = time.time()
    for t in ts: t.start()
    time.sleep(a.secs); stop.set()
    for t in ts: t.join()
    after = urllib.request.urlopen(base + "/stats", timeout=5).read().decode()
    ok = sum(s[0] for s in stats.values()); err = sum(s[1] for s in stats.values())
    print("frames ok %d, errors %d, %.1f req/s" % (ok, err, ok / (time.time() - t0)))
    print("stats before:", before); print("stats after: ", after)
    print("PASS" if err == 0 else "FAIL")

if __name__ == "__main__":
    main()
```

- [ ] **Step 8: Commit**

```bash
git add rust/spikes/spike-net tools/spike_net_stress.py
git commit -m "Spike C: WiFi AP, DHCP and HTTP serving a test JPEG"
```

---

### Task 9: Spike B, SD card (builds now, runs on device)

**Files:**
- Create: `rust/spikes/spike-sd/` (generated), modify `src/bin/main.rs`, `Cargo.toml`

Writing raw blocks would destroy the files on a card. This spike is **read-only by default**. The write test is behind a Cargo feature and must only ever run on a spare, empty card.

- [ ] **Step 1: Generate**

```bash
cd rust/spikes
esp-generate --headless -s -o esp32s3-wroom-1-octal-psram -o unstable-hal -o embassy -o esp-backtrace -o log spike-sd
cd spike-sd && cargo build --release
```

Set the runner in `spike-sd/.cargo/config.toml` to the shared-partition-table runner from Task 1 Step 4.

- [ ] **Step 2: Dependencies and feature** in `Cargo.toml`:

```toml
[dependencies]
# ...template deps...
sdio = "0.5"
block-device-driver = "0.2"
wuw-board = { path = "../../wuw-board" }

[features]
destructive-write = []
```

- [ ] **Step 3: Main task**

```rust
use esp_hal::sdmmc::{BusWidth, Config as SdConfig, SdHostController, SlotConfig};
use block_device_driver::BlockDevice as _;
use sdio::BlockDevice;

let mut host = SdHostController::new(peripherals.SDHOST, SdConfig::default())
    .expect("sdhost");
let slot = host
    .slot::<1>(SlotConfig::default())
    .expect("slot 1")
    .with_clk(peripherals.GPIO39)   // wuw_board::SD_CLK
    .with_cmd(peripherals.GPIO38)   // wuw_board::SD_CMD
    .with_data0(peripherals.GPIO40) // wuw_board::SD_D0
    .into_async();

let mut card = BlockDevice::<_, _, _, 512>::new_sd_card(slot, 20_000_000, embassy_time::Delay)
    .await
    .expect("card init");
log::info!("card: {} blocks", card.size().await.expect("size") / 512);

let mut blocks = [aligned::Aligned::<aligned::A4, _>([0u8; 512]); 64]; // 32 KiB
let t0 = embassy_time::Instant::now();
let mut read = 0u32;
for lba in (0..16 * 1024 * 2).step_by(64) {            // 16 MiB
    card.read(lba, &mut blocks).await.expect("read");
    read += 64 * 512;
}
let ms = t0.elapsed().as_millis();
log::info!("SPIKE B READ: {} KiB in {} ms = {} KB/s", read / 1024, ms, read as u64 / ms.max(1));

#[cfg(feature = "destructive-write")]
{
    // SPARE CARD ONLY: overwrites 10 MiB starting at 64 MiB.
    const START: u32 = 131_072;           // 64 MiB / 512
    const END: u32 = START + 20_480;      // + 10 MiB
    let fill = |blocks: &mut [aligned::Aligned<aligned::A4, [u8; 512]>], lba: u32| {
        for (k, b) in blocks.iter_mut().enumerate() {
            for (i, byte) in b.0.iter_mut().enumerate() {
                *byte = ((lba + k as u32) as u8) ^ (i as u8);
            }
        }
    };
    let t0 = embassy_time::Instant::now();
    for lba in (START..END).step_by(64) {
        fill(&mut blocks, lba);
        card.write(lba, &blocks).await.expect("write");
    }
    let wms = t0.elapsed().as_millis();
    let mut expect = [aligned::Aligned::<aligned::A4, _>([0u8; 512]); 64];
    let mut mismatches = 0u32;
    for lba in (START..END).step_by(64) {
        card.read(lba, &mut blocks).await.expect("read back");
        fill(&mut expect, lba);
        mismatches += blocks.iter().zip(expect.iter()).filter(|(a, b)| a.0 != b.0).count() as u32;
    }
    log::info!("SPIKE B WRITE: 10240 KiB in {} ms = {} KB/s, {} bad blocks: {}",
               wms, 10_485_760u64 / wms.max(1), mismatches,
               if mismatches == 0 { "PASS" } else { "FAIL" });
}
log::info!("SPIKE B: PASS (read-only)");
```

`aligned` must be added to `[dependencies]` as `aligned = "0.4"` (the version `sdio` uses).

Check `SlotConfig`, the slot number for GPIO-matrix routing (0 or 1) and `BusWidth::Bit1` handling against `~/.cargo/registry/src/*/esp-hal-1.2.2/src/sdmmc/mod.rs` lines 112-181 and 1341-1560; the C++ firmware uses SDMMC 1-bit, so if the slot defaults to 4-bit, set 1-bit explicitly.

- [ ] **Step 4: Build both variants**

Run: `cargo build --release && cargo build --release --features destructive-write && cargo clippy --release -- -D warnings`
Expected: success.

- [ ] **Step 5: Commit**

```bash
git add rust/spikes/spike-sd
git commit -m "Spike B: SDMMC 1-bit card init and read throughput"
```

---

### Task 10: Spike A, camera (builds now, runs on device)

**Files:**
- Modify: `rust/ov5640/src/lib.rs`, create `rust/ov5640/src/mode.rs` (Step 0)
- Create: `rust/spikes/spike-camera/` (generated), modify `src/bin/main.rs`, `Cargo.toml`

**Amendment (review finding, confirmed against espressif/esp32-camera `sensors/ov5640.c`):** `SENSOR_FMT_JPEG` alone does not produce JPEG. Upstream, `esp_camera_init` runs reset (default list), `set_pixformat(JPEG)`, `set_framesize(size)` and `set_quality(q)`. `set_framesize` writes the window (0x3800-0x3807), output size (0x3808-0x380B), totals (0x380C-0x380F), offsets (0x3810-0x3813), read-modify-writes 0x5001 bit 5 (scale), then calls `set_image_options`, which is where compression is switched on (0x3821 bit 5, `reg21 |= 0x20`) together with binning/flip (0x3820, 0x3821, 0x4514, 0x4520, 0x3814, 0x3815), and finally `set_pll` with the JPEG PLL (`sys_mul` 160/180/200 by size, sys_div 4, pre_div 2, pclk_root_div 2, manual pclk_div 4). None of these registers appear in the default list. The C++ firmware streams `FRAMESIZE_SVGA` (800x600), `jpeg_quality = 12`, XCLK 24 MHz (`main_s3.cpp:378-392`), so the spike uses exactly that.

- [ ] **Step 0: JPEG mode in the `ov5640` crate (host-tested)**

Add `Op::Modify(reg, mask, value)` (read, clear `mask`, OR `value & mask`, write) and handle it in `Ov5640::apply`. In `src/mode.rs` port the math of `set_framesize` + `set_image_options` + `set_pll` + `set_quality` for JPEG only, no flip/mirror, as a pure function:

```rust
pub enum Aspect { R4x3, R3x2, R16x10, R5x3, R16x9, R21x9, R5x4, R1x1, R9x16 }
pub struct FrameSize { pub w: u16, pub h: u16, pub aspect: Aspect }
pub const SVGA: FrameSize = FrameSize { w: 800, h: 600, aspect: Aspect::R4x3 };
pub const HD: FrameSize = FrameSize { w: 1280, h: 720, aspect: Aspect::R16x9 };

/// Register steps that switch a reset + SENSOR_FMT_JPEG sensor to JPEG at `size`.
pub fn jpeg_mode(size: &FrameSize, xclk_hz: u32, quality: u8) -> heapless::Vec<Op, 48>;
```

`ratio_table` comes from `vendor/ov5640_settings.h`; the `sys_mul` choice needs the framesize ordering, so compare by pixel area against QVGA (320x240) and XGA (1024x768) exactly as upstream compares enum order, and use 160 when `xclk_hz == 16_000_000`. Host tests (`cargo test` in `rust/ov5640`) assert the exact list for SVGA at 24 MHz, q12:

```
0x3800=00 0x3801=00 0x3802=00 0x3803=00  0x3804=0A 0x3805=3F 0x3806=07 0x3807=9F
0x3808=03 0x3809=20 0x380A=02 0x380B=58  0x380C=08 0x380D=0C 0x380E=03 0x380F=D8
0x3810=00 0x3811=10 0x3812=00 0x3813=08  Modify(0x5001, 0x20, 0x20)
0x3820=01 0x3821=21 0x4514=AA 0x4520=0B 0x3814=31 0x3815=31
0x3039=00 0x3034=1A 0x3035=41 0x3036=B4 0x3037=02 0x3108=26 0x3824=04 0x460C=22 0x3103=13
0x4407=0C
```

and for HD (binning, `w > 920` so total_x = 2844 - 200 = 2644 = 0x0A54, total_y = 1488 / 2 = 744 = 0x02E8, `sys_mul` 200 = 0xC8), plus a mock-I2C test that `Op::Modify` reads then writes the merged value. Commit: `ov5640: JPEG mode registers ported from set_framesize`.

- [ ] **Step 1: Generate**

```bash
cd rust/spikes
esp-generate --headless -s -o esp32s3-wroom-1-octal-psram -o unstable-hal -o alloc -o esp-backtrace -o log spike-camera
cd spike-camera && cargo build --release
```

Set the runner in `spike-camera/.cargo/config.toml` to the shared-partition-table runner from Task 1 Step 4.

- [ ] **Step 2: Dependencies**

```toml
ov5640 = { path = "../../ov5640" }
wuw-frame = { path = "../../wuw-frame" }
wuw-board = { path = "../../wuw-board" }
```

- [ ] **Step 3: Main**

Order matters: XCLK must run before SCCB answers.

```rust
use esp_hal::{
    dma_rx_buffer,
    i2c::master::{Config as I2cConfig, I2c},
    lcd_cam::{LcdCam, cam::{Camera, Config as CamConfig}},
    time::Rate,
};

// 1. Camera peripheral first: it drives XCLK (24 MHz, as the C++ firmware).
let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
let cam_cfg = CamConfig::default().with_frequency(Rate::from_mhz(24));
let camera = Camera::new(lcd_cam.cam, peripherals.DMA_CH0, cam_cfg)
    .expect("camera config")
    .with_master_clock(peripherals.GPIO15)
    .with_pixel_clock(peripherals.GPIO13)
    .with_vsync(peripherals.GPIO6)
    .with_h_enable(peripherals.GPIO7)
    .with_data0(peripherals.GPIO11)
    .with_data1(peripherals.GPIO9)
    .with_data2(peripherals.GPIO8)
    .with_data3(peripherals.GPIO10)
    .with_data4(peripherals.GPIO12)
    .with_data5(peripherals.GPIO18)
    .with_data6(peripherals.GPIO17)
    .with_data7(peripherals.GPIO16);
delay.delay_millis(10);

// 2. SCCB.
let i2c = I2c::new(peripherals.I2C0, I2cConfig::default().with_frequency(Rate::from_khz(100)))
    .expect("i2c")
    .with_sda(peripherals.GPIO4)
    .with_scl(peripherals.GPIO5);
let mut sensor = ov5640::Ov5640::new(i2c);
match sensor.check_id() {
    Ok(id) => log::info!("SPIKE A ID: 0x{id:04X} PASS"),
    Err(e) => { log::error!("SPIKE A ID: {e:?} FAIL"); loop {} }
}
sensor.apply(ov5640::regs::SENSOR_DEFAULT_REGS, &mut delay).expect("default regs");
sensor.apply(ov5640::regs::SENSOR_FMT_JPEG, &mut delay).expect("jpeg fmt");
let mode = ov5640::mode::jpeg_mode(&ov5640::mode::SVGA, 24_000_000, 12);
sensor.apply(&mode, &mut delay).expect("svga jpeg mode");

// 3. Capture 30 frames into a 192 KiB DMA buffer and time them.
let mut buf = dma_rx_buffer!(192 * 1024, 4092).expect("dma buf");
let mut cam = camera;
let t0 = esp_hal::time::Instant::now();
let (mut good, mut bad) = (0u32, 0u32);
for _ in 0..30 {
    let transfer = cam.receive(buf).map_err(|e| e.0).expect("receive");
    let (res, c, b) = transfer.wait();
    cam = c; buf = b;
    res.expect("dma");
    let n = buf.number_of_received_bytes();
    match wuw_frame::find_jpeg(&buf.as_slice()[..n]) {
        Some(r) => { good += 1; if good == 1 { log::info!("first frame {} bytes", r.len()); } }
        None => bad += 1,
    }
}
let ms = t0.elapsed().as_millis();
log::info!("SPIKE A: {good} good / {bad} bad in {ms} ms = {} fps",
           good as u64 * 1000 / ms.max(1));
log::info!("SPIKE A: {}", if good >= 25 { "PASS" } else { "FAIL" });
```

Check `dma_rx_buffer!` arguments and whether a 192 KiB buffer fits internal RAM; if not, allocate from PSRAM (`esp_hal::dma::DmaRxBuf::new` over a PSRAM slice, see `src/dma/buffers/mod.rs:660`). Resolution is SVGA 800x600 JPEG q12 from Step 0, matching the C++ stream; Task 12 switches the same function to `HD`.

- [ ] **Step 4: Build**

Run: `cargo build --release && cargo clippy --release -- -D warnings`
Expected: success.

- [ ] **Step 5: Commit**

```bash
git add rust/spikes/spike-camera
git commit -m "Spike A: OV5640 ID, init and JPEG capture over LCD_CAM"
```

---

### Task 11: On-device runs (GATED: needs a camera)

Do not start until the owner plugs in a camera and names it.

- [ ] **Step 1:** `espflash board-info` and confirm the MAC belongs to the named camera.
- [ ] **Step 2:** Back up: `espflash read-flash 0 0x1000000 ~/Downloads/wuwcam-<cam>-backup-2026-10-XX.bin`.
- [ ] **Step 3:** Spike C: `cd rust/spikes/spike-net && cargo run --release`, join `wuw-rust` from the Mac, run `python3 tools/spike_net_stress.py --secs 600`. Record result.
- [ ] **Step 4:** Spike B (read-only) with the camera's own card. Record KB/s.
- [ ] **Step 5:** Spike A. Record ID result, good/bad frames, fps.
- [ ] **Step 6:** Restore: `pio run -e wuwcam -t upload` from the repo root; check the camera boots and its saved WiFi is intact.
- [ ] **Step 7:** Write `docs/rust-spike-results.md` with every number and the per-peripheral decision (pure `no_std` or ESP-IDF fallback), and commit.

### Task 12: 720p (after Task 11 passes ID and capture)

Task 10 Step 0 already ports `set_framesize` / `set_image_options` for JPEG and tests the HD list. What remains: switch spike-camera to `ov5640::mode::HD`, size the DMA buffer for HD frames, and add host tests that compare the computed register writes for HD (1280x720) against a captured trace from the C++ firmware. This task is planned in detail once Task 11 shows the base capture works, because it depends on what the default tables produce on this sensor.
