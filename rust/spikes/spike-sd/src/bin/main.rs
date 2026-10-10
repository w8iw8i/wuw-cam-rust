//! Spike B: onboard microSD over SDMMC, 1-bit, 20 MHz.
//!
//! READ-ONLY BY DEFAULT. The default build initialises the card, logs its
//! size and times a 16 MiB raw read from block 0. It never writes.
//!
//! The `destructive-write` feature also overwrites 10 MiB of raw blocks at
//! the 64 MiB offset and reads them back. That destroys whatever the file
//! system keeps there: only ever run it on a spare, empty card.
//!
//!     cargo run --release                                # read-only
//!     cargo run --release --features destructive-write   # SPARE CARD ONLY
//!
//! Wiring matches the C++ firmware (`SD_MMC.setPins(39, 38, 40)`, 1-bit):
//! CLK GPIO39, CMD GPIO38, D0 GPIO40, routed through the GPIO matrix to
//! SDMMC slot 1. With only D0 connected the host reports a 1-bit bus, so the
//! card is never switched to 4-bit.
//!
//! Log lines to look for: `SPIKE B READ:`, `SPIKE B WRITE:` (feature only),
//! then `SPIKE B: PASS` or `SPIKE B: FAIL`.

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use aligned::{A4, Aligned};
use block_device_driver::BlockDevice;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::sdmmc::{Config as SdConfig, SdHostController, SlotConfig};
use esp_hal::timer::timg::TimerGroup;
use log::{error, info};
use sdio::MmcError;
use sdio::sd::Card;
use static_cell::ConstStaticCell;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const BLOCK: usize = 512;
/// Blocks per transfer: 64 x 512 B = 32 KiB.
const CHUNK_BLOCKS: usize = 64;
type Chunk = [Aligned<A4, [u8; BLOCK]>; CHUNK_BLOCKS];

/// Card clock. Below 25 MHz, so the card stays in default-speed mode.
const SD_HZ: u32 = 20_000_000;

/// Bytes timed by the read test, starting at block 0.
const READ_BYTES: u32 = 16 * 1024 * 1024;

/// First block of the write test (64 MiB offset).
#[cfg(feature = "destructive-write")]
const WRITE_START: u32 = 64 * 1024 * 1024 / BLOCK as u32;
/// Bytes written and verified by the write test.
#[cfg(feature = "destructive-write")]
const WRITE_BYTES: u32 = 10 * 1024 * 1024;

// The SDMMC pins are typed peripherals (`peripherals.GPIO39`), so the board
// constants cannot select them at run time. Fail the build if they drift.
const _: () = assert!(
    wuw_board::SD_CLK == 39 && wuw_board::SD_CMD == 38 && wuw_board::SD_D0 == 40,
    "spike-sd pins disagree with wuw-board"
);

/// Transfer buffer, allocated once in internal RAM (DMA-capable).
static BUF: ConstStaticCell<Chunk> = ConstStaticCell::new([Aligned([0; BLOCK]); CHUNK_BLOCKS]);
/// Expected contents for the write verification.
#[cfg(feature = "destructive-write")]
static EXPECT: ConstStaticCell<Chunk> = ConstStaticCell::new([Aligned([0; BLOCK]); CHUNK_BLOCKS]);

/// Why the spike failed. `MmcError` has no `PartialEq`, so neither does this.
#[derive(Debug, Clone, Copy)]
enum SpikeError {
    Init(MmcError),
    Size(MmcError),
    TooSmall {
        blocks: u64,
        need: u64,
    },
    Read {
        lba: u32,
        err: MmcError,
    },
    #[cfg(feature = "destructive-write")]
    Write {
        lba: u32,
        err: MmcError,
    },
    #[cfg(feature = "destructive-write")]
    Verify {
        bad_blocks: u32,
    },
}

impl core::fmt::Display for SpikeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Init(e) => write!(f, "card init: {e:?}"),
            Self::Size(e) => write!(f, "card size: {e:?}"),
            Self::TooSmall { blocks, need } => {
                write!(f, "card has {blocks} blocks, test needs {need}")
            }
            Self::Read { lba, err } => write!(f, "read at block {lba}: {err:?}"),
            #[cfg(feature = "destructive-write")]
            Self::Write { lba, err } => write!(f, "write at block {lba}: {err:?}"),
            #[cfg(feature = "destructive-write")]
            Self::Verify { bad_blocks } => write!(f, "{bad_blocks} blocks read back wrong"),
        }
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    // generator version: 1.4.0
    // generator parameters: -o esp32s3 -o esp32s3-wroom-1-octal-psram -o unstable-hal -o embassy -o esp-backtrace -o log

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The following pins are used to bootstrap the chip. They are available
    // for use, but check the datasheet of the module for more information on them.
    // - GPIO0
    // - GPIO3
    // - GPIO45
    // - GPIO46
    // These GPIO pins are in use by some feature of the module and should not be used.
    let _gpio27 = peripherals.GPIO27;
    let _gpio28 = peripherals.GPIO28;
    let _gpio29 = peripherals.GPIO29;
    let _gpio30 = peripherals.GPIO30;
    let _gpio31 = peripherals.GPIO31;
    let _gpio32 = peripherals.GPIO32;
    let _gpio33 = peripherals.GPIO33;
    let _gpio34 = peripherals.GPIO34;
    let _gpio35 = peripherals.GPIO35;
    let _gpio36 = peripherals.GPIO36;
    let _gpio37 = peripherals.GPIO37;

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    #[cfg(feature = "destructive-write")]
    log::warn!("destructive-write build: 10 MiB at the 64 MiB offset will be overwritten");

    let host = SdHostController::new(peripherals.SDHOST, SdConfig::default())
        .expect("default SDMMC config is valid");
    let slot = host
        .slot::<1>(SlotConfig::default())
        .expect("slot 1 is requested only once")
        .with_clk(peripherals.GPIO39) // wuw_board::SD_CLK
        .with_cmd(peripherals.GPIO38) // wuw_board::SD_CMD
        .with_data0(peripherals.GPIO40) // wuw_board::SD_D0
        .into_async();

    let buf = BUF.take();
    #[cfg(feature = "destructive-write")]
    let expect = EXPECT.take();

    let result = async {
        let mut card = sdio::BlockDevice::<Card, _, _, BLOCK>::new_sd_card(slot, SD_HZ, Delay)
            .await
            .map_err(SpikeError::Init)?;
        let blocks = card.size().await.map_err(SpikeError::Size)? / BLOCK as u64;
        info!(
            "card: {} blocks ({} MiB), bus 1-bit at {} kHz",
            blocks,
            blocks / 2048,
            card.freq() / 1000
        );

        read_test(&mut card, buf, blocks).await?;

        #[cfg(feature = "destructive-write")]
        write_test(&mut card, buf, expect, blocks).await?;

        Ok::<(), SpikeError>(())
    }
    .await;

    match result {
        #[cfg(not(feature = "destructive-write"))]
        Ok(()) => info!("SPIKE B: PASS (read-only)"),
        #[cfg(feature = "destructive-write")]
        Ok(()) => info!("SPIKE B: PASS (read + write)"),
        Err(e) => error!("SPIKE B: FAIL ({})", e),
    }

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

/// Times a sequential 16 MiB read from block 0 in 32 KiB transfers.
async fn read_test<D>(card: &mut D, buf: &mut Chunk, blocks: u64) -> Result<(), SpikeError>
where
    D: BlockDevice<BLOCK, Align = A4, Error = MmcError>,
{
    let end = READ_BYTES / BLOCK as u32;
    if blocks < u64::from(end) {
        return Err(SpikeError::TooSmall {
            blocks,
            need: u64::from(end),
        });
    }

    let t0 = Instant::now();
    for lba in (0..end).step_by(CHUNK_BLOCKS) {
        card.read(lba, buf)
            .await
            .map_err(|err| SpikeError::Read { lba, err })?;
        if lba == 0 {
            // Informational: an MBR or FAT boot sector ends in 55 AA.
            let sig = buf[0][510] == 0x55 && buf[0][511] == 0xAA;
            info!("block 0 boot signature 55AA: {}", sig);
        }
    }
    let ms = t0.elapsed().as_millis();
    info!(
        "SPIKE B READ: {} KiB in {} ms = {} KB/s",
        READ_BYTES / 1024,
        ms,
        u64::from(READ_BYTES) / ms.max(1)
    );
    Ok(())
}

/// SPARE CARD ONLY: overwrites 10 MiB at the 64 MiB offset, then reads it
/// back and compares.
#[cfg(feature = "destructive-write")]
#[allow(
    clippy::large_stack_frames,
    reason = "the frame is this future, holding the sdio write and read futures; it is awaited \
    inside main's task, whose storage is static, so it never lands on a thread stack"
)]
async fn write_test<D>(
    card: &mut D,
    buf: &mut Chunk,
    expect: &mut Chunk,
    blocks: u64,
) -> Result<(), SpikeError>
where
    D: BlockDevice<BLOCK, Align = A4, Error = MmcError>,
{
    let end = WRITE_START + WRITE_BYTES / BLOCK as u32;
    if blocks < u64::from(end) {
        return Err(SpikeError::TooSmall {
            blocks,
            need: u64::from(end),
        });
    }

    let t0 = Instant::now();
    for lba in (WRITE_START..end).step_by(CHUNK_BLOCKS) {
        fill(buf, lba);
        card.write(lba, buf)
            .await
            .map_err(|err| SpikeError::Write { lba, err })?;
    }
    let ms = t0.elapsed().as_millis();

    let mut bad_blocks = 0u32;
    for lba in (WRITE_START..end).step_by(CHUNK_BLOCKS) {
        card.read(lba, buf)
            .await
            .map_err(|err| SpikeError::Read { lba, err })?;
        fill(expect, lba);
        let bad = buf
            .iter()
            .zip(expect.iter())
            .filter(|(a, b)| a != b)
            .count();
        bad_blocks += u32::try_from(bad).expect("at most CHUNK_BLOCKS per chunk");
    }

    info!(
        "SPIKE B WRITE: {} KiB in {} ms = {} KB/s, {} bad blocks: {}",
        WRITE_BYTES / 1024,
        ms,
        u64::from(WRITE_BYTES) / ms.max(1),
        bad_blocks,
        if bad_blocks == 0 { "PASS" } else { "FAIL" }
    );
    if bad_blocks == 0 {
        Ok(())
    } else {
        Err(SpikeError::Verify { bad_blocks })
    }
}

/// Test pattern that differs per block and per byte, so a block written to
/// the wrong address or a shifted transfer both show up as mismatches.
#[cfg(feature = "destructive-write")]
fn fill(chunk: &mut Chunk, first_lba: u32) {
    for (lba, block) in (first_lba..).zip(chunk.iter_mut()) {
        let seed = lba.wrapping_mul(0x9E37_79B9).to_le_bytes();
        for (i, byte) in block.iter_mut().enumerate() {
            *byte = seed[i % 4] ^ (i as u8);
        }
    }
}
