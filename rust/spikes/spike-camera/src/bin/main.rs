//! Spike A: OV5640 ID, init and JPEG capture over LCD_CAM.
//!
//! Proves that the Rust stack can drive the camera the way the C++ firmware
//! does: LCD_CAM generates XCLK (24 MHz) and receives the 8-bit DVP stream,
//! the `ov5640` crate identifies the sensor over SCCB and switches it to
//! SVGA (800x600) JPEG at quality 12, and every captured frame is checked for
//! a complete JPEG (SOI..EOI) with `wuw-frame`.
//!
//! Data path, as esp32-camera does it: GDMA streams into a small ring in
//! internal RAM, the CPU copies each frame (VSYNC marks the frame end) into
//! one frame buffer in PSRAM, and the JPEG check runs on that copy.
//!
//! Log lines to look for: `SPIKE A ID:` (sensor answered with the right chip
//! ID) and `SPIKE A:` (frame counts, fps, PASS/FAIL).

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::alloc::Layout;

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::dma_rx_stream_buffer;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::lcd_cam::LcdCam;
use esp_hal::lcd_cam::cam::{Camera, Config as CamConfig};
use esp_hal::main;
use esp_hal::psram::{PsramConfig, PsramMode};
use esp_hal::time::{Duration, Instant, Rate};
use log::{error, info};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

/// Sensor master clock, as the C++ firmware (`main_s3.cpp`).
const XCLK_HZ: u32 = 24_000_000;
const SCCB_KHZ: u32 = 100;
const JPEG_QUALITY: u8 = 12;

/// Frames to capture after the first frame boundary, and how many must pass.
const FRAMES: u32 = 30;
const PASS_GOOD: u32 = 25;

/// One assembled frame, in PSRAM. SVGA q12 frames are tens of KiB.
const FRAME_CAP: usize = 192 * 1024;
/// GDMA ring in internal RAM, split into `DMA_RING / DMA_CHUNK` descriptors.
const DMA_RING: usize = 64 * 1024;
const DMA_CHUNK: usize = 2048;
/// A frame boundary must arrive within this time, or the run fails.
const FRAME_TIMEOUT: Duration = Duration::from_millis(1000);

// The GPIOs below are typed peripherals, so they cannot be read from
// `wuw-board`; this fails the build if the pin map moves one of them.
const _: () = {
    assert!(wuw_board::CAM_XCLK == 15);
    assert!(wuw_board::CAM_PCLK == 13);
    assert!(wuw_board::CAM_VSYNC == 6);
    assert!(wuw_board::CAM_HREF == 7);
    assert!(wuw_board::CAM_D0 == 11);
    assert!(wuw_board::CAM_D1 == 9);
    assert!(wuw_board::CAM_D2 == 8);
    assert!(wuw_board::CAM_D3 == 10);
    assert!(wuw_board::CAM_D4 == 12);
    assert!(wuw_board::CAM_D5 == 18);
    assert!(wuw_board::CAM_D6 == 17);
    assert!(wuw_board::CAM_D7 == 16);
    assert!(wuw_board::CAM_SIOD == 4);
    assert!(wuw_board::CAM_SIOC == 5);
};

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.4.0
    // generator parameters: -o esp32s3 -o esp32s3-wroom-1-octal-psram -o unstable-hal -o alloc -o esp-backtrace -o log

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

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    // The N16R8 module has 8 MB of octal PSRAM; the frame buffer lives there.
    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        PsramConfig {
            mode: PsramMode::OctalSpi,
            ..Default::default()
        }
    );
    let frame = psram_frame_buffer();

    let mut delay = Delay::new();

    // 1. Camera peripheral first: it drives XCLK, and the sensor does not
    //    answer on SCCB without a clock.
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
    let cam_cfg = CamConfig::default().with_frequency(Rate::from_hz(XCLK_HZ));
    let camera = Camera::new(lcd_cam.cam, peripherals.DMA_CH0, cam_cfg)
        .expect("24 MHz is within the LCD_CAM clock divider range")
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

    // 2. SCCB: identify, reset to defaults, then JPEG at SVGA.
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(SCCB_KHZ)),
    )
    .expect("100 kHz is a valid I2C frequency")
    .with_sda(peripherals.GPIO4)
    .with_scl(peripherals.GPIO5);
    let mut sensor = ov5640::Ov5640::new(i2c);
    match sensor.check_id() {
        Ok(id) => info!("SPIKE A ID: 0x{id:04X} PASS"),
        Err(e) => {
            error!("SPIKE A ID: {e:?} FAIL");
            halt(&delay);
        }
    }
    let mode = ov5640::mode::jpeg_mode(&ov5640::mode::SVGA, XCLK_HZ, JPEG_QUALITY);
    for (name, list) in [
        ("default regs", ov5640::regs::SENSOR_DEFAULT_REGS),
        ("JPEG format", ov5640::regs::SENSOR_FMT_JPEG),
        ("SVGA JPEG mode", &mode[..]),
    ] {
        if let Err(e) = sensor.apply(list, &mut delay) {
            error!("SPIKE A: {name}: {e:?} FAIL");
            halt(&delay);
        }
    }
    info!("SPIKE A: sensor set to SVGA JPEG q{JPEG_QUALITY}");

    // 3. Stream frames and check each one.
    let ring = dma_rx_stream_buffer!(DMA_RING, DMA_CHUNK);
    let mut transfer = camera
        .receive(ring)
        .map_err(|e| e.0)
        .expect("a fresh stream buffer is a valid DMA target");

    // The frame in flight when the stream starts is partial: drop everything
    // up to the first frame boundary, then start the clock.
    let mut synced = false;
    let mut len = 0usize;
    let mut overflow = false;
    let (mut good, mut bad) = (0u32, 0u32);
    let (mut min_len, mut max_len) = (usize::MAX, 0usize);
    let mut t0 = Instant::now();
    let mut last_eof = Instant::now();

    while good + bad < FRAMES {
        let (data, eof) = transfer.peek_until_eof();
        let n = data.len();
        if n == 0 && !eof {
            if transfer.is_done() {
                error!("SPIKE A: DMA ring overran (CPU fell behind the sensor)");
                break;
            }
            if last_eof.elapsed() > FRAME_TIMEOUT {
                error!("SPIKE A: no frame boundary for {FRAME_TIMEOUT} (VSYNC/PCLK?)");
                break;
            }
            continue;
        }
        if synced && !overflow {
            match frame.get_mut(len..len + n) {
                Some(dst) => {
                    dst.copy_from_slice(data);
                    len += n;
                }
                None => overflow = true,
            }
        }
        transfer.consume(n);
        if !eof {
            continue;
        }

        last_eof = Instant::now();
        if !synced {
            synced = true;
            t0 = last_eof;
        } else if overflow {
            bad += 1;
            error!("SPIKE A: frame larger than {FRAME_CAP} bytes");
        } else if let Some(r) = wuw_frame::find_jpeg(&frame[..len]) {
            good += 1;
            if good == 1 {
                info!("SPIKE A: first frame {} bytes ({len} received)", r.len());
            }
            min_len = min_len.min(r.len());
            max_len = max_len.max(r.len());
        } else {
            bad += 1;
        }
        len = 0;
        overflow = false;
    }
    let ms = (last_eof - t0).as_millis();
    let (_camera, _ring) = transfer.stop();

    // Good JPEGs per second, to one decimal.
    let deci_fps = u64::from(good) * 10_000 / ms.max(1);
    info!(
        "SPIKE A: {good} good / {bad} bad in {ms} ms = {}.{} fps",
        deci_fps / 10,
        deci_fps % 10
    );
    if good > 0 {
        info!("SPIKE A: JPEG sizes {min_len}..={max_len} bytes");
    }
    info!(
        "SPIKE A: {}",
        if good >= PASS_GOOD { "PASS" } else { "FAIL" }
    );
    halt(&delay)
}

/// Allocates the frame buffer once, from PSRAM only.
fn psram_frame_buffer() -> &'static mut [u8] {
    let layout = Layout::from_size_align(FRAME_CAP, 4).expect("non-zero size, power-of-two align");
    // SAFETY: `layout` has a non-zero size, as `alloc_caps` requires.
    let ptr =
        unsafe { esp_alloc::HEAP.alloc_caps(esp_alloc::MemoryCapability::External.into(), layout) };
    assert!(
        !ptr.is_null(),
        "PSRAM missing or too small for the frame buffer"
    );
    // SAFETY: `ptr` is a fresh, exclusive allocation of `FRAME_CAP` bytes that
    // is never freed, so a `'static` slice over it is sound once initialised.
    unsafe {
        ptr.write_bytes(0, FRAME_CAP);
        core::slice::from_raw_parts_mut(ptr, FRAME_CAP)
    }
}

fn halt(delay: &Delay) -> ! {
    loop {
        delay.delay_millis(1000);
    }
}
