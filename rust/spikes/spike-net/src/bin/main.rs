//! Spike C: Wi-Fi access point + DHCP server + HTTP server.
//!
//! Proves that the Rust stack can stand in for the C++ firmware's network
//! side: the chip brings up its own access point (`wuw-rust`), hands out
//! addresses to phones over DHCP, and serves a JPEG to several clients in
//! parallel. The JPEG is a static test image; no camera is involved.
//!
//! - `GET /jpg`   the test JPEG (`Cache-Control: no-store`)
//! - `GET /stats` heap and uptime as JSON
//!
//! Stress test from a laptop joined to the AP:
//! `python3 tools/spike_net_stress.py --host 192.168.4.1 --conns 6 --secs 600`
//!
//! PSRAM is not used by this spike: both heap regions below are internal
//! SRAM, and all network buffers are statics in internal RAM.

#![no_std]
#![no_main]
// `AppBuilder::PathRouter = impl PathRouter`, the pattern picoserve uses for a
// `'static` router. The esp toolchain is a nightly toolchain.
#![feature(impl_trait_in_assoc_type)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::fmt::Write as _;
use core::net::{Ipv4Addr, SocketAddr, SocketAddrV4};

use edge_dhcp::io::{DEFAULT_SERVER_PORT, server::run as run_dhcp_server};
use edge_dhcp::server::{Server as DhcpServer, ServerOptions};
use edge_nal::UdpBind;
use edge_nal_embassy::{Udp, UdpBuffers};
use embassy_executor::Spawner;
use embassy_net::{Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::ap::{AccessPointConfig, EventInfo};
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config as WifiConfig, ControllerConfig, Interface, WifiController,
};
use log::{info, warn};
use picoserve::response::Response;
use picoserve::routing::{PathRouter, get};
use picoserve::{AppBuilder, AppRouter, Router, Timeouts};
use static_cell::{ConstStaticCell, StaticCell};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

/// Must differ from `wuw`, the SSID the C++ cameras use.
const AP_SSID: &str = "wuw-rust";
const AP_PASSWORD: &str = "wuwuwuwu";
const AP_CHANNEL: u8 = 6;

/// The AP's own address; also the gateway handed out over DHCP.
const AP_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 4, 1);
const AP_PREFIX_LEN: u8 = 24;

/// Lease table size. Leases come from 192.168.4.50..=200 (edge-dhcp default).
const DHCP_MAX_LEASES: usize = 32;

const HTTP_PORT: u16 = 80;
/// One picoserve worker serves one connection at a time.
const WEB_TASK_POOL_SIZE: usize = 6;

/// One TCP socket per web worker, one UDP socket for DHCP, one spare.
const STACK_SOCKETS: usize = WEB_TASK_POOL_SIZE + 2;

/// Test image served by `/jpg`. Lives in flash; no per-request copy.
static TEST_JPEG: &[u8] = include_bytes!("../test.jpg");

/// picoserve timeouts: 5 s for a request to start, 2 s to finish reading
/// it, 2 s per write. Keep-alive on; an idle kept-alive connection is
/// dropped after 1 s (picoserve's default) so it frees its worker.
static HTTP_CONFIG: picoserve::Config = picoserve::Config::new(Timeouts {
    start_read_request: Duration::from_secs(5),
    persistent_start_read_request: Duration::from_secs(1),
    read_request: Duration::from_secs(2),
    write: Duration::from_secs(2),
})
.keep_connection_alive();

static APP: StaticCell<AppRouter<AppProps>> = StaticCell::new();

// Large buffers live in const-initialised statics (.bss) rather than in task
// futures, so they are never built on a stack and moved.
static STACK_RESOURCES: ConstStaticCell<StackResources<STACK_SOCKETS>> =
    ConstStaticCell::new(StackResources::new());
static DHCP_BUFFERS: ConstStaticCell<DhcpBuffers> = ConstStaticCell::new(DhcpBuffers::new());
static WEB_BUFFERS: ConstStaticCell<[WebBuffers; WEB_TASK_POOL_SIZE]> =
    ConstStaticCell::new([const { WebBuffers::new() }; WEB_TASK_POOL_SIZE]);

/// UDP socket storage and packet buffer for the DHCP server.
struct DhcpBuffers {
    udp: UdpBuffers<1, 1024, 1024, 4>,
    packet: [u8; 1500],
}

impl DhcpBuffers {
    #[allow(
        clippy::large_stack_frames,
        reason = "only evaluated at compile time, to const-initialise a static"
    )]
    const fn new() -> Self {
        Self {
            udp: UdpBuffers::new(),
            packet: [0; 1500],
        }
    }
}

/// Per-worker TCP socket and HTTP buffers. The 4 KiB TX buffer lets a
/// worker push a large slice of the JPEG per write.
struct WebBuffers {
    tcp_rx: [u8; 1024],
    tcp_tx: [u8; 4096],
    http: [u8; 2048],
}

impl WebBuffers {
    #[allow(
        clippy::large_stack_frames,
        reason = "only evaluated at compile time, to const-initialise a static"
    )]
    const fn new() -> Self {
        Self {
            tcp_rx: [0; 1024],
            tcp_tx: [0; 4096],
            http: [0; 2048],
        }
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.4.0
    // generator parameters: -o esp32s3 -o esp32s3-wroom-1-octal-psram -o unstable-hal -o alloc -o wifi -o embassy -o esp-backtrace -o log

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
    // Extra headroom for the Wi-Fi driver's dynamic TX/RX buffers while six
    // clients pull images at once.
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    info!("Embassy initialized!");

    let ap_config = AccessPointConfig::default()
        .with_ssid(AP_SSID.try_into().expect("AP SSID fits in 32 bytes"))
        .with_channel(AP_CHANNEL)
        .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
            AP_PASSWORD
                .try_into()
                .expect("AP password fits in 64 bytes"),
        ));
    let controller_config =
        ControllerConfig::default().with_initial_config(WifiConfig::AccessPoint(ap_config));
    // The AP starts here: `WifiController::new` applies the initial config via
    // `set_config`, which switches the mode from NULL (set by the driver init)
    // to AP and calls `esp_wifi_start` (esp-radio 1.0.0-beta.1). No explicit
    // start call is needed.
    let wifi_controller = WifiController::new(peripherals.WIFI, controller_config)
        .expect("Failed to initialize Wi-Fi controller");
    let ap_interface = Interface::access_point();

    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(AP_IP, AP_PREFIX_LEN),
        gateway: Some(AP_IP),
        dns_servers: Default::default(),
    });
    // Seed for embassy-net (TCP sequence numbers, ephemeral ports). It is read
    // after `WifiController::new`, which registers the radio as the RNG's
    // entropy source; read any earlier it would be weak. Either way it is good
    // enough for a spike on a private AP.
    let rng = Rng::new();
    let seed = (u64::from(rng.random()) << 32) | u64::from(rng.random());
    let (stack, runner) = embassy_net::new(ap_interface, net_config, STACK_RESOURCES.take(), seed);

    spawner.spawn(tasks::net_task(runner).expect("net task spawns once"));
    spawner.spawn(tasks::dhcp_task(stack, DHCP_BUFFERS.take()).expect("DHCP task spawns once"));

    let app: &'static AppRouter<AppProps> = APP.init(AppProps.build_app());
    for (task_id, buffers) in WEB_BUFFERS.take().iter_mut().enumerate() {
        spawner.spawn(
            tasks::web_task(task_id, stack, app, &HTTP_CONFIG, buffers)
                .expect("web task pool has a free slot"),
        );
    }

    info!(
        "AP '{}' on channel {}, http://{}/jpg",
        AP_SSID, AP_CHANNEL, AP_IP
    );

    // Keep the controller alive (dropping it stops Wi-Fi) and log stations.
    loop {
        match wifi_controller
            .wait_for_access_point_connected_event_async()
            .await
        {
            Ok(EventInfo::Connected(station)) => {
                info!(
                    "station joined: {:02x?} (aid {}), heap free {}",
                    station.mac,
                    station.aid,
                    esp_alloc::HEAP.free()
                );
            }
            Ok(EventInfo::Disconnected(station)) => {
                info!(
                    "station left: {:02x?} ({:?}), heap free {}",
                    station.mac,
                    station.reason,
                    esp_alloc::HEAP.free()
                );
            }
            Err(err) => {
                warn!("Wi-Fi event error: {:?}", err);
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }
}

/// The long-running tasks spawned from `main`.
mod tasks {
    // The macro-generated constructors build each task future (1.5 to 3 KiB:
    // picoserve's connection state, the DHCP lease table) before it is moved
    // into the executor's static task pool. Big buffers are already statics.
    #![allow(
        clippy::large_stack_frames,
        reason = "task futures are built once at spawn time, then live in the static task pool"
    )]

    use super::*;

    /// Drives the embassy-net stack.
    #[embassy_executor::task]
    pub(super) async fn net_task(mut runner: Runner<'static, Interface>) -> ! {
        runner.run().await
    }

    /// DHCP server for the AP's /24. Never panics: on error it logs, waits 1 s
    /// and starts again, keeping the lease table.
    #[embassy_executor::task]
    pub(super) async fn dhcp_task(stack: Stack<'static>, buffers: &'static mut DhcpBuffers) -> ! {
        let DhcpBuffers { udp, packet } = buffers;
        let udp = Udp::new(stack, udp);
        let mut gateway_buf = [AP_IP];
        // Gateway = AP_IP, netmask 255.255.255.0, no DNS servers.
        let options = ServerOptions::new(AP_IP, Some(&mut gateway_buf));
        let mut server = DhcpServer::<_, DHCP_MAX_LEASES>::new_with_et(AP_IP);
        let listen = SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::UNSPECIFIED,
            DEFAULT_SERVER_PORT,
        ));

        loop {
            match udp.bind(listen).await {
                Ok(mut socket) => {
                    if let Err(err) =
                        run_dhcp_server(&mut server, &options, &mut socket, packet).await
                    {
                        warn!("DHCP server error: {:?}", err);
                    }
                }
                Err(err) => warn!("DHCP bind error: {:?}", err),
            }
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    /// One HTTP worker; `WEB_TASK_POOL_SIZE` of them listen on the same port.
    #[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
    pub(super) async fn web_task(
        task_id: usize,
        stack: Stack<'static>,
        app: &'static AppRouter<AppProps>,
        config: &'static picoserve::Config,
        buffers: &'static mut WebBuffers,
    ) -> ! {
        let WebBuffers {
            tcp_rx,
            tcp_tx,
            http,
        } = buffers;

        picoserve::Server::new(app, config, http)
            .listen_and_serve(task_id, stack, HTTP_PORT, tcp_rx, tcp_tx)
            .await
            .into_never()
    }
}

struct AppProps;

impl AppBuilder for AppProps {
    type PathRouter = impl PathRouter;

    fn build_app(self) -> Router<Self::PathRouter> {
        Router::new()
            .route(
                "/jpg",
                get(async || {
                    Response::ok(TEST_JPEG)
                        .with_content_type("image/jpeg")
                        .with_header("Cache-Control", "no-store")
                }),
            )
            .route(
                "/stats",
                get(async || {
                    Response::ok(stats_json())
                        .with_content_type("application/json")
                        .with_header("Cache-Control", "no-store")
                }),
            )
    }
}

/// `{"heap_free":..,"heap_used":..,"uptime_ms":..}` without touching the heap.
fn stats_json() -> heapless::String<128> {
    let mut body = heapless::String::new();
    // 128 bytes always fits the keys plus three 20-digit numbers.
    let _ = write!(
        body,
        "{{\"heap_free\":{},\"heap_used\":{},\"uptime_ms\":{}}}",
        esp_alloc::HEAP.free(),
        esp_alloc::HEAP.used(),
        Instant::now().as_millis()
    );
    body
}
