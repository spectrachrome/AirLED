#![no_std]
#![no_main]

use core::net::Ipv4Addr;

use defmt::info;
use edge_dhcp::server::{Server as DhcpServer, ServerOptions as DhcpServerOptions};
use edge_dhcp::{Options as DhcpOptions, Packet as DhcpPacket};
use embassy_net::{Ipv4Address, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4, tcp::TcpSocket};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rmt::Rmt;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::Ws2812SmartLeds;
use esp_radio::wifi::{
    AccessPointConfig, ModeConfig, WifiApState, ap_state,
};
use panic_rtt_target as _;
use smart_leds::{SmartLedsWrite, brightness, RGB8};
use xiao_drone_led_controller::pattern::{Pattern, RippleEffect};
use xiao_drone_led_controller::state::STATE;
use static_cell::StaticCell;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();
use alloc::string::ToString;

/// Number of WS2812 LEDs in the strip.
const NUM_LEDS: usize = 150;

/// RMT buffer size: 24 bits per LED (8 per channel * 3 channels) + 1 end marker.
const BUFFER_SIZE: usize = NUM_LEDS * 24 + 1;

/// Wi-Fi AP SSID.
const WIFI_SSID: &str = "XIAO-LED-Controller";

/// AP static IP address.
const AP_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);

/// Simple HTML test page served by the AP.
const TEST_PAGE: &str = r#"<!DOCTYPE html>
<html>
<head><title>XIAO LED Controller</title></head>
<body>
<h1>XIAO LED Controller</h1>
<p>Wi-Fi AP is running. LED strip is active.</p>
<p>LEDs configured: 150</p>
</body>
</html>"#;

#[esp_hal::main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    info!("Initializing...");

    // Start the RTOS scheduler (required before esp-radio init)
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // --- LED setup ---
    let rmt = Rmt::new(peripherals.RMT, esp_hal::time::Rate::from_mhz(80))
        .expect("failed to initialize RMT");

    let led_driver =
        Ws2812SmartLeds::<BUFFER_SIZE, _>::new(rmt.channel0, peripherals.GPIO2)
            .expect("failed to create WS2812 driver");

    // --- Wi-Fi setup (scheduler is now running) ---
    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio_controller: &'static esp_radio::Controller<'static> =
        RADIO.init(esp_radio::init().expect("failed to init esp-radio"));

    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_controller, peripherals.WIFI, esp_radio::wifi::Config::default())
            .expect("failed to create wifi");

    let ap_config = AccessPointConfig::default()
        .with_ssid(WIFI_SSID.to_string())
        .with_channel(6);

    wifi_controller
        .set_config(&ModeConfig::AccessPoint(ap_config))
        .expect("failed to set wifi config");

    wifi_controller.start().expect("failed to start wifi");

    info!("Wi-Fi AP starting...");

    // --- Network stack ---
    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(AP_IP, 24),
        gateway: Some(AP_IP),
        dns_servers: Default::default(),
    });

    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        interfaces.ap,
        net_config,
        RESOURCES.init(StackResources::new()),
        0, // random seed — no true randomness needed for AP
    );

    // Start embassy executor
    static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
    executor.run(move |spawner| {
        spawner.must_spawn(led_task(led_driver));
        spawner.must_spawn(net_task(runner));
        spawner.must_spawn(web_server(stack));
        spawner.must_spawn(dhcp_server(stack));
        spawner.must_spawn(wifi_keepalive(wifi_controller));
    })
}

/// Keeps the Wi-Fi controller alive and logs AP state changes.
#[embassy_executor::task]
async fn wifi_keepalive(wifi_controller: esp_radio::wifi::WifiController<'static>) {
    // Wait for AP to start
    while ap_state() != WifiApState::Started {
        Timer::after(Duration::from_millis(100)).await;
    }
    info!("Wi-Fi AP started on channel 6");

    // Keep wifi controller alive (dropping it stops wifi)
    let _controller = wifi_controller;
    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}

/// Estimate strip current and clamp brightness to stay within the mA budget.
///
/// Model: each WS2812B draws ~1 mA idle + 8 mA per color channel at full PWM.
/// Current scales linearly with the global brightness byte, so we solve for the
/// exact brightness value that hits the budget — no iteration needed.
fn clamp_brightness(buf: &[RGB8], brightness: u8, max_ma: u32) -> u8 {
    // Sum (R + G + B) across all LEDs for the channel-current estimate.
    let channel_sum: u32 = buf
        .iter()
        .map(|c| c.r as u32 + c.g as u32 + c.b as u32)
        .sum();

    // total_ma = (idle + channel_draw) * brightness / 255
    //   idle         = num_leds * 1 mA
    //   channel_draw = channel_sum * 8 / 255 mA  (at full brightness)
    let num = buf.len() as u32;
    let raw_ma_x255 = num * 255 + channel_sum * 8; // numerator, denominator is 255
    let total_ma_x255 = raw_ma_x255 * brightness as u32; // denominator is 255*255

    // Compare: total_ma_x255 / (255*255) vs max_ma
    let limit = max_ma * 255 * 255;
    if total_ma_x255 <= limit {
        brightness
    } else {
        // clamped = brightness * max_ma / total_ma
        //         = brightness * limit / total_ma_x255
        (brightness as u32 * limit / total_ma_x255) as u8
    }
}

/// Drives the WS2812 LED strip using the active pattern.
#[embassy_executor::task]
async fn led_task(mut driver: Ws2812SmartLeds<'static, BUFFER_SIZE, esp_hal::Blocking>) {
    let mut pattern = RippleEffect::new(0xDEAD_BEEF);
    let mut buf = [RGB8 { r: 0, g: 0, b: 0 }; NUM_LEDS];

    loop {
        pattern.render(&mut buf);

        let state = STATE.lock().await;
        let led_brightness = state.brightness;
        let max_ma = state.max_current_ma;
        drop(state);

        let clamped = clamp_brightness(&buf, led_brightness, max_ma);

        if let Err(e) = driver.write(brightness(buf.iter().copied(), clamped)) {
            defmt::warn!("LED write error: {}", defmt::Debug2Format(&e));
        }

        Timer::after(Duration::from_millis(20)).await;
    }
}

/// Runs the embassy-net network stack.
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, esp_radio::wifi::WifiDevice<'static>>) {
    runner.run().await;
}

/// DHCP server assigning IPs to clients connecting to the AP.
#[embassy_executor::task]
async fn dhcp_server(stack: Stack<'static>) {
    // Wait until the stack is configured
    while !stack.is_config_up() {
        Timer::after(Duration::from_millis(100)).await;
    }

    let mut rx_meta = [PacketMetadata::EMPTY; 2];
    let mut rx_buffer = [0u8; 600];
    let mut tx_meta = [PacketMetadata::EMPTY; 2];
    let mut tx_buffer = [0u8; 600];

    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    socket.bind(67).expect("failed to bind DHCP server socket");

    info!("DHCP server running on port 67");

    let server_ip = Ipv4Addr::new(192, 168, 4, 1);
    let mut gw_buf = [Ipv4Addr::UNSPECIFIED; 1];
    let server_options = DhcpServerOptions::new(server_ip, Some(&mut gw_buf));

    // Up to 8 concurrent leases
    let mut server = DhcpServer::<_, 8>::new_with_et(server_ip);
    server.range_start = Ipv4Addr::new(192, 168, 4, 50);
    server.range_end = Ipv4Addr::new(192, 168, 4, 200);

    let mut buf = [0u8; 600];

    loop {
        let (len, _meta) = match socket.recv_from(&mut buf).await {
            Ok(result) => result,
            Err(_) => continue,
        };

        let request = match DhcpPacket::decode(&buf[..len]) {
            Ok(pkt) => pkt,
            Err(e) => {
                defmt::warn!("DHCP decode error: {}", defmt::Debug2Format(&e));
                continue;
            }
        };

        let mut opt_buf = DhcpOptions::buf();

        if let Some(reply) = server.handle_request(&mut opt_buf, &server_options, &request) {
            match reply.encode(&mut buf) {
                Ok(encoded) => {
                    // DHCP replies go to broadcast 255.255.255.255:68
                    let dest = (Ipv4Address::new(255, 255, 255, 255), 68);
                    if let Err(e) = socket.send_to(encoded, dest).await {
                        defmt::warn!("DHCP send error: {}", defmt::Debug2Format(&e));
                    }
                }
                Err(e) => {
                    defmt::warn!("DHCP encode error: {}", defmt::Debug2Format(&e));
                }
            }
        }
    }
}

/// Simple HTTP server serving a test page.
#[embassy_executor::task]
async fn web_server(stack: Stack<'static>) {
    // Wait until the stack is configured
    loop {
        if stack.is_config_up() {
            break;
        }
        Timer::after(Duration::from_millis(100)).await;
    }
    info!("Web server listening on 192.168.4.1:80");

    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 2048];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        if let Err(_e) = socket.accept(80).await {
            defmt::warn!("Accept error");
            continue;
        }

        info!("Client connected");

        // Read request (we don't parse it, just drain it)
        let mut buf = [0u8; 512];
        match socket.read(&mut buf).await {
            Ok(0) | Err(_) => {
                continue;
            }
            Ok(_) => {}
        }

        // Send HTTP response
        let response_header = alloc::format!(
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
            TEST_PAGE.len()
        );

        let _ = socket.write_all(response_header.as_bytes()).await;
        let _ = socket.write_all(TEST_PAGE.as_bytes()).await;
        let _ = socket.flush().await;
        socket.close();
        Timer::after(Duration::from_millis(50)).await;
    }
}
