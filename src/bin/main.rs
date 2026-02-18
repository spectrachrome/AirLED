#![no_std]
#![no_main]

use core::net::Ipv4Addr;

use defmt::info;
use edge_dhcp::server::{Server as DhcpServer, ServerOptions as DhcpServerOptions};
use edge_dhcp::{Options as DhcpOptions, Packet as DhcpPacket};
use embassy_net::{Ipv4Address, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4, tcp::TcpSocket};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::{Read as AsyncRead, Write};
use esp_hal::clock::CpuClock;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use bleps::ad_structure::{
    create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
};
use bleps::async_attribute_server::AttributeServer;
use bleps::asynch::Ble;
use bleps::attribute_server::NotificationData;
use bleps::gatt;
use esp_radio::ble::controller::BleConnector;
use esp_radio::wifi::{
    AccessPointConfig, ModeConfig, WifiApState, ap_state,
};
use panic_rtt_target as _;
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_spi::prerendered::Ws2812;
use xiao_drone_led_controller::msp::{
    self, BoxId, MspParser, ParseResult,
};
use xiao_drone_led_controller::pattern::{
    Animation, ColorScheme, Pulse, RippleEffect, StaticAnim,
};
use xiao_drone_led_controller::postfx::{PostEffect, apply_pipeline};
use xiao_drone_led_controller::ble::{
    self as ble_proto, HandleResult,
};
use xiao_drone_led_controller::state::{
    AnimMode, AnimModeParams, BLE_FLASH, ColorMode, FlightMode, STATE, STATE_CHANGED,
};
use static_cell::StaticCell;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();
use alloc::string::ToString;

/// Maximum number of WS2812 LEDs supported (compile-time buffer size).
const MAX_LEDS: usize = 200;

/// SPI pre-rendered buffer size for ws2812-spi (4 SPI bytes per 2 data bits × 12 per LED).
const SPI_BUF_LEN: usize = MAX_LEDS * 12;

/// Wi-Fi AP SSID.
const WIFI_SSID: &str = "AirLED";

/// AP static IP address.
const AP_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);

/// Maximum number of active LEDs (must match `MAX_LEDS`).
const MAX_NUM_LEDS: u16 = MAX_LEDS as u16;

/// Enable MSP debug LED overlay (flag bits on first 33 LEDs when disarmed).
const MSP_DEBUG_LEDS: bool = false;

#[esp_hal::main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 96 * 1024);

    info!("Initializing...");

    // Start the RTOS scheduler (required before esp-radio init)
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // --- LED setup (SPI + DMA) ---
    let (rx_buf, rx_desc, tx_buf, tx_desc) = dma_buffers!(SPI_BUF_LEN);
    let dma_rx = DmaRxBuf::new(rx_desc, rx_buf).expect("failed to create DMA RX buf");
    let dma_tx = DmaTxBuf::new(tx_desc, tx_buf).expect("failed to create DMA TX buf");

    let spi_config = SpiConfig::default()
        .with_frequency(Rate::from_khz(3200))
        .with_mode(Mode::_0);

    let spi_bus = Spi::new(peripherals.SPI2, spi_config)
        .expect("failed to create SPI")
        .with_mosi(peripherals.GPIO10)
        .with_dma(peripherals.DMA_CH0)
        .with_buffers(dma_rx, dma_tx);

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

    // --- BLE setup ---
    info!("Setting up BLE...");
    let ble_connector = BleConnector::new(
        radio_controller,
        peripherals.BT,
        esp_radio::ble::Config::default(),
    )
    .expect("BLE init failed");
    info!("BLE ready");

    // --- MSP UART setup ---
    info!("Setting up MSP UART...");
    let msp_uart = Uart::new(peripherals.UART0, UartConfig::default())
        .expect("failed to create MSP UART")
        .with_rx(peripherals.GPIO20)
        .with_tx(peripherals.GPIO21)
        .into_async();
    info!("MSP UART ready");

    // Start embassy executor
    static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
    executor.run(move |spawner| {
        spawner.must_spawn(led_task(spi_bus));
        spawner.must_spawn(msp_task(msp_uart));
        spawner.must_spawn(ble_task(ble_connector));
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

/// BLE notification chunk size (BLE default MTU payload).
const BLE_CHUNK_SIZE: usize = 20;


/// Shared BLE RX reassembly buffer (written by write callback, read by notifier).
struct BleRxBuf {
    data: [u8; 256],
    len: usize,
}

/// Shared BLE TX response buffer (written by write callback/notifier, sent by notifier).
struct BleTxBuf {
    data: [u8; 512],
    len: usize,
    offset: usize,
}

static BLE_RX: critical_section::Mutex<core::cell::RefCell<BleRxBuf>> =
    critical_section::Mutex::new(core::cell::RefCell::new(BleRxBuf {
        data: [0; 256],
        len: 0,
    }));

static BLE_TX: critical_section::Mutex<core::cell::RefCell<BleTxBuf>> =
    critical_section::Mutex::new(core::cell::RefCell::new(BleTxBuf {
        data: [0; 512],
        len: 0,
        offset: 0,
    }));

/// Signal to wake the BLE notifier when there is data to send.
static BLE_NOTIFY: embassy_sync::signal::Signal<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    (),
> = embassy_sync::signal::Signal::new();

/// Process a complete command message from the RX buffer.
///
/// Called from the sync write callback. Writes the response into BLE_TX
/// and signals the notifier.
fn ble_handle_message(msg: &[u8]) {
    let Some(cmd) = ble_proto::parse_command(msg) else {
        // Write error response
        critical_section::with(|cs| {
            let mut tx = BLE_TX.borrow_ref_mut(cs);
            let err = b"err:parse\n";
            tx.data[..err.len()].copy_from_slice(err);
            tx.len = err.len();
            tx.offset = 0;
        });
        BLE_NOTIFY.signal(());
        return;
    };

    // Try to lock state synchronously (should almost always succeed)
    if let Ok(mut state) = STATE.try_lock() {
        let result = ble_proto::handle_command(&cmd, &mut state);
        critical_section::with(|cs| {
            let mut tx = BLE_TX.borrow_ref_mut(cs);
            tx.offset = 0;
            match result {
                HandleResult::SendState => {
                    let resp = ble_proto::build_state_response(&state);
                    tx.len = ble_proto::serialize_state(&resp, &mut tx.data).unwrap_or(0);
                }
                HandleResult::Ack => {
                    tx.data[..3].copy_from_slice(b"ok\n");
                    tx.len = 3;
                }
                HandleResult::Error(e) => {
                    let eb = e.as_bytes();
                    let len = eb.len().min(tx.data.len());
                    tx.data[..len].copy_from_slice(&eb[..len]);
                    tx.len = len;
                }
            }
        });
        BLE_NOTIFY.signal(());
    }
}

/// BLE Nordic UART Service task.
///
/// Advertises as "AirLED", accepts connections, and serves the NUS GATT service.
/// Commands arrive as JSON on RX; responses go out as notifications on TX.
#[embassy_executor::task]
async fn ble_task(mut connector: BleConnector<'static>) {
    info!("BLE task started");

    let current_millis = || embassy_time::Instant::now().as_millis();
    let mut ble = Ble::new(&mut connector, current_millis);

    loop {
        // Reset buffers between connections
        critical_section::with(|cs| {
            let mut rx = BLE_RX.borrow_ref_mut(cs);
            rx.len = 0;
            let mut tx = BLE_TX.borrow_ref_mut(cs);
            tx.len = 0;
            tx.offset = 0;
        });

        // Initialize BLE stack
        if let Err(e) = ble.init().await {
            defmt::warn!("BLE init error: {}", defmt::Debug2Format(&e));
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        // Log our BLE MAC address (once)
        match ble.cmd_read_br_addr().await {
            Ok(addr) => info!(
                "BLE MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]
            ),
            Err(e) => defmt::warn!("BLE read addr error: {}", defmt::Debug2Format(&e)),
        }

        // Set advertising data
        let adv_data = create_advertising_data(&[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(WIFI_SSID),
        ]);
        match adv_data {
            Ok(data) => {
                if let Err(e) = ble.cmd_set_le_advertising_data(data).await {
                    defmt::warn!("BLE adv data error: {}", defmt::Debug2Format(&e));
                    continue;
                }
            }
            Err(e) => {
                defmt::warn!("BLE adv build error: {}", defmt::Debug2Format(&e));
                continue;
            }
        }

        if let Err(e) = ble.cmd_set_le_advertising_parameters().await {
            defmt::warn!("BLE adv params error: {}", defmt::Debug2Format(&e));
            continue;
        }

        if let Err(e) = ble.cmd_set_le_advertise_enable(true).await {
            defmt::warn!("BLE adv enable error: {}", defmt::Debug2Format(&e));
            continue;
        }

        info!("BLE advertising as \"{}\"", WIFI_SSID);

        // Track whether we've seen a real client (first RX write = client connected)
        static BLE_CONNECTED: critical_section::Mutex<core::cell::Cell<bool>> =
            critical_section::Mutex::new(core::cell::Cell::new(false));
        critical_section::with(|cs| BLE_CONNECTED.borrow(cs).set(false));

        // Write callback for NUS RX characteristic (sync — runs inside do_work)
        let mut rx_wf = |_offset: usize, data: &[u8]| {
            // Flash blue on first write (= real client connection confirmed)
            critical_section::with(|cs| {
                if !BLE_CONNECTED.borrow(cs).get() {
                    BLE_CONNECTED.borrow(cs).set(true);
                    BLE_FLASH.signal(1);
                    defmt::info!("BLE client connected");
                }
            });

            critical_section::with(|cs| {
                let mut rx = BLE_RX.borrow_ref_mut(cs);
                let space = rx.data.len() - rx.len;
                let n = data.len().min(space);
                let start = rx.len;
                rx.data[start..start + n].copy_from_slice(&data[..n]);
                rx.len = start + n;
            });

            // Check for complete message (newline-delimited)
            let msg_result = critical_section::with(|cs| {
                let rx = BLE_RX.borrow_ref(cs);
                rx.data[..rx.len].iter().position(|&b| b == b'\n')
            });

            if let Some(nl_pos) = msg_result {
                // Extract message and shift remainder
                let mut msg = [0u8; 256];
                let msg_len = critical_section::with(|cs| {
                    let mut rx = BLE_RX.borrow_ref_mut(cs);
                    msg[..nl_pos].copy_from_slice(&rx.data[..nl_pos]);
                    let total = rx.len;
                    let remaining = total - nl_pos - 1;
                    rx.data.copy_within(nl_pos + 1..total, 0);
                    rx.len = remaining;
                    nl_pos
                });
                ble_handle_message(&msg[..msg_len]);
            }
        };

        // Read callback for NUS TX (unused — we use notifications)
        let mut tx_rf = |_offset: usize, data: &mut [u8]| {
            let msg = b"use notify";
            let len = msg.len().min(data.len());
            data[..len].copy_from_slice(&msg[..len]);
            len
        };

        gatt!([service {
            uuid: "6e400001-b5a3-f393-e0a9-e50e24dcca9e",
            characteristics: [
                characteristic {
                    name: "nus_rx",
                    uuid: "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
                    write: rx_wf,
                },
                characteristic {
                    name: "nus_tx",
                    uuid: "6e400003-b5a3-f393-e0a9-e50e24dcca9e",
                    notify: true,
                    read: tx_rf,
                },
            ],
        },]);

        let mut no_rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut no_rng);

        info!("BLE waiting for connection...");

        // Notifier: returns the next chunk to send as a notification.
        //
        // If there is unsent data in BLE_TX, returns the next chunk immediately.
        // Otherwise waits for BLE_NOTIFY (command response) or STATE_CHANGED (MSP push).
        let mut notifier = || async {
            loop {
                // Check for pending chunk data
                let chunk = critical_section::with(|cs| {
                    let mut tx = BLE_TX.borrow_ref_mut(cs);
                    if tx.offset < tx.len {
                        let end = (tx.offset + BLE_CHUNK_SIZE).min(tx.len);
                        let mut buf = [0u8; BLE_CHUNK_SIZE];
                        let chunk_len = end - tx.offset;
                        buf[..chunk_len].copy_from_slice(&tx.data[tx.offset..end]);
                        tx.offset = end;
                        Some((buf, chunk_len))
                    } else {
                        None
                    }
                });

                if let Some((buf, len)) = chunk {
                    return NotificationData::new(nus_tx_handle, &buf[..len]);
                }

                // No pending data — wait for new response or state change
                let notify_fut = BLE_NOTIFY.wait();
                let state_fut = STATE_CHANGED.wait();

                futures::pin_mut!(notify_fut);
                futures::pin_mut!(state_fut);

                match futures::future::select(notify_fut, state_fut).await {
                    futures::future::Either::Left(_) => {
                        // Command response queued — loop back to send chunks
                    }
                    futures::future::Either::Right(_) => {
                        // State changed — snapshot and queue
                        let state = STATE.lock().await;
                        let resp = ble_proto::build_state_response(&state);
                        drop(state);
                        critical_section::with(|cs| {
                            let mut tx = BLE_TX.borrow_ref_mut(cs);
                            tx.len = ble_proto::serialize_state(&resp, &mut tx.data)
                                .unwrap_or(0);
                            tx.offset = 0;
                        });
                        // Loop back to send chunks
                    }
                }
            }
        };

        match srv.run(&mut notifier).await {
            Ok(()) => {
                info!("BLE client disconnected");
                BLE_FLASH.signal(2);
            }
            Err(e) => {
                defmt::warn!("BLE server error: {}", defmt::Debug2Format(&e));
                BLE_FLASH.signal(2);
            }
        }
    }
}

/// Read bytes from UART until a complete MSP frame is parsed or timeout.
async fn read_msp_response(
    uart: &mut Uart<'static, esp_hal::Async>,
    parser: &mut MspParser,
    timeout: Duration,
) -> Option<msp::MspFrame> {
    parser.reset();
    let fut = async {
        let mut byte = [0u8; 1];
        loop {
            if AsyncRead::read(uart, &mut byte).await.is_err() {
                return None;
            }
            match parser.feed(byte[0]) {
                ParseResult::Incomplete => continue,
                ParseResult::Frame(f) => return Some(f),
                ParseResult::Error => return None,
            }
        }
    };
    with_timeout(timeout, fut).await.unwrap_or_default()
}

/// Polls the flight controller over MSP UART, updating shared state with
/// the current flight mode.
#[embassy_executor::task]
async fn msp_task(mut uart: Uart<'static, esp_hal::Async>) {
    info!("MSP task started");
    let mut parser = MspParser::new();
    let mut tx_buf = [0u8; 16];

    // --- Phase 1: query BOXNAMES to build the box map ---
    let mut box_map = [BoxId::Unknown; 48];
    let mut got_boxnames = false;

    // Try BOXIDS first (more reliable), then BOXNAMES as fallback.
    info!("MSP: querying box map...");
    let box_cmds = [msp::MSP_BOXIDS, msp::MSP_BOXNAMES];
    'startup: for attempt in 0..10 {
        for &cmd in &box_cmds {
            let len = msp::build_request(cmd, &[], &mut tx_buf);
            if Write::write_all(&mut uart, &tx_buf[..len]).await.is_err() {
                continue;
            }
            if let Some(frame) =
                read_msp_response(&mut uart, &mut parser, Duration::from_millis(500)).await
            {
                if frame.cmd == msp::MSP_BOXIDS {
                    box_map = msp::parse_boxids(&frame.payload, frame.size);
                    got_boxnames = true;
                    info!("MSP: BOXIDS received (attempt {})", attempt + 1);
                    break 'startup;
                } else if frame.cmd == msp::MSP_BOXNAMES {
                    box_map = msp::parse_boxnames(&frame.payload, frame.size);
                    got_boxnames = true;
                    info!("MSP: BOXNAMES received (attempt {})", attempt + 1);
                    break 'startup;
                }
            }
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    if got_boxnames {
        let mut state = STATE.lock().await;
        for (i, b) in box_map.iter().enumerate() {
            match b {
                BoxId::Arm => {
                    state.debug_arm_box = i as u8;
                    info!("MSP: box[{}] = ARM", i);
                }
                BoxId::Failsafe => {
                    state.debug_failsafe_box = i as u8;
                    info!("MSP: box[{}] = FAILSAFE", i);
                }
                _ => {}
            }
        }
    } else {
        info!("MSP: box map failed after 10 attempts, continuing with defaults");
    }
    info!("MSP: entering poll loop");

    // --- Phase 2: poll MSP_STATUS at ~10 Hz, MSP_RC at ~5 Hz ---
    let mut error_count: u8 = 0;
    let mut logged_raw_status = false;
    let mut rc_channels = [0u16; msp::MAX_RC_CHANNELS];
    let mut prev_aux = [0u16; 12]; // AUX1–AUX12 (channels 5–16)
    let mut rc_tick: u8 = 0;
    let mut logged_rc_once = false;

    loop {
        // Retry box map if we never got it (FC wasn't ready at startup)
        if !got_boxnames {
            for &cmd in &[msp::MSP_BOXIDS, msp::MSP_BOXNAMES] {
                let len = msp::build_request(cmd, &[], &mut tx_buf);
                if Write::write_all(&mut uart, &tx_buf[..len]).await.is_ok() {
                    if let Some(frame) =
                        read_msp_response(&mut uart, &mut parser, Duration::from_millis(500)).await
                    {
                        if frame.cmd == msp::MSP_BOXIDS {
                            box_map = msp::parse_boxids(&frame.payload, frame.size);
                            got_boxnames = true;
                        } else if frame.cmd == msp::MSP_BOXNAMES {
                            box_map = msp::parse_boxnames(&frame.payload, frame.size);
                            got_boxnames = true;
                        }
                        if got_boxnames {
                            let mut state = STATE.lock().await;
                            for (i, b) in box_map.iter().enumerate() {
                                match b {
                                    BoxId::Arm => {
                                        state.debug_arm_box = i as u8;
                                        info!("MSP: box[{}] = ARM", i);
                                    }
                                    BoxId::Failsafe => {
                                        state.debug_failsafe_box = i as u8;
                                        info!("MSP: box[{}] = FAILSAFE", i);
                                    }
                                    _ => {}
                                }
                            }
                            info!("MSP: box map received (late)");
                            break;
                        }
                    }
                }
            }
        }

        let len = msp::build_request(msp::MSP_STATUS, &[], &mut tx_buf);
        let send_ok = Write::write_all(&mut uart, &tx_buf[..len]).await.is_ok();

        let frame = if send_ok {
            read_msp_response(&mut uart, &mut parser, Duration::from_millis(100)).await
        } else {
            None
        };

        if let Some(frame) = frame {
            if frame.cmd == msp::MSP_STATUS {
                // Dump first 16 bytes of payload for debugging
                if !logged_raw_status {
                    info!("MSP STATUS size={} raw: {:02x} {:02x} {:02x} {:02x}  {:02x} {:02x} {:02x} {:02x}  {:02x} {:02x} {:02x} {:02x}  {:02x} {:02x} {:02x} {:02x}",
                        frame.size,
                        frame.payload[0], frame.payload[1], frame.payload[2], frame.payload[3],
                        frame.payload[4], frame.payload[5], frame.payload[6], frame.payload[7],
                        frame.payload[8], frame.payload[9], frame.payload[10], frame.payload[11],
                        frame.payload[12], frame.payload[13], frame.payload[14], frame.payload[15],
                    );
                    logged_raw_status = true;
                }
                if let Some(flags) = msp::extract_mode_flags(&frame.payload, frame.size) {
                    let arming_disable = msp::extract_arming_disable_flags(&frame.payload, frame.size)
                        .unwrap_or(0);
                    let mode = msp::resolve_flight_mode(flags, &box_map, arming_disable);
                    let mut state = STATE.lock().await;
                    if state.flight_mode != mode || !state.fc_connected {
                        info!("MSP: flags=0x{:08x} mode={}", flags, defmt::Debug2Format(&mode));
                    }
                    let changed = state.flight_mode != mode || !state.fc_connected;
                    state.fc_connected = true;
                    state.flight_mode = mode;
                    state.debug_flags = flags;
                    drop(state);
                    if changed {
                        STATE_CHANGED.signal(());
                    }
                    error_count = 0;
                } else {
                    error_count = error_count.saturating_add(1);
                }
            } else {
                error_count = error_count.saturating_add(1);
            }
        } else {
            error_count = error_count.saturating_add(1);
        }

        if error_count >= 10 {
            let mut state = STATE.lock().await;
            state.fc_connected = false;
            state.flight_mode = FlightMode::ArmingForbidden;
            drop(state);
            // Reset counter to avoid spamming state writes every tick
            error_count = 10;
        }

        // Poll RC channels every 5th tick (~2 Hz) with short timeout
        rc_tick = rc_tick.wrapping_add(1);
        if rc_tick.is_multiple_of(5) {
            let len = msp::build_request(msp::MSP_RC, &[], &mut tx_buf);
            if Write::write_all(&mut uart, &tx_buf[..len]).await.is_ok() {
                if let Some(frame) =
                    read_msp_response(&mut uart, &mut parser, Duration::from_millis(50)).await
                {
                    if frame.cmd == msp::MSP_RC {
                        let count =
                            msp::parse_rc_channels(&frame.payload, frame.size, &mut rc_channels);

                        // Dump all channels once so we can see which are active
                        if !logged_rc_once && count >= 4 {
                            logged_rc_once = true;
                            info!(
                                "MSP RC ({} ch): {} {} {} {}  {} {} {} {}  {} {} {} {}  {} {} {} {}",
                                count,
                                rc_channels[0], rc_channels[1], rc_channels[2], rc_channels[3],
                                rc_channels[4], rc_channels[5], rc_channels[6], rc_channels[7],
                                rc_channels[8], rc_channels[9], rc_channels[10], rc_channels[11],
                                rc_channels[12], rc_channels[13], rc_channels[14], rc_channels[15],
                            );
                        }

                        // AUX8 (channel 12, index 11) strobe trigger
                        // Check BEFORE updating prev_aux so we compare old vs new
                        if count >= 12 {
                            let strobe = rc_channels[11] > 1800;
                            let was_strobe = prev_aux[7] > 1800;
                            if strobe != was_strobe {
                                let mut state = STATE.lock().await;
                                state.aux_strobe = strobe;
                                info!("MSP AUX8 strobe: {}", strobe);
                            }
                        }

                        // Log AUX channel changes with deadband (channels 5–16)
                        let aux_count = count.saturating_sub(4).min(12);
                        for i in 0..aux_count {
                            let ch = rc_channels[i + 4];
                            let diff = ch.abs_diff(prev_aux[i]);
                            if diff > 50 {
                                info!("MSP AUX{}: {} -> {}", i + 1, prev_aux[i], ch);
                                prev_aux[i] = ch;
                            }
                        }
                    }
                }
            }
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}

/// Build a [`ColorScheme`] from the current [`ColorMode`].
fn build_color_scheme(mode: ColorMode, use_hsi: bool) -> ColorScheme {
    match mode {
        ColorMode::SolidGreen => ColorScheme::Solid(RGB8 { r: 0, g: 204, b: 0 }),
        ColorMode::SolidRed => ColorScheme::Solid(RGB8 { r: 204, g: 0, b: 0 }),
        ColorMode::Split => ColorScheme::Split(
            RGB8 { r: 0, g: 204, b: 0 },
            RGB8 { r: 204, g: 0, b: 0 },
        ),
        ColorMode::Rainbow => ColorScheme::Rainbow { hue: 0, speed: 1, use_hsi },
    }
}

/// Number of frames per armed show sub-pattern before advancing to the next.
const ARMED_SHOW_FRAMES: u32 = 1000; // ~10 s at 100 FPS

/// Number of armed show sub-patterns in the cycle.
const ARMED_SHOW_COUNT: u32 = 4;

/// Render the armed show pattern: cyclic rainbow variations.
///
/// `frame` is the global frame counter; the sub-pattern index is derived from it.
fn render_armed_show(leds: &mut [RGB8], frame: u32) {
    let sub = (frame / ARMED_SHOW_FRAMES) % ARMED_SHOW_COUNT;
    let num = leds.len();
    let tick = frame as u8; // wrapping is fine for hue rotation

    match sub {
        0 => {
            // Static rainbow gradient, slowly rotating
            for (i, led) in leds.iter_mut().enumerate() {
                let hue = tick.wrapping_add((i * 256 / num.max(1)) as u8);
                *led = smart_leds::hsv::hsv2rgb(smart_leds::hsv::Hsv {
                    hue,
                    sat: 255,
                    val: 255,
                });
            }
        }
        1 => {
            // Rainbow pulse: all LEDs same hue (rotating), pulsing brightness
            // Triangle wave: 0→1→0 over 200 frames
            let half = (frame % 200) as u16;
            let t = if half < 100 { half } else { 200 - half } as f32 / 100.0;
            let brightness = 0.4 + 0.6 * t;
            let val = (brightness * 255.0) as u8;
            for (i, led) in leds.iter_mut().enumerate() {
                let hue = tick.wrapping_add((i * 256 / num.max(1)) as u8);
                *led = smart_leds::hsv::hsv2rgb(smart_leds::hsv::Hsv {
                    hue,
                    sat: 255,
                    val,
                });
            }
        }
        2 => {
            // Rainbow chase: fast scroll
            let offset = (frame * 3) as u8;
            for (i, led) in leds.iter_mut().enumerate() {
                let hue = offset.wrapping_add((i * 256 / num.max(1)) as u8);
                *led = smart_leds::hsv::hsv2rgb(smart_leds::hsv::Hsv {
                    hue,
                    sat: 255,
                    val: 255,
                });
            }
        }
        _ => {
            // Rainbow with sparkle: normal rainbow + occasional bright white
            let xor_state = frame.wrapping_mul(2654435761); // simple hash
            for (i, led) in leds.iter_mut().enumerate() {
                let hue =
                    (tick.wrapping_mul(2)).wrapping_add((i * 256 / num.max(1)) as u8);
                let sparkle = (xor_state ^ (i as u32 * 7919)).is_multiple_of(40);
                if sparkle {
                    *led = RGB8 { r: 255, g: 255, b: 255 };
                } else {
                    *led = smart_leds::hsv::hsv2rgb(smart_leds::hsv::Hsv {
                        hue,
                        sat: 255,
                        val: 255,
                    });
                }
            }
        }
    }
}

/// Render the failsafe pattern: sliding red bars with black gaps.
fn render_failsafe(leds: &mut [RGB8], frame: u32) {
    let num = leds.len();
    if num == 0 {
        return;
    }
    // Bar and gap width: ~1/5 of strip length, minimum 2
    let bar_width = (num / 5).max(2);
    let period = bar_width * 2; // bar + gap
    // Offset advances by 2 LEDs per frame
    let offset = (frame as usize * 2) % period;

    for (i, led) in leds.iter_mut().enumerate() {
        let pos = (i + offset) % period;
        if pos < bar_width {
            *led = RGB8 { r: 255, g: 0, b: 0 };
        } else {
            *led = RGB8 { r: 0, g: 0, b: 0 };
        }
    }
}

/// Drives the WS2812 LED strip using the active animation + color scheme via SPI+DMA.
#[embassy_executor::task]
async fn led_task(spi_bus: SpiDmaBus<'static, esp_hal::Blocking>) {
    let mut ws_buf = [0u8; SPI_BUF_LEN];
    let mut ws = Ws2812::new(spi_bus, &mut ws_buf);

    let mut pulse = Pulse::new();
    let mut slow_pulse = Pulse::new();
    slow_pulse.set_params(400, 0.1);
    let mut ripple = RippleEffect::new(0xDEAD_BEEF);
    let mut static_anim = StaticAnim;

    let mut color_scheme = build_color_scheme(ColorMode::Split, false);
    let mut prev_color_mode = ColorMode::Split;
    let mut prev_use_hsi = false;

    let mut buf = [RGB8 { r: 0, g: 0, b: 0 }; MAX_LEDS];
    let mut write_err_logged = false;
    let mut frame_counter: u32 = 0;

    // BLE flash state: remaining frames in the flash sequence, number of flashes
    let mut flash_remaining: u32 = 0;
    let mut flash_count: u8 = 0;
    let mut flash_total_frames: u32 = 0;

    loop {
        // Check for new BLE flash request
        if let Some(count) = BLE_FLASH.try_take() {
            flash_count = count;
            // Will be computed after we know FPS below
            flash_remaining = u32::MAX; // sentinel — set properly after FPS read
        }

        let state = STATE.lock().await;
        let num_leds = state.num_leds.min(MAX_NUM_LEDS) as usize;
        let led_brightness = state.brightness;
        let max_ma = state.max_current_ma;
        let fps = state.fps.max(1);

        // Initialize flash frame count now that we know FPS
        if flash_remaining == u32::MAX {
            flash_total_frames = (fps as u32 * 750) / 1000; // 750ms worth of frames
            flash_remaining = flash_total_frames;
        }
        let fc_connected = state.fc_connected;
        let flight_mode = state.flight_mode;
        let debug_flags = state.debug_flags;
        let debug_arm_box = state.debug_arm_box;
        let debug_failsafe_box = state.debug_failsafe_box;
        let color_mode = state.color_mode;
        let color_params = state.color_params;
        let use_hsi = state.use_hsi;
        let bal_r = state.color_bal_r;
        let bal_g = state.color_bal_g;
        let bal_b = state.color_bal_b;
        let anim_mode = state.anim_mode;
        let anim_params = state.anim_params;
        let aux_strobe = state.aux_strobe;
        drop(state);

        let active = &mut buf[..num_leds];

        // AUX8 strobe override: fast white strobe (~12.5 Hz)
        if aux_strobe {
            // Toggle every 3 frames at 100 FPS ≈ 16.7 Hz strobe
            let on = (frame_counter / 3).is_multiple_of(2);
            let color = if on {
                RGB8 { r: 255, g: 255, b: 255 }
            } else {
                RGB8 { r: 0, g: 0, b: 0 }
            };
            for led in active.iter_mut() {
                *led = color;
            }

            // Apply brightness + current limit but skip gamma/color balance
            let pipeline = [
                PostEffect::Brightness(led_brightness),
                PostEffect::CurrentLimit { max_ma },
            ];
            apply_pipeline(active, &pipeline);

            match ws.write(active.iter().copied()) {
                Err(e) if !write_err_logged => {
                    defmt::warn!("LED write error: {}", defmt::Debug2Format(&e));
                    write_err_logged = true;
                }
                Ok(_) if write_err_logged => {
                    info!("LED write recovered");
                    write_err_logged = false;
                }
                _ => {}
            }
            frame_counter = frame_counter.wrapping_add(1);
            Timer::after(Duration::from_millis(1000 / fps as u64)).await;
            continue;
        }

        // BLE flash override: solid blue flashes (1× connect, 2× disconnect)
        if flash_remaining > 0 {
            let blue = RGB8 { r: 0, g: 0, b: 255 };
            let black = RGB8 { r: 0, g: 0, b: 0 };

            let on = if flash_count == 1 {
                // Single flash: solid blue for the entire 750ms
                true
            } else {
                // Two flashes: on/off/on split across the total frames
                // Pattern: [on 40%] [off 20%] [on 40%]
                let pos = flash_total_frames - flash_remaining;
                let first_end = flash_total_frames * 2 / 5;
                let gap_end = flash_total_frames * 3 / 5;
                pos < first_end || pos >= gap_end
            };

            let color = if on { blue } else { black };
            for led in active.iter_mut() {
                *led = color;
            }

            flash_remaining -= 1;

            // Skip normal rendering and post-processing — write directly
            match ws.write(active.iter().copied()) {
                Err(e) if !write_err_logged => {
                    defmt::warn!("LED write error: {}", defmt::Debug2Format(&e));
                    write_err_logged = true;
                }
                Ok(_) if write_err_logged => {
                    info!("LED write recovered");
                    write_err_logged = false;
                }
                _ => {}
            }
            frame_counter = frame_counter.wrapping_add(1);
            Timer::after(Duration::from_millis(1000 / fps as u64)).await;
            continue;
        }

        // Flight-mode override logic:
        // - Armed → rainbow show cycle
        // - Failsafe → sliding red bars
        // - Arming forbidden (FC connected) → slow red pulse
        // - Arming allowed (FC connected) → solid green pulse
        // - No FC → user-selected pattern from web UI
        if fc_connected && flight_mode == FlightMode::Armed {
            render_armed_show(active, frame_counter);
        } else if fc_connected && flight_mode == FlightMode::Failsafe {
            render_failsafe(active, frame_counter);
        } else if fc_connected && flight_mode == FlightMode::ArmingForbidden {
            // FC connected, arming forbidden: slow red pulse
            let mut red_scheme = ColorScheme::Solid(RGB8 { r: 204, g: 0, b: 0 });
            slow_pulse.render(active, &mut red_scheme);
        } else if fc_connected && flight_mode == FlightMode::ArmingAllowed {
            // FC connected, disarmed: green pulse with debug overlay
            let mut green_scheme = ColorScheme::Solid(RGB8 { r: 0, g: 204, b: 0 });
            pulse.render(active, &mut green_scheme);

            // Debug: first 33 LEDs show flag bits (compile-time flag)
            if MSP_DEBUG_LEDS {
                let overlay_len = 33.min(active.len());
                if overlay_len > 0 {
                    active[0] = if debug_arm_box != 255 {
                        RGB8 { r: 255, g: 255, b: 255 }
                    } else {
                        RGB8 { r: 255, g: 0, b: 255 }
                    };
                }
                for (i, led) in active.iter_mut().enumerate().take(overlay_len).skip(1) {
                    let bit = i - 1;
                    let bit_set = debug_flags & (1 << bit) != 0;
                    *led = if bit as u8 == debug_arm_box {
                        if bit_set { RGB8 { r: 0, g: 255, b: 0 } } else { RGB8 { r: 0, g: 40, b: 0 } }
                    } else if bit as u8 == debug_failsafe_box {
                        if bit_set { RGB8 { r: 255, g: 0, b: 0 } } else { RGB8 { r: 40, g: 0, b: 0 } }
                    } else if bit_set {
                        RGB8 { r: 0, g: 0, b: 128 }
                    } else {
                        RGB8 { r: 0, g: 0, b: 0 }
                    }
                }
            }
        } else {
            // Normal user-selected pattern
            if color_mode != prev_color_mode || use_hsi != prev_use_hsi {
                color_scheme = build_color_scheme(color_mode, use_hsi);
                prev_color_mode = color_mode;
                prev_use_hsi = use_hsi;
            }
            color_scheme.set_hue_speed(color_params.hue_speed);

            match anim_params {
                AnimModeParams::Pulse { speed, min_intensity_pct } => {
                    pulse.set_params(speed, min_intensity_pct as f32 / 100.0);
                }
                AnimModeParams::Ripple { speed_x10, width_x10, decay_pct } => {
                    ripple.set_params(
                        speed_x10 as f32 / 10.0,
                        width_x10 as f32 / 10.0,
                        decay_pct as f32 / 100.0,
                    );
                }
                AnimModeParams::Static => {}
            }

            match anim_mode {
                AnimMode::Static => static_anim.render(active, &mut color_scheme),
                AnimMode::Pulse => pulse.render(active, &mut color_scheme),
                AnimMode::Ripple => ripple.render(active, &mut color_scheme),
            }
        }

        let pipeline = [
            PostEffect::Gamma,
            PostEffect::ColorBalance { r: bal_r, g: bal_g, b: bal_b },
            PostEffect::Brightness(led_brightness),
            PostEffect::CurrentLimit { max_ma },
        ];
        apply_pipeline(active, &pipeline);

        match ws.write(active.iter().copied()) {
            Err(e) if !write_err_logged => {
                defmt::warn!("LED write error: {}", defmt::Debug2Format(&e));
                write_err_logged = true;
            }
            Ok(_) if write_err_logged => {
                info!("LED write recovered");
                write_err_logged = false;
            }
            _ => {}
        }

        frame_counter = frame_counter.wrapping_add(1);
        Timer::after(Duration::from_millis(1000 / fps as u64)).await;
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

/// Parse query parameters from a request path, updating state values.
///
/// Expects the query portion after `?`, e.g. `brightness=128&color=split&anim=pulse`.
/// Unknown keys are silently ignored.
fn parse_query_params(query: &str, state: &mut xiao_drone_led_controller::state::LedState) {
    // Check for color/anim mode changes first — if present, reset params to defaults
    // before applying per-mode overrides in the same request.
    for pair in query.split('&') {
        if let Some((key, value)) = pair.split_once('=') {
            match key {
                "color" => {
                    let new_mode = match value {
                        "solid_green" => Some(ColorMode::SolidGreen),
                        "solid_red" => Some(ColorMode::SolidRed),
                        "split" => Some(ColorMode::Split),
                        "rainbow" => Some(ColorMode::Rainbow),
                        _ => None,
                    };
                    if let Some(m) = new_mode {
                        state.color_mode = m;
                    }
                }
                "anim" => {
                    let new_mode = match value {
                        "static" => Some(AnimMode::Static),
                        "pulse" => Some(AnimMode::Pulse),
                        "ripple" => Some(AnimMode::Ripple),
                        _ => None,
                    };
                    if let Some(m) = new_mode {
                        if m != state.anim_mode {
                            state.anim_mode = m;
                            state.anim_params = AnimModeParams::default_for(m);
                        }
                    }
                }
                _ => {}
            }
        }
    }

    for pair in query.split('&') {
        if let Some((key, value)) = pair.split_once('=') {
            match key {
                "brightness" => {
                    if let Ok(v) = value.parse::<u16>() {
                        state.brightness = v.min(255) as u8;
                    }
                }
                "num_leds" => {
                    if let Ok(v) = value.parse::<u16>() {
                        state.num_leds = v.clamp(1, MAX_NUM_LEDS);
                    }
                }
                "fps" => {
                    if let Ok(v) = value.parse::<u8>() {
                        state.fps = v.clamp(1, 150);
                    }
                }
                "max_current_ma" => {
                    if let Ok(v) = value.parse::<u32>() {
                        state.max_current_ma = v.clamp(100, 2500);
                    }
                }
                "color" | "anim" => { /* already handled above */ }
                "bal_r" => {
                    if let Ok(v) = value.parse::<u16>() {
                        state.color_bal_r = v.min(255) as u8;
                    }
                }
                "bal_g" => {
                    if let Ok(v) = value.parse::<u16>() {
                        state.color_bal_g = v.min(255) as u8;
                    }
                }
                "bal_b" => {
                    if let Ok(v) = value.parse::<u16>() {
                        state.color_bal_b = v.min(255) as u8;
                    }
                }
                "use_hsi" => {
                    state.use_hsi = value == "1";
                }
                "hue_speed" => {
                    if let Ok(v) = value.parse::<u8>() {
                        state.color_params.hue_speed = v.clamp(1, 10);
                    }
                }
                "pulse_speed" => {
                    if let Ok(v) = value.parse::<u16>() {
                        if let AnimModeParams::Pulse { speed, .. } = &mut state.anim_params {
                            *speed = v.clamp(100, 2000);
                        }
                    }
                }
                "min_brightness" => {
                    if let Ok(v) = value.parse::<u8>() {
                        if let AnimModeParams::Pulse { min_intensity_pct, .. } = &mut state.anim_params {
                            *min_intensity_pct = v.min(80);
                        }
                    }
                }
                "ripple_speed" => {
                    if let Ok(v) = value.parse::<u8>() {
                        if let AnimModeParams::Ripple { speed_x10, .. } = &mut state.anim_params {
                            *speed_x10 = v.clamp(5, 50);
                        }
                    }
                }
                "ripple_width" => {
                    if let Ok(v) = value.parse::<u8>() {
                        if let AnimModeParams::Ripple { width_x10, .. } = &mut state.anim_params {
                            *width_x10 = v.clamp(10, 255);
                        }
                    }
                }
                "ripple_decay" => {
                    if let Ok(v) = value.parse::<u8>() {
                        if let AnimModeParams::Ripple { decay_pct, .. } = &mut state.anim_params {
                            *decay_pct = v.clamp(90, 99);
                        }
                    }
                }
                _ => {}
            }
        }
    }
}

/// Map a [`ColorMode`] to its query-string key.
fn color_key(mode: ColorMode) -> &'static str {
    match mode {
        ColorMode::SolidGreen => "solid_green",
        ColorMode::SolidRed => "solid_red",
        ColorMode::Split => "split",
        ColorMode::Rainbow => "rainbow",
    }
}

/// Map an [`AnimMode`] to its query-string key.
fn anim_key(mode: AnimMode) -> &'static str {
    match mode {
        AnimMode::Static => "static",
        AnimMode::Pulse => "pulse",
        AnimMode::Ripple => "ripple",
    }
}

/// Build the HTML control page with current state values injected.
fn build_html_page(state: &xiao_drone_led_controller::state::LedState) -> alloc::string::String {
    let brightness = state.brightness;
    let num_leds = state.num_leds;
    let fps = state.fps;
    let max_current_ma = state.max_current_ma;
    let color_mode = state.color_mode;
    let anim_mode = state.anim_mode;
    let anim_params = state.anim_params;
    let hue_speed = state.color_params.hue_speed;
    let bal_r = state.color_bal_r;
    let bal_g = state.color_bal_g;
    let bal_b = state.color_bal_b;
    let use_hsi = state.use_hsi;
    let hsi_checked = if use_hsi { " checked" } else { "" };

    // Extract param values (use defaults for non-matching variants).
    let (pulse_speed, min_brightness) = match anim_params {
        AnimModeParams::Pulse { speed, min_intensity_pct } => (speed, min_intensity_pct),
        _ => (600, 40),
    };
    let (ripple_speed, ripple_width, ripple_decay) = match anim_params {
        AnimModeParams::Ripple { speed_x10, width_x10, decay_pct } => (speed_x10, width_x10, decay_pct),
        _ => (15, 190, 97),
    };

    let csel = |key| if color_key(color_mode) == key { " selected" } else { "" };
    let asel = |key| if anim_key(anim_mode) == key { " selected" } else { "" };

    let sel_solid_green = csel("solid_green");
    let sel_solid_red = csel("solid_red");
    let sel_split = csel("split");
    let sel_rainbow = csel("rainbow");
    let sel_static = asel("static");
    let sel_pulse = asel("pulse");
    let sel_ripple = asel("ripple");

    alloc::format!(
        r#"<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AirLED</title>
<style>
*{{box-sizing:border-box;margin:0}}
body{{font:18px/1.5 -apple-system,system-ui,sans-serif;background:#0f172a;color:#e2e8f0;padding:16px}}
.c{{max-width:400px;margin:0 auto}}
h1{{font-size:1.3em;color:#f8fafc;margin-bottom:12px}}
.g{{background:#1e293b;border-radius:10px;padding:14px;margin-bottom:10px}}
label{{display:flex;justify-content:space-between;font-size:.85em;color:#94a3b8;margin-bottom:4px}}
.v{{color:#38bdf8}}
select,input[type=range]{{width:100%}}
select{{background:#0f172a;color:#e2e8f0;border:1px solid #334155;border-radius:6px;padding:8px;font-size:.9em}}
input[type=range]{{-webkit-appearance:none;height:6px;border-radius:3px;background:#334155;outline:none}}
input[type=range]::-webkit-slider-thumb{{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;background:#38bdf8;cursor:pointer}}
#sb{{position:fixed;bottom:0;left:0;right:0;text-align:center;font-size:.85em;font-weight:600;padding:8px;transform:translateY(100%);transition:transform .3s}}
#sb.show{{transform:translateY(0)}}
#sb.ok{{background:#166534;color:#bbf7d0}}
#sb.err{{background:#991b1b;color:#fecaca}}
</style>
</head>
<body>
<div class="c">
<h1>AirLED</h1>
<div class="g"><label>Color</label>
<select id="cm">
<option value="solid_green"{sel_solid_green}>Solid Green</option>
<option value="solid_red"{sel_solid_red}>Solid Red</option>
<option value="split"{sel_split}>Split (Green/Red)</option>
<option value="rainbow"{sel_rainbow}>Rainbow</option>
</select></div>
<div class="g"><label>Animation</label>
<select id="am">
<option value="static"{sel_static}>Static</option>
<option value="pulse"{sel_pulse}>Pulse</option>
<option value="ripple"{sel_ripple}>Ripple</option>
</select></div>
<div class="g"><label>Brightness <span class="v" id="bv">{brightness}</span></label>
<input type="range" id="br" min="0" max="255" value="{brightness}"></div>
<div class="g"><label>LEDs <span class="v" id="lv">{num_leds}</span></label>
<input type="range" id="lc" min="1" max="{max_leds}" value="{num_leds}"></div>
<div class="g"><label>FPS <span class="v" id="fv">{fps}</span></label>
<input type="range" id="fp" min="1" max="150" value="{fps}"></div>
<div class="g"><label>Current Limit <span class="v" id="cv">{max_current_ma}</span> mA</label>
<input type="range" id="cl" min="100" max="2500" step="50" value="{max_current_ma}"></div>
<div class="g"><label>Balance R <span class="v" id="brv">{bal_r}</span></label>
<input type="range" id="blr" min="0" max="255" value="{bal_r}"></div>
<div class="g"><label>Balance G <span class="v" id="bgv">{bal_g}</span></label>
<input type="range" id="blg" min="0" max="255" value="{bal_g}"></div>
<div class="g"><label>Balance B <span class="v" id="bbv">{bal_b}</span></label>
<input type="range" id="blb" min="0" max="255" value="{bal_b}"></div>
<div class="g pm" data-color="rainbow"><label>Hue Speed <span class="v" id="hsv">{hue_speed}</span></label>
<input type="range" id="hs" min="1" max="10" value="{hue_speed}"></div>
<div class="g pm" data-color="rainbow"><label><input type="checkbox" id="hi"{hsi_checked}> Use HSI color space</label></div>
<div class="g pm" data-anim="pulse"><label>Pulse Speed <span class="v" id="psv">{pulse_speed}</span></label>
<input type="range" id="ps" min="100" max="2000" value="{pulse_speed}"></div>
<div class="g pm" data-anim="pulse"><label>Min Brightness <span class="v" id="mbv">{min_brightness}</span>%</label>
<input type="range" id="mb" min="0" max="80" value="{min_brightness}"></div>
<div class="g pm" data-anim="ripple"><label>Ripple Speed <span class="v" id="rsv">{ripple_speed}</span></label>
<input type="range" id="rs" min="5" max="50" value="{ripple_speed}"></div>
<div class="g pm" data-anim="ripple"><label>Ripple Width <span class="v" id="rwv">{ripple_width}</span></label>
<input type="range" id="rw" min="10" max="255" value="{ripple_width}"></div>
<div class="g pm" data-anim="ripple"><label>Ripple Decay <span class="v" id="rdv">{ripple_decay}</span>%</label>
<input type="range" id="rd" min="90" max="99" value="{ripple_decay}"></div>
</div>
<div id="sb"></div>
<script>
var br=document.getElementById('br'),lc=document.getElementById('lc'),fp=document.getElementById('fp'),cl=document.getElementById('cl');
var blr=document.getElementById('blr'),blg=document.getElementById('blg'),blb=document.getElementById('blb');
var cm=document.getElementById('cm'),am=document.getElementById('am');
var bv=document.getElementById('bv'),lv=document.getElementById('lv'),fv=document.getElementById('fv'),cv=document.getElementById('cv');
var brv=document.getElementById('brv'),bgv=document.getElementById('bgv'),bbv=document.getElementById('bbv');
var ps=document.getElementById('ps'),mb=document.getElementById('mb');
var rs=document.getElementById('rs'),rw=document.getElementById('rw'),rd=document.getElementById('rd');
var hs=document.getElementById('hs'),hi=document.getElementById('hi');
var psv=document.getElementById('psv'),mbv=document.getElementById('mbv');
var rsv=document.getElementById('rsv'),rwv=document.getElementById('rwv'),rdv=document.getElementById('rdv');
var hsv=document.getElementById('hsv');
var t;
function updateVis(){{var c=cm.value,a=am.value;document.querySelectorAll('.pm').forEach(function(el){{var sc=el.dataset.color,sa=el.dataset.anim;var show=true;if(sc)show=show&&sc===c;if(sa)show=show&&sa===a;el.style.display=show?'':'none'}})}}
var sb=document.getElementById('sb'),con=null,ht;
function toast(ok){{if(ok===con)return;con=ok;sb.textContent=ok?'Connected':'Disconnected';sb.className=ok?'show ok':'show err';clearTimeout(ht);if(ok)ht=setTimeout(function(){{sb.className=''}},2000)}}
function send(){{var q='brightness='+br.value+'&num_leds='+lc.value+'&fps='+fp.value+'&max_current_ma='+cl.value+'&bal_r='+blr.value+'&bal_g='+blg.value+'&bal_b='+blb.value+'&color='+cm.value+'&anim='+am.value;var c=cm.value,a=am.value;if(c==='rainbow')q+='&hue_speed='+hs.value+'&use_hsi='+(hi.checked?'1':'0');if(a==='pulse')q+='&pulse_speed='+ps.value+'&min_brightness='+mb.value;if(a==='ripple')q+='&ripple_speed='+rs.value+'&ripple_width='+rw.value+'&ripple_decay='+rd.value;fetch('/set?'+q).then(function(){{toast(true)}}).catch(function(){{toast(false)}})}}
setInterval(function(){{fetch('/set').then(function(){{toast(true)}}).catch(function(){{toast(false)}})}},3000);
function sl(el,vl){{el.oninput=function(){{vl.textContent=el.value;clearTimeout(t);t=setTimeout(send,80)}}}}
sl(br,bv);sl(lc,lv);sl(fp,fv);sl(cl,cv);sl(blr,brv);sl(blg,bgv);sl(blb,bbv);sl(ps,psv);sl(mb,mbv);sl(rs,rsv);sl(rw,rwv);sl(rd,rdv);sl(hs,hsv);
hi.onchange=function(){{send()}};
var _bro=br.oninput;br.oninput=function(){{_bro.call(this);ubr()}};
function ubr(){{var v=br.value/255;br.style.background='rgb('+Math.round(51+5*v)+','+Math.round(65+124*v)+','+Math.round(85+163*v)+')'}}
ubr();
cm.onchange=function(){{updateVis();send()}};
am.onchange=function(){{updateVis();send()}};
updateVis();
</script>
</body>
</html>"#,
        brightness = brightness,
        num_leds = num_leds,
        max_leds = MAX_NUM_LEDS,
        fps = fps,
        max_current_ma = max_current_ma,
        sel_solid_green = sel_solid_green,
        sel_solid_red = sel_solid_red,
        sel_split = sel_split,
        sel_rainbow = sel_rainbow,
        sel_static = sel_static,
        sel_pulse = sel_pulse,
        sel_ripple = sel_ripple,
        pulse_speed = pulse_speed,
        min_brightness = min_brightness,
        ripple_speed = ripple_speed,
        ripple_width = ripple_width,
        ripple_decay = ripple_decay,
        bal_r = bal_r,
        bal_g = bal_g,
        bal_b = bal_b,
        hue_speed = hue_speed,
        hsi_checked = hsi_checked,
    )
}

/// HTTP server with interactive LED control page.
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
    let mut tx_buffer = [0u8; 4096];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        if let Err(_e) = socket.accept(80).await {
            defmt::warn!("Accept error");
            continue;
        }

        // Read HTTP request
        let mut buf = [0u8; 512];
        let n = match socket.read(&mut buf).await {
            Ok(0) | Err(_) => {
                continue;
            }
            Ok(n) => n,
        };

        // Extract the request path from the first line (e.g. "GET /set?brightness=128 HTTP/1.1")
        let request = core::str::from_utf8(&buf[..n]).unwrap_or("");
        let path = request
            .split_once(' ')       // skip method
            .and_then(|(_, rest)| rest.split_once(' ')) // isolate path from HTTP version
            .map(|(path, _)| path)
            .unwrap_or("/");

        if path.starts_with("/set") {
            // Parse query params and update state
            if let Some((_, query)) = path.split_once('?') {
                let mut state = STATE.lock().await;
                parse_query_params(query, &mut state);
            }

            let response = b"HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n";
            let _ = socket.write_all(response).await;
        } else {
            // Serve the control page with current values
            let state = STATE.lock().await;
            let page = build_html_page(&state);
            drop(state);

            let header = alloc::format!(
                "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                page.len()
            );

            let _ = socket.write_all(header.as_bytes()).await;
            let _ = socket.write_all(page.as_bytes()).await;
        }

        let _ = socket.flush().await;
        socket.close();
        Timer::after(Duration::from_millis(50)).await;
    }
}
