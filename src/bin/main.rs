#![no_std]
#![no_main]

use defmt::info;
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
use xiao_drone_led_controller::ble::{self as ble_proto, HandleResult};
use xiao_drone_led_controller::state::{
    AnimMode, AnimModeParams, BLE_FLASH, ColorMode, FlightMode, STATE, STATE_CHANGED,
    TEST_PATTERN,
};
use static_cell::StaticCell;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

/// Maximum number of WS2812 LEDs supported (compile-time buffer size).
const MAX_LEDS: usize = 200;

/// SPI pre-rendered buffer size for ws2812-spi (4 SPI bytes per 2 data bits × 12 per LED).
const SPI_BUF_LEN: usize = MAX_LEDS * 12;

/// BLE device advertising name.
const BLE_DEVICE_NAME: &str = "AirLED";

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

    // --- Radio + BLE setup (scheduler is now running) ---
    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio_controller: &'static esp_radio::Controller<'static> =
        RADIO.init(esp_radio::init().expect("failed to init esp-radio"));

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
    })
}

/// BLE notification chunk size (BLE default MTU payload).
const BLE_CHUNK_SIZE: usize = 20;


/// Shared BLE TX response buffer (written by write callback/notifier, sent by notifier).
struct BleTxBuf {
    data: [u8; 32],
    len: usize,
    offset: usize,
}

static BLE_TX: critical_section::Mutex<core::cell::RefCell<BleTxBuf>> =
    critical_section::Mutex::new(core::cell::RefCell::new(BleTxBuf {
        data: [0; 32],
        len: 0,
        offset: 0,
    }));

/// Signal to wake the BLE notifier when there is data to send.
static BLE_NOTIFY: embassy_sync::signal::Signal<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    (),
> = embassy_sync::signal::Signal::new();

/// Process a binary command from the RX write callback.
///
/// Writes the response into BLE_TX and signals the notifier.
fn ble_handle_message(data: &[u8]) {
    // Try to lock state synchronously (should almost always succeed)
    if let Ok(mut state) = STATE.try_lock() {
        let result = ble_proto::handle_binary_command(data, &mut state);
        critical_section::with(|cs| {
            let mut tx = BLE_TX.borrow_ref_mut(cs);
            tx.offset = 0;
            match result {
                HandleResult::SendState => {
                    tx.len = ble_proto::encode_state(&state, &mut tx.data);
                }
                HandleResult::SendVersion => {
                    tx.len = ble_proto::encode_version(&mut tx.data);
                }
                HandleResult::Ack(code) => {
                    tx.data[0] = code;
                    tx.len = 1;
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
        // Reset TX buffer between connections
        critical_section::with(|cs| {
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
            AdStructure::CompleteLocalName(BLE_DEVICE_NAME),
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

        info!("BLE advertising as \"{}\"", BLE_DEVICE_NAME);

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

            // Binary protocol: each BLE write is a complete command (no framing)
            ble_handle_message(data);
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
                        // State changed — encode and queue binary snapshot
                        let state = STATE.lock().await;
                        critical_section::with(|cs| {
                            let mut tx = BLE_TX.borrow_ref_mut(cs);
                            tx.len = ble_proto::encode_state(&state, &mut tx.data);
                            tx.offset = 0;
                        });
                        drop(state);
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

    // --- Phase 2: poll MSP_STATUS + MSP_RC + MSP_ANALOG every tick (~20 Hz) ---
    let mut error_count: u8 = 0;
    let mut logged_raw_status = false;
    let mut rc_channels = [0u16; msp::MAX_RC_CHANNELS];
    let mut prev_aux = [0u16; 12]; // AUX1–AUX12 (channels 5–16)
    let mut rc_tick: u8 = 0;
    let mut logged_rc_once = false;
    let mut tx_linked = false;

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
            read_msp_response(&mut uart, &mut parser, Duration::from_millis(30)).await
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
            state.aux_strobe = 0;
            state.tx_linked = false;
            tx_linked = false;
            drop(state);
            // Reset counter to avoid spamming state writes every tick
            error_count = 10;
        }

        // Poll RC channels every tick with short timeout
        rc_tick = rc_tick.wrapping_add(1);
        {
            let len = msp::build_request(msp::MSP_RC, &[], &mut tx_buf);
            if Write::write_all(&mut uart, &tx_buf[..len]).await.is_ok() {
                if let Some(frame) =
                    read_msp_response(&mut uart, &mut parser, Duration::from_millis(20)).await
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

                        // TX link detection: when no TX is bound all 4 stick
                        // channels (AETR) sit at exactly 1500 µs.
                        if count >= 4 {
                            let linked = !(rc_channels[0] == 1500
                                && rc_channels[1] == 1500
                                && rc_channels[2] == 1500
                                && rc_channels[3] == 1500);
                            if linked != tx_linked {
                                info!("MSP: TX link {}", if linked { "up" } else { "down" });
                                tx_linked = linked;
                                let mut state = STATE.lock().await;
                                state.tx_linked = linked;
                                if !linked {
                                    state.aux_strobe = 0;
                                }
                                drop(state);
                                STATE_CHANGED.signal(());
                            }
                        }

                        // AUX7 (channel 11, index 10) 3-position strobe
                        // AUX8 (channel 12, index 11) spring switch override → full
                        // Suppress strobe for first 10s after boot and when TX is not linked
                        let uptime_ms = embassy_time::Instant::now().as_millis();
                        if count >= 12 && uptime_ms > 10_000 && tx_linked {
                            let aux7 = rc_channels[10];
                            let aux8 = rc_channels[11];
                            let strobe_level: u8 = if aux8 > 1800 {
                                255 // AUX8 spring switch → full blast
                            } else if aux7 > 1650 {
                                255 // AUX7 position 3 → full
                            } else if aux7 > 1250 {
                                80  // AUX7 position 2 → low
                            } else {
                                0   // off
                            };
                            let mut state = STATE.lock().await;
                            if state.aux_strobe != strobe_level {
                                info!("MSP strobe: {}", strobe_level);
                            }
                            state.aux_strobe = strobe_level;
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

        Timer::after(Duration::from_millis(10)).await;
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
    let mut fc_pulse = Pulse::new();
    fc_pulse.set_params(400, 0.5);
    let mut ripple = RippleEffect::new(0xDEAD_BEEF);
    let mut static_anim = StaticAnim;

    let mut color_scheme = build_color_scheme(ColorMode::Split, false);
    let mut armed_scheme = ColorScheme::Rainbow { hue: 0, speed: 2, use_hsi: false };
    let mut armed_ripple = RippleEffect::new(0xCAFE_BABE);
    let mut prev_color_mode = ColorMode::Split;
    let mut prev_use_hsi = false;

    let mut buf = [RGB8 { r: 0, g: 0, b: 0 }; MAX_LEDS];
    let mut write_err_logged = false;
    let mut frame_counter: u32 = 0;

    // BLE flash state: remaining frames in the flash sequence, number of flashes
    let mut flash_remaining: u32 = 0;
    let mut flash_count: u8 = 0;
    let mut flash_total_frames: u32 = 0;

    // Test pattern state: wall-clock deadline, solid color to display
    let mut test_deadline: Option<embassy_time::Instant> = None;
    let mut test_color = RGB8 { r: 0, g: 0, b: 0 };

    loop {
        // Check for new BLE flash request
        if let Some(count) = BLE_FLASH.try_take() {
            flash_count = count;
            // Will be computed after we know FPS below
            flash_remaining = u32::MAX; // sentinel — set properly after FPS read
        }

        // Check for new test pattern request
        if let Some(color_code) = TEST_PATTERN.try_take() {
            test_color = match color_code {
                0 => RGB8 { r: 255, g: 0, b: 0 },
                1 => RGB8 { r: 0, g: 255, b: 0 },
                2 => RGB8 { r: 0, g: 0, b: 255 },
                _ => RGB8 { r: 255, g: 255, b: 255 },
            };
            test_deadline = Some(embassy_time::Instant::now() + Duration::from_secs(5));
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

        // Clear LEDs beyond active count so they don't hold stale colors
        for led in buf[num_leds..].iter_mut() {
            *led = RGB8 { r: 0, g: 0, b: 0 };
        }
        let active = &mut buf[..num_leds];

        // AUX7 strobe override: fast white strobe (~25 Hz) with short attack/decay
        if aux_strobe > 0 {
            // 4-frame cycle: 2 on, 2 off → 25 Hz at 100 FPS
            const STROBE_HALF: u32 = 2;
            const STROBE_PERIOD: u32 = STROBE_HALF * 2;
            let peak = aux_strobe;
            let phase = frame_counter % STROBE_PERIOD;
            let intensity = if phase < STROBE_HALF {
                ((phase + 1) as u16 * peak as u16 / STROBE_HALF as u16) as u8
            } else {
                let off_phase = phase - STROBE_HALF;
                ((STROBE_HALF - off_phase) as u16 * peak as u16 / STROBE_HALF as u16) as u8
            };
            let color = RGB8 { r: intensity, g: intensity, b: intensity };
            for led in active.iter_mut() {
                *led = color;
            }

            match ws.write(buf.iter().copied()) {
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
            match ws.write(buf.iter().copied()) {
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

        // Test pattern override: solid color for 5 seconds, no post-processing
        if test_deadline.is_some_and(|d| embassy_time::Instant::now() < d) {
            for led in active.iter_mut() {
                *led = test_color;
            }

            match ws.write(buf.iter().copied()) {
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
        // - Armed → rainbow ripple
        // - Failsafe → sliding red bars
        // - Disarmed (FC connected) → continuous pulse, red=forbidden / green=allowed
        // - No FC → user-selected pattern from BLE app
        if fc_connected && flight_mode == FlightMode::Armed {
            armed_ripple.render(active, &mut armed_scheme);
        } else if fc_connected && flight_mode == FlightMode::Failsafe {
            render_failsafe(active, frame_counter);
        } else if fc_connected && (flight_mode == FlightMode::ArmingForbidden || flight_mode == FlightMode::ArmingAllowed) {
            // FC connected, disarmed: continuous pulse, color indicates arming state
            let mut scheme = if flight_mode == FlightMode::ArmingForbidden {
                ColorScheme::Solid(RGB8 { r: 204, g: 0, b: 0 })
            } else {
                ColorScheme::Solid(RGB8 { r: 0, g: 204, b: 0 })
            };
            fc_pulse.render(active, &mut scheme);

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

        match ws.write(buf.iter().copied()) {
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



