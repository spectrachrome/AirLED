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
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::{
    AccessPointConfig, ModeConfig, WifiApState, ap_state,
};
use panic_rtt_target as _;
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_spi::prerendered::Ws2812;
use xiao_drone_led_controller::pattern::{
    Animation, ColorScheme, Pulse, RippleEffect, StaticAnim,
};
use xiao_drone_led_controller::postfx::{PostEffect, apply_pipeline};
use xiao_drone_led_controller::state::{AnimMode, AnimModeParams, ColorMode, STATE};
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

    // Start embassy executor
    static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
    executor.run(move |spawner| {
        spawner.must_spawn(led_task(spi_bus));
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

/// Build a [`ColorScheme`] from the current [`ColorMode`].
fn build_color_scheme(mode: ColorMode) -> ColorScheme {
    match mode {
        ColorMode::SolidGreen => ColorScheme::Solid(RGB8 { r: 0, g: 204, b: 0 }),
        ColorMode::SolidRed => ColorScheme::Solid(RGB8 { r: 204, g: 0, b: 0 }),
        ColorMode::Split => ColorScheme::Split(
            RGB8 { r: 0, g: 204, b: 0 },
            RGB8 { r: 204, g: 0, b: 0 },
        ),
        ColorMode::Rainbow => ColorScheme::Rainbow { hue: 0, speed: 1 },
    }
}

/// Drives the WS2812 LED strip using the active animation + color scheme via SPI+DMA.
#[embassy_executor::task]
async fn led_task(spi_bus: SpiDmaBus<'static, esp_hal::Blocking>) {
    let mut ws_buf = [0u8; SPI_BUF_LEN];
    let mut ws = Ws2812::new(spi_bus, &mut ws_buf);

    let mut pulse = Pulse::new();
    let mut ripple = RippleEffect::new(0xDEAD_BEEF);
    let mut static_anim = StaticAnim;

    let mut color_scheme = build_color_scheme(ColorMode::Split);
    let mut prev_color_mode = ColorMode::Split;

    let mut buf = [RGB8 { r: 0, g: 0, b: 0 }; MAX_LEDS];
    let mut write_err_logged = false;

    loop {
        let state = STATE.lock().await;
        let num_leds = state.num_leds.min(MAX_NUM_LEDS) as usize;
        let led_brightness = state.brightness;
        let max_ma = state.max_current_ma;
        let fps = state.fps.max(1);
        let color_mode = state.color_mode;
        let color_params = state.color_params;
        let anim_mode = state.anim_mode;
        let anim_params = state.anim_params;
        drop(state);

        // Rebuild color scheme when the mode changes, preserving rainbow hue otherwise.
        if color_mode != prev_color_mode {
            color_scheme = build_color_scheme(color_mode);
            prev_color_mode = color_mode;
        }
        color_scheme.set_hue_speed(color_params.hue_speed);

        // Apply live animation parameters.
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

        let active = &mut buf[..num_leds];
        match anim_mode {
            AnimMode::Static => static_anim.render(active, &mut color_scheme),
            AnimMode::Pulse => pulse.render(active, &mut color_scheme),
            AnimMode::Ripple => ripple.render(active, &mut color_scheme),
        }

        let pipeline = [
            PostEffect::Gamma,
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
<div class="g pm" data-color="rainbow"><label>Hue Speed <span class="v" id="hsv">{hue_speed}</span></label>
<input type="range" id="hs" min="1" max="10" value="{hue_speed}"></div>
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
var cm=document.getElementById('cm'),am=document.getElementById('am');
var bv=document.getElementById('bv'),lv=document.getElementById('lv'),fv=document.getElementById('fv'),cv=document.getElementById('cv');
var ps=document.getElementById('ps'),mb=document.getElementById('mb');
var rs=document.getElementById('rs'),rw=document.getElementById('rw'),rd=document.getElementById('rd');
var hs=document.getElementById('hs');
var psv=document.getElementById('psv'),mbv=document.getElementById('mbv');
var rsv=document.getElementById('rsv'),rwv=document.getElementById('rwv'),rdv=document.getElementById('rdv');
var hsv=document.getElementById('hsv');
var t;
function updateVis(){{var c=cm.value,a=am.value;document.querySelectorAll('.pm').forEach(function(el){{var sc=el.dataset.color,sa=el.dataset.anim;var show=true;if(sc)show=show&&sc===c;if(sa)show=show&&sa===a;el.style.display=show?'':'none'}})}}
var sb=document.getElementById('sb'),con=null,ht;
function toast(ok){{if(ok===con)return;con=ok;sb.textContent=ok?'Connected':'Disconnected';sb.className=ok?'show ok':'show err';clearTimeout(ht);if(ok)ht=setTimeout(function(){{sb.className=''}},2000)}}
function send(){{var q='brightness='+br.value+'&num_leds='+lc.value+'&fps='+fp.value+'&max_current_ma='+cl.value+'&color='+cm.value+'&anim='+am.value;var c=cm.value,a=am.value;if(c==='rainbow')q+='&hue_speed='+hs.value;if(a==='pulse')q+='&pulse_speed='+ps.value+'&min_brightness='+mb.value;if(a==='ripple')q+='&ripple_speed='+rs.value+'&ripple_width='+rw.value+'&ripple_decay='+rd.value;fetch('/set?'+q).then(function(){{toast(true)}}).catch(function(){{toast(false)}})}}
setInterval(function(){{fetch('/set').then(function(){{toast(true)}}).catch(function(){{toast(false)}})}},3000);
function sl(el,vl){{el.oninput=function(){{vl.textContent=el.value;clearTimeout(t);t=setTimeout(send,80)}}}}
sl(br,bv);sl(lc,lv);sl(fp,fv);sl(cl,cv);sl(ps,psv);sl(mb,mbv);sl(rs,rsv);sl(rw,rwv);sl(rd,rdv);sl(hs,hsv);
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
        hue_speed = hue_speed,
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
