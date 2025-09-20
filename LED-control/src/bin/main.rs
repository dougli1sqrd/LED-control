#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, Pull};
use esp_hal::interrupt::Priority;
use esp_hal::ledc::channel::config::PinConfig;
use esp_hal::ledc::timer::config::Duty;
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::peripherals::{Interrupt, Peripherals};
use esp_hal::riscv::asm::wfi;
use esp_hal::timer::timg::{Timer, Timer0, TimerGroup};
use esp_hal::timer::PeriodicTimer;
use esp_hal::{interrupt, prelude::*};
use log::info;
use rotary_encoder_embedded::standard::StandardMode;
use rotary_encoder_embedded::{Direction, RotaryEncoder};

extern crate alloc;

static mut ENCODER: Option<(RotaryEncoder<StandardMode, Input<'_>, Input<'_>>, RotaryEncoder<StandardMode, Input<'_>, Input<'_>>, RotaryEncoder<StandardMode, Input<'_>, Input<'_>>)> = None;

static mut R: u8 = 0;
static mut G: u8 = 0;
static mut B: u8 = 0;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_println::logger::init_logger_from_env();

    esp_alloc::heap_allocator!(72 * 1024);

    let red_pinA = Input::new(peripherals.GPIO0, Pull::Up);
    let red_pinB = Input::new(peripherals.GPIO1, Pull::Up);

    let green_pinA = Input::new(peripherals.GPIO2, Pull::Up);
    let green_pinB = Input::new(peripherals.GPIO3, Pull::Up);

    let blue_pinA = Input::new(peripherals.GPIO4, Pull::Up);
    let blue_pinB = Input::new(peripherals.GPIO5, Pull::Up);

    let red_rotary_encoder = RotaryEncoder::new(red_pinA, red_pinB).into_standard_mode();

    let green_rotary_encoder = RotaryEncoder::new(green_pinA, green_pinB).into_standard_mode();

    let blue_rotary_encoder = RotaryEncoder::new(blue_pinA, blue_pinB).into_standard_mode();

    unsafe {
        ENCODER.replace((red_rotary_encoder, green_rotary_encoder, blue_rotary_encoder));
    }

    let mut led = Ledc::new(peripherals.LEDC);
    led.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut pwm_timer = led.timer::<LowSpeed>(timer::Number::Timer1);
    pwm_timer
        .configure(timer::config::Config {
            duty: Duty::Duty12Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 19.kHz(),
        })
        .unwrap();

    let red_pwm = peripherals.GPIO7;
    let green_pwm = peripherals.GPIO10;
    let blue_pwm = peripherals.GPIO8;

    let mut channel0 = led.channel(channel::Number::Channel0, red_pwm);
    channel0
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();
    let mut channel1 = led.channel(channel::Number::Channel1, green_pwm);
    channel1
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();

    let mut channel2 = led.channel(channel::Number::Channel2, blue_pwm);
    channel2
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();

    interrupt::enable(Interrupt::TG0_T0_LEVEL, Priority::Priority1).unwrap();

    let tg = TimerGroup::new(peripherals.TIMG0);
    let timer = tg.timer0;

    timer.set_interrupt_handler(timer_interrupt);
    timer.load_value(1.millis()).unwrap();
    timer.set_auto_reload(true);
    timer.listen();
    timer.start();

    channel0.set_duty_hw(0);
    channel1.set_duty_hw(0);
    channel2.set_duty_hw(0);

    loop {
        let old_red = unsafe { R };
        let old_green = unsafe { G };
        let old_blue = unsafe { B };
        wfi();
        let next_red = unsafe { R };
        if next_red != old_red {
            let next = next_red as u32; //percent: next / 100
            let hw = next * (1 << 12) / 100; // hw: hw / 2^11
            let vis = hw * hw / (1 << 12);
            // info!("{}%, hw: {}, vis: {}", next, hw, vis);
            let _ = channel0.set_duty_hw(vis);
        }
        let next_green = unsafe { G };
        if next_green != old_green {
            let next = next_green as u32; //percent: next / 100
            let hw = next * (1 << 12) / 100; // hw: hw / 2^11
            let vis = hw * hw / (1 << 12);
            // info!("{}%, hw: {}, vis: {}", next, hw, vis);
            let _ = channel1.set_duty_hw(vis);
        }
        let next_blue = unsafe { B };
        if next_blue != old_blue {
            let next = next_blue as u32; //percent: next / 100
            let hw = next * (1 << 12) / 100; // hw: hw / 2^11
            let vis = hw * hw / (1 << 12);
            // info!("{}%, hw: {}, vis: {}", next, hw, vis);
            let _ = channel2.set_duty_hw(vis);
        }
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.22.0/examples/src/bin
}

#[handler]
fn timer_interrupt() {
    critical_section::with(|_| {
        let timer = unsafe { TimerGroup::new(Peripherals::steal().TIMG0) }.timer0;
        timer.clear_interrupt();
        timer.load_value(1.millis()).unwrap();
        timer.start();

        if let Some((red_encoder, green_encoder, blue_encoder)) = unsafe { &mut ENCODER } {
            match red_encoder.update() {
                Direction::Clockwise => {
                    unsafe { R = R.saturating_add(2).min(100) };
                }
                Direction::Anticlockwise => {
                    unsafe { R = R.saturating_sub(2) };
                }
                Direction::None => {
                    // Do nothing
                }
            }
            match green_encoder.update() {
                Direction::Clockwise => {
                    unsafe { G = G.saturating_add(2).min(100) };
                }
                Direction::Anticlockwise => {
                    unsafe { G = G.saturating_sub(2) };
                }
                Direction::None => {
                    // Do nothing
                }
            }
            match blue_encoder.update() {
                Direction::Clockwise => {
                    unsafe { B = B.saturating_add(2).min(100) };
                }
                Direction::Anticlockwise => {
                    unsafe { B = B.saturating_sub(2) };
                }
                Direction::None => {
                    // Do nothing
                }
            }
        }
    });
}
