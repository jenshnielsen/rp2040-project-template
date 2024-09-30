//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
//! https://github.com/rp-rs/rp-hal-boards/blob/main/boards/rp-pico/examples/pico_pwm_blink.rs
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::pwm::SetDutyCycle;
use panic_probe as _;
use rp_pico;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::prelude::*;
use rp_pico::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};

#[entry]
fn main() -> ! {
    info!("Program start");
    let low = 0;
    let high = 25000;
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM7 to the on board LED
    let led = &mut pwm.channel_b;
    led.output_to(pins.led);
    led.set_duty_cycle_percent(50).unwrap();
    led.set_enabled(true);
    // buzzer.

    defmt::info!("Start");
    loop {
        // Ramp brightness up
        for i in (low..=high).skip(25) {
            delay.delay_us(32);
            led.set_duty_cycle(i).unwrap();
        }
        delay.delay_ms(2000);
        // Ramp brightness down
        for i in (low..=high).rev().skip(25) {
            delay.delay_us(32);
            led.set_duty_cycle(i).unwrap();
        }
        delay.delay_ms(2000);
    }
}
