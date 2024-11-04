//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
//! https://github.com/rp-rs/rp-hal-boards/blob/main/boards/rp-pico/examples/pico_pwm_blink.rs
#![no_std]
#![no_main]

use core::cell::Cell;
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use panic_probe as _;
use rp_pico;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::prelude::*;
use rp_pico::hal::{
    clocks::init_clocks_and_plls, pac, pac::interrupt, sio::Sio, watchdog::Watchdog,
};
static GLOBAL_BUTTON: Mutex<
    RefCell<
        Option<
            rp_pico::hal::gpio::Pin<
                rp_pico::hal::gpio::bank0::Gpio0,
                rp_pico::hal::gpio::FunctionSio<rp_pico::hal::gpio::SioInput>,
                rp_pico::hal::gpio::PullDown,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

static LED_ON: Mutex<Cell<bool>> = Mutex::new(Cell::new(true));

#[entry]
fn main() -> ! {
    info!("Program start");
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

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM7
    let pwm = &mut pwm_slices.pwm7;
    // need to find a clean way to set the freq from the divider and top value
    // see description on page 530 of rp2040 sheet for equation
    pwm.set_div_int(40);
    pwm.set_top(0xffff);
    pwm.set_div_int(4);
    pwm.disable(); // toggle to enable

    // Output channel B on PWM7 to the Buzzer pin
    let buzzer = &mut pwm.channel_b;
    buzzer.output_to(pins.gpio15);
    buzzer.set_duty_cycle_percent(50).unwrap();
    buzzer.set_enabled(true);
    // buzzer.

    let mut led_pin = pins.led.into_push_pull_output();
    let button_pin = pins.gpio0.into_pull_down_input();

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
    button_pin.set_interrupt_enabled(rp_pico::hal::gpio::Interrupt::EdgeLow, true);
    button_pin.set_interrupt_enabled(rp_pico::hal::gpio::Interrupt::EdgeHigh, true);

    critical_section::with(|cs| {
        GLOBAL_BUTTON.borrow(cs).replace(Some(button_pin));
    });

    defmt::info!("Start");
    loop {
        cortex_m::asm::wfe();
        defmt::info!("Waking up");
        critical_section::with(|cs| match LED_ON.borrow(cs).get() {
            false => {
                led_pin.set_low().unwrap();
            }
            true => {
                led_pin.set_high().unwrap();
            }
        });
    }
}

#[pac::interrupt]
fn IO_IRQ_BANK0() {
    static mut BUTTON: Option<
        rp_pico::hal::gpio::Pin<
            rp_pico::hal::gpio::bank0::Gpio0,
            rp_pico::hal::gpio::FunctionSio<rp_pico::hal::gpio::SioInput>,
            rp_pico::hal::gpio::PullDown,
        >,
    > = None;
    defmt::info!("Interrupt triggered");
    if BUTTON.is_none() {
        critical_section::with(|cs| {
            *BUTTON = GLOBAL_BUTTON.borrow(cs).take();
        });
    };

    critical_section::with(|cs| {
        if let Some(button) = BUTTON {
            let led_on = LED_ON.borrow(cs);
            let edge_low = button.interrupt_status(rp_pico::hal::gpio::Interrupt::EdgeLow);
            if edge_low {
                defmt::info!("Edge Low");
                button.clear_interrupt(rp_pico::hal::gpio::Interrupt::EdgeLow);
                led_on.set(false);
            }
            let edge_high = button.interrupt_status(rp_pico::hal::gpio::Interrupt::EdgeHigh);
            if edge_high {
                defmt::info!("Edge High");
                button.clear_interrupt(rp_pico::hal::gpio::Interrupt::EdgeHigh);
                led_on.set(true);
            }
        }
    });
    cortex_m::asm::sev();
}

// End of file
