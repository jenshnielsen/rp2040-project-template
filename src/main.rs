//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
<<<<<<< HEAD
use embedded_hal::digital::OutputPin;
=======
use embedded_hal::digital::v2::{InputPin, OutputPin};
>>>>>>> 455c8fb (Simple but working exp2)
use panic_probe as _;
use rp_pico;
use rp_pico::entry;
use rp_pico::hal::{
    clocks::init_clocks_and_plls, pac, pac::interrupt, sio::Sio, watchdog::Watchdog,
};

static GLOBAL_BUTTON: Mutex<
    RefCell<
        Option<
            rp_pico::hal::gpio::Pin<
                rp_pico::hal::gpio::bank0::Gpio2,
                rp_pico::hal::gpio::FunctionSio<rp_pico::hal::gpio::SioInput>,
                rp_pico::hal::gpio::PullDown,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let _ = init_clocks_and_plls(
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

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let mut led_pin = pins.led.into_push_pull_output();
    let button_pin = pins.gpio2.into_pull_down_input();

    unsafe {
        pac::NVIC::unmask(rp_pico::hal::pac::Interrupt::IO_IRQ_BANK0);
    }
    button_pin.set_interrupt_enabled(rp_pico::hal::gpio::Interrupt::EdgeLow, true);
    // button_pin.clear_interrupt(rp_pico::hal::gpio::Interrupt::EdgeLow);

    critical_section::with(|cs| {
        GLOBAL_BUTTON.borrow(cs).replace(Some(button_pin));
    });

    defmt::info!("Start");
    loop {}
}

#[pac::interrupt]
fn IO_IRQ_BANK0() {
    static mut BUTTON: Option<
        rp_pico::hal::gpio::Pin<
            rp_pico::hal::gpio::bank0::Gpio2,
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
    if let Some(button) = BUTTON {
        button.clear_interrupt(rp_pico::hal::gpio::Interrupt::EdgeLow);
    }
}

// End of file
