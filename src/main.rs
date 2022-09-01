#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use rp_pico::{
    entry,
    hal::{
        self,
        gpio::{
            bank0::{Gpio0, Gpio1},
            Output, Pin, PushPull,
        },
        pac,
        pwm::{FreeRunning, InputHighRunning},
        Clock,
    },
};

fn set_up(out_1: &mut Pin<Gpio0, Output<PushPull>>, out_2: &mut Pin<Gpio1, Output<PushPull>>) {
    out_1.set_high().unwrap();
    out_2.set_low().unwrap();
}

fn set_down(out_1: &mut Pin<Gpio0, Output<PushPull>>, out_2: &mut Pin<Gpio1, Output<PushPull>>) {
    out_1.set_low().unwrap();
    out_2.set_high().unwrap();
}

fn set_stop(out_1: &mut Pin<Gpio0, Output<PushPull>>, out_2: &mut Pin<Gpio1, Output<PushPull>>) {
    out_1.set_high().unwrap();
    out_2.set_high().unwrap();
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    let mut out_1 = pins.gpio0.into_push_pull_output();
    let mut out_2 = pins.gpio1.into_push_pull_output();

    let pwm_pin = pins.gpio15.into_floating_input();
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm = pwm_slices.pwm7.into_mode::<InputHighRunning>();
    let pwm_pin_token = pwm.input_from(pwm_pin);

    let count = pwm.get_counter();

    loop {
        set_up(&mut out_1, &mut out_2);
        led_pin.set_high().unwrap();
        delay.delay_ms(500);

        set_down(&mut out_1, &mut out_2);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);

        set_stop(&mut out_1, &mut out_2);
        led_pin.set_high().unwrap();
        delay.delay_ms(2000);
    }
}
