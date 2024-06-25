#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use defmt_rtt as _;
use panic_probe as _;
use pwm_pca9685::{Address, Channel, Pca9685};
use rp_pico as bsp;
use rp_pico::hal::fugit::RateExtU32;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin = pins.gpio4.reconfigure();
    let scl_pin = pins.gpio5.reconfigure();

    let i2c = rp_pico::hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400u32.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let address = Address::default();
    let mut pca9685 = Pca9685::new(i2c, address).unwrap();

    // PCA9685 OSC = 25MHz  Resolution = 4096  ServoFreq = 50Hz
    let prescale = 121; // prescale = OSC / (Resolution * ServoFreq) - 1
    let servo_min = 102; // min = 0.5ms / (20ms / 4096)
    let servo_max = 491; // max = 2.4ms / (20ms / 4096)

    pca9685.set_prescale(prescale as u8).unwrap();
    pca9685.enable().unwrap();

    pca9685.set_channel_on(Channel::C0, 0).unwrap();

    loop {
        for i in servo_min..=servo_max {
            pca9685.set_channel_off(Channel::C0, i).unwrap();
            delay.delay_ms(10);
        }
        for i in (servo_min..=servo_max).rev() {
            pca9685.set_channel_off(Channel::C0, i).unwrap();
            delay.delay_ms(10);
        }
    }
}
