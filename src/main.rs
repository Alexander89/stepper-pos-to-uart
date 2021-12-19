#![cfg_attr(not(test), no_std)]
#![no_main]

mod i2c;
mod magnet_sensor;
mod usb_serial;

use core::cell::RefCell;
use core::panic::PanicInfo;
use cortex_m::interrupt::Mutex;
use embedded_hal::digital::v2::PinState;
use hal::{
    clock::GenericClockController,
    pac::{interrupt, CorePeripherals, Peripherals},
    prelude::*,
};

use magnet_sensor::MagnetSensor;
use usb_serial::UsbSerial;
use xiao_m0::hal::sercom::v2::uart::{self, BaudMode, Oversampling};
use xiao_m0::{entry, hal, Led1, Led2};

type RefMutOpt<T> = Mutex<RefCell<Option<T>>>;
static LED_1: RefMutOpt<Led1> = Mutex::new(RefCell::new(None));
static LED_2: RefMutOpt<Led2> = Mutex::new(RefCell::new(None));
static mut USB_SERIAL: Option<UsbSerial> = None;

#[entry]
fn main() -> ! {
    run()
}
fn run() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    let pins = xiao_m0::Pins::new(peripherals.PORT);

    // create i2c
    let mut i2c = i2c::I2c::init(
        &mut clocks,
        peripherals.SERCOM0,
        &mut peripherals.PM,
        pins.a4,
        pins.a5,
    );

    // setup static serial
    let usb = {
        let usb_dm = pins.usb_dm;
        let usb_dp = pins.usb_dp;
        let usb = peripherals.USB;
        let pm = &mut peripherals.PM;
        let nvic = &mut core.NVIC;
        cortex_m::interrupt::free(|_| {
            // usb serial
            let serial = UsbSerial::init(&mut clocks, usb, pm, usb_dm, usb_dp, nvic);

            unsafe {
                USB_SERIAL = Some(serial);
                USB_SERIAL.as_mut().unwrap()
            }
        })
    };

    // Setup UART peripheral
    let mut uart = {
        let gclk0 = clocks.gclk0();
        let clock = &clocks.sercom4_core(&gclk0).unwrap();
        let pads = uart::Pads::default().rx(pins.a7).tx(pins.a6);
        uart::Config::new(&mut peripherals.PM, peripherals.SERCOM4, pads, clock.freq())
            .baud(
                (115200 * 4).hz(),
                BaudMode::Fractional(Oversampling::Bits16),
            )
            .stop_bits(uart::StopBits::OneBit)
            .collision_detection(false)
            .parity(uart::Parity::Odd)
            .enable()
    };

    // setup static LEDs
    let led1 = pins.led1;
    let led2 = pins.led2;
    cortex_m::interrupt::free(|cs| {
        let mut l: Led1 = led1.into();
        l.set_high().unwrap();
        LED_1.borrow(cs).replace(Some(l));
        let mut l: Led2 = led2.into();
        l.set_high().unwrap();
        LED_2.borrow(cs).replace(Some(l));
    });

    let mut magnet_sensor = MagnetSensor::init();
    const STEPS_PER_RESOLUTION: i16 = 200;
    const STEPS_PER_RESOLUTION_QUARTER: i32 = 50;

    let mut last = 0;
    let mut skip = 0;
    let _ = magnet_sensor.poll_setup(&mut i2c);

    loop {
        let _ = magnet_sensor.poll(&mut i2c);
        // magnet_sensor.poll_setup(&mut i2c);
        if magnet_sensor.detected {
            let angle = {
                let r = ((magnet_sensor.raw_angle as i32 * STEPS_PER_RESOLUTION_QUARTER + 512)
                    / 1024) as i16;
                if r >= STEPS_PER_RESOLUTION {
                    r - STEPS_PER_RESOLUTION
                } else {
                    r
                }
            };

            if last == angle && skip < 0x4000 {
                skip += 1;
                // just a guess but still not a good idea to add this additional delay :-(
                if skip == 0x4000 {
                    let _ = magnet_sensor.poll_setup(&mut i2c);
                }
            } else {
                last = angle;
                skip = 0;
                let _ = uart.write((angle & 0xFF) as u8);

                // serial.write(b" ").unwrap();
                usb.serial_write(b"r<: ");
                usb.serial_write_num(magnet_sensor.raw_angle as usize);
                usb.serial_write(b" st: ");
                usb.serial_write_num(angle as usize);

                usb.serial_write(b" agc: ");
                usb.serial_write_num(magnet_sensor.agc as usize);

                usb.serial_write(b" mag: ");
                usb.serial_write_num(magnet_sensor.magnitude as usize);

                usb.serial_write(b"\r\n");
            }
        }
    }
}

#[interrupt]
fn USB() {
    unsafe {
        USB_SERIAL.as_mut().map(|usb_dev| {
            usb_dev.poll();
            // TODO add configuration?
            let _ = usb_dev.read_poll();
        });
    };
}

#[inline(never)]
#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    let mut state: PinState = PinState::High;

    for _ in 0..20 {
        for _ in 0..0xfffff {
            cortex_m::asm::nop();
        }
        cortex_m::interrupt::free(|cs| {
            let mut l = LED_2.borrow(cs).borrow_mut();
            if l.is_some() {
                let _res = l.as_mut().unwrap().set_state(state);
            }
            let mut l = LED_1.borrow(cs).borrow_mut();
            if l.is_some() {
                let _res = l.as_mut().unwrap().set_state(!state);
            }
        });
        state = !state;
    }
    hal::pac::SCB::sys_reset();
}
