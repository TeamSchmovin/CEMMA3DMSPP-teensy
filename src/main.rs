#![feature(abi_unadjusted)]
#![feature(link_llvm_intrinsics)]
#![feature(array_chunks)]
#![feature(slice_flatten)]
#![feature(exclusive_range_pattern)]
#![feature(maybe_uninit_slice)]
#![no_std]
#![no_main]

extern crate alloc;

mod intrinsics;
mod peripherals;
mod pins;

use core::arch::asm;
use core::ptr;

use cortex_m::delay::Delay;
use cortex_m::interrupt;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::prelude::{
    _embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_spi_Transfer,
    _embedded_hal_blocking_spi_Write,
};
use cortex_m::register::basepri;
use embedded_alloc::Heap;
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeManager};
use teensy4_bsp::board::{
    self, lpi2c_baud, prepare_clocks_and_power, Lpi2c1, Lpi2c3, Lpi2cClockSpeed, LpspiPins,
    ARM_FREQUENCY,
};
use teensy4_bsp::hal;
use teensy4_bsp::hal::gpio::Port;
use teensy4_bsp::hal::iomuxc::into_pads;
use teensy4_bsp::hal::lpuart::Pins;
use teensy4_bsp::pins::t41::*;
use teensy4_bsp::ral::{self, modify_reg};
#[allow(unused_imports)]
use teensy4_panic as _;

use crate::intrinsics::init_heap;

#[global_allocator]
static mut HEAP: Heap = Heap::empty();

static mut FRAMEBUFFER_TRANSFER: [u8; 57601] = [0x0; 57601];

#[teensy4_bsp::rt::entry]
fn main() -> ! {
    // interrupt::disable();
    unsafe {
        // asm!("CPSID f");
        // basepri::write(0);
    }

    unsafe {
        init_heap(&HEAP);
    }

    prepare_clocks_and_power(
        &mut peripherals::ccm(),
        &mut peripherals::ccm_analog(),
        &mut peripherals::dcdc(),
    );

    peripherals::dcb().enable_trace();
    peripherals::dwt().enable_cycle_counter();

    modify_reg!(ral::iomuxc_gpr, peripherals::iomuxc_gpr(), GPR1, CM7_FORCE_HCLK_EN: CM7_FORCE_HCLK_EN_1);
    unsafe {
        let mut scb = peripherals::scb();
        scb.enable_icache();
        scb.enable_dcache(&mut peripherals::cpuid());

        // enable SEVONPEND (for systick), disable DEEPSLEEP
        // scb.scr.write(0b10000);
    }

    // let mut systick = peripherals::syst();
    // systick.set_clock_source(SystClkSource::Core);
    // systick.clear_current();
    // systick.enable_interrupt();

    let iomuxc = into_pads(peripherals::iomuxc());
    let pins = from_pads(iomuxc);

    // let camera1_spi = teensy4_bsp::board::lpspi(
    //     peripherals::lpspi3(),
    //     teensy4_bsp::board::LpspiPins {
    //         sdo: pins.p11,
    //         sdi: pins.p12,
    //         sck: pins.p13,
    //         pcs0: pins.p0,
    //     },
    //     8_000_000,
    // );

    // let mut camera1_spi = teensy4_bsp::board::lpspi(
    //     peripherals::lpspi4(),
    //     teensy4_bsp::board::LpspiPins {
    //         sdo: pins.p11,
    //         sdi: pins.p12,
    //         sck: pins.p13,
    //         pcs0: pins.p10,
    //     },
    //     8_000_000,
    // );

    // camera1_spi.set_enable(true);
    // let _ = camera1_spi.write(&[0x81_u8, 0x00_u8]);

    // // may need to make bit 3 true, controls reverse pclk
    // let _ = camera1_spi.write(&[0x83_u8, 0x00_u8]);

    // let accel_gyro = Lpi2c1::new(lpi2c, pins, timings);
    let mut camera1_i2c = Lpi2c3::new(
        peripherals::lpi2c3(),
        hal::lpi2c::Pins {
            scl: pins.p16,
            sda: pins.p17,
        },
        &lpi2c_baud(Lpi2cClockSpeed::KHz100),
    );
    camera1_i2c.set_controller_enable(true);

    // initialize ov2640 registers
    // camera1_i2c.write(0x05_u8, &[0b00000001_u8]);
    // camera1_i2c.write(0xC2_u8, &[0b00000010_u8]);
    let ov2640_reg_values: [(u8, u8); 221] = [
        (0xff, 0x00),
        (0xC2, 0b00000010),
        (0x2c, 0xff),
        (0x2e, 0xdf),
        (0xff, 0x01),
        (0x3c, 0x32),
        (0x11, 0x00),
        (0x09, 0x02),
        (0x04, 0xA8),
        (0x13, 0xe5),
        (0x14, 0x48),
        (0x2c, 0x0c),
        (0x33, 0x78),
        (0x3a, 0x33),
        (0x3b, 0xfB),
        (0x3e, 0x00),
        (0x43, 0x11),
        (0x16, 0x10),
        (0x4a, 0x81),
        (0x21, 0x99),
        (0x24, 0x40),
        (0x25, 0x38),
        (0x26, 0x82),
        (0x5c, 0x00),
        (0x63, 0x00),
        (0x46, 0x3f),
        (0x0c, 0x3c),
        (0x61, 0x70),
        (0x62, 0x80),
        (0x7c, 0x05),
        (0x20, 0x80),
        (0x28, 0x30),
        (0x6c, 0x00),
        (0x6d, 0x80),
        (0x6e, 0x00),
        (0x70, 0x02),
        (0x71, 0x94),
        (0x73, 0xc1),
        (0x3d, 0x34),
        (0x5a, 0x57),
        (0x12, 0x00),
        (0x11, 0x00),
        (0x17, 0x11),
        (0x18, 0x75),
        (0x19, 0x01),
        (0x1a, 0x97),
        (0x32, 0x36),
        (0x03, 0x0f),
        (0x37, 0x40),
        (0x4f, 0xbb),
        (0x50, 0x9c),
        (0x5a, 0x57),
        (0x6d, 0x80),
        (0x6d, 0x38),
        (0x39, 0x02),
        (0x35, 0x88),
        (0x22, 0x0a),
        (0x37, 0x40),
        (0x23, 0x00),
        (0x34, 0xa0),
        (0x36, 0x1a),
        (0x06, 0x02),
        (0x07, 0xc0),
        (0x0d, 0xb7),
        (0x0e, 0x01),
        (0x4c, 0x00),
        (0xff, 0x00),
        (0xe5, 0x7f),
        (0xf9, 0xc0),
        (0x41, 0x24),
        (0xe0, 0x14),
        (0x76, 0xff),
        (0x33, 0xa0),
        (0x42, 0x20),
        (0x43, 0x18),
        (0x4c, 0x00),
        (0x87, 0xd0),
        (0x88, 0x3f),
        (0xd7, 0x03),
        (0xd9, 0x10),
        (0xd3, 0x82),
        (0xc8, 0x08),
        (0xc9, 0x80),
        (0x7d, 0x00),
        (0x7c, 0x03),
        (0x7d, 0x48),
        (0x7c, 0x08),
        (0x7d, 0x20),
        (0x7d, 0x10),
        (0x7d, 0x0e),
        (0x90, 0x00),
        (0x91, 0x0e),
        (0x91, 0x1a),
        (0x91, 0x31),
        (0x91, 0x5a),
        (0x91, 0x69),
        (0x91, 0x75),
        (0x91, 0x7e),
        (0x91, 0x88),
        (0x91, 0x8f),
        (0x91, 0x96),
        (0x91, 0xa3),
        (0x91, 0xaf),
        (0x91, 0xc4),
        (0x91, 0xd7),
        (0x91, 0xe8),
        (0x91, 0x20),
        (0x92, 0x00),
        (0x93, 0x06),
        (0x93, 0xe3),
        (0x93, 0x02),
        (0x93, 0x02),
        (0x93, 0x00),
        (0x93, 0x04),
        (0x93, 0x00),
        (0x93, 0x03),
        (0x93, 0x00),
        (0x93, 0x00),
        (0x93, 0x00),
        (0x93, 0x00),
        (0x93, 0x00),
        (0x93, 0x00),
        (0x93, 0x00),
        (0x96, 0x00),
        (0x97, 0x08),
        (0x97, 0x19),
        (0x97, 0x02),
        (0x97, 0x0c),
        (0x97, 0x24),
        (0x97, 0x30),
        (0x97, 0x28),
        (0x97, 0x26),
        (0x97, 0x02),
        (0x97, 0x98),
        (0x97, 0x80),
        (0x97, 0x00),
        (0x97, 0x00),
        (0xc3, 0xef),
        (0xff, 0x00),
        (0xba, 0xdc),
        (0xbb, 0x08),
        (0xb6, 0x24),
        (0xb8, 0x33),
        (0xb7, 0x20),
        (0xb9, 0x30),
        (0xb3, 0xb4),
        (0xb4, 0xca),
        (0xb5, 0x43),
        (0xb0, 0x5c),
        (0xb1, 0x4f),
        (0xb2, 0x06),
        (0xc7, 0x00),
        (0xc6, 0x51),
        (0xc5, 0x11),
        (0xc4, 0x9c),
        (0xbf, 0x00),
        (0xbc, 0x64),
        (0xa6, 0x00),
        (0xa7, 0x1e),
        (0xa7, 0x6b),
        (0xa7, 0x47),
        (0xa7, 0x33),
        (0xa7, 0x00),
        (0xa7, 0x23),
        (0xa7, 0x2e),
        (0xa7, 0x85),
        (0xa7, 0x42),
        (0xa7, 0x33),
        (0xa7, 0x00),
        (0xa7, 0x23),
        (0xa7, 0x1b),
        (0xa7, 0x74),
        (0xa7, 0x42),
        (0xa7, 0x33),
        (0xa7, 0x00),
        (0xa7, 0x23),
        (0xc0, 0xc8),
        (0xc1, 0x96),
        (0x8c, 0x00),
        (0x86, 0x3d),
        (0x50, 0x92),
        (0x51, 0x90),
        (0x52, 0x2c),
        (0x53, 0x00),
        (0x54, 0x00),
        (0x55, 0x88),
        (0x5a, 0x50),
        (0x5b, 0x3c),
        (0x5c, 0x00),
        (0xd3, 0x04),
        (0x7f, 0x00),
        (0xda, 0x00),
        (0xe5, 0x1f),
        (0xe1, 0x67),
        (0xe0, 0x00),
        (0xdd, 0x7f),
        (0x05, 0x00),
        (0xff, 0x00),
        (0xe0, 0x04),
        (0xc0, 0xc8),
        (0xc1, 0x96),
        (0x86, 0x3d),
        (0x50, 0x92),
        (0x51, 0x90),
        (0x52, 0x2c),
        (0x53, 0x00),
        (0x54, 0x00),
        (0x55, 0x88),
        (0x57, 0x00),
        (0x5a, 0x28),
        (0x5b, 0x1E),
        (0x5c, 0x00),
        (0xd3, 0x08),
        (0xe0, 0x00),
        (0xFF, 0x00),
        (0x05, 0x00),
        (0xDA, 0x08),
        (0xda, 0x09),
        (0x98, 0x00),
        (0x99, 0x00),
        (0x00, 0x00),
    ];

    for reg_val in ov2640_reg_values {
        let _ = camera1_i2c.write(reg_val.0, &[reg_val.1]);
    }

    // // loop {
    // // start capturing
    // let _ = camera1_spi.write(&[0x84_u8, 0b00000010_u8]);

    // // wait until capture is done
    // loop {
    //     let mut register = [0x41_u8];
    //     let recieved_val = unsafe {
    //         camera1_spi
    //             .transfer(&mut register)
    //             .unwrap_unchecked()
    //             .first()
    //             .unwrap_unchecked()
    //     };

    //     if (recieved_val & 0b00001000_u8) != 0 {
    //         break;
    //     }
    // }

    // // clear capture done flag
    // let _ = camera1_spi.write(&[0x41_u8, 0b00000001_u8]);

    // unsafe {
    //     FRAMEBUFFER_TRANSFER[0] = 0x3C;
    // }

    // // read multiple bytes at once
    // let recieved_frame_info = unsafe {
    //     camera1_spi
    //         .transfer(&mut FRAMEBUFFER_TRANSFER)
    //         .unwrap_unchecked()
    // };

    // unsafe {
    //     FRAMEBUFFER_TRANSFER.copy_from_slice(recieved_frame_info);
    // }

    // camera1_spi.disabled(|_| {});

    let mut sd_spi = teensy4_bsp::board::lpspi(
        peripherals::lpspi1(),
        teensy4_bsp::board::LpspiPins {
            sdo: pins.p43,
            sdi: pins.p42,
            sck: pins.p45,
            pcs0: unsafe { ptr::read(&pins.p44 as *const _) },
        },
        25_000_000,
    );

    sd_spi.set_enable(true);

    let cs_pin = Port::new(peripherals::gpio3()).output(pins.p44);

    let sdcard = embedded_sdmmc::SdCard::new(
        sd_spi,
        cs_pin,
        Delay::new(peripherals::syst(), ARM_FREQUENCY),
    );
    let mut volume_mgr = VolumeManager::new(sdcard, NoopTimeSource);
    let volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .unwrap();
    // let root_dir = volume_mgr.open_root_dir(volume0).unwrap();
    // let my_file = volume_mgr
    //     .open_file_in_dir(
    //         root_dir,
    //         "MY_FILE.TXT",
    //         embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
    //     )
    //     .unwrap();

    // let _ = volume_mgr.write(my_file, &[0x0, 0x1, 0x2, 0x3]); //unsafe { &FRAMEBUFFER_TRANSFER });
    // volume_mgr.close_file(my_file).unwrap();
    // volume_mgr.close_dir(root_dir).unwrap();
    // volume_mgr.close_volume(volume0).unwrap();

    // if unsafe { FRAMEBUFFER_TRANSFER[1] == 0x00_u8 } {
    //     panic!();
    // }
    // }
    // loop {}
    // panic!()
    loop {}
}

pub struct NoopTimeSource;

impl TimeSource for NoopTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        Timestamp::from_fat(0, 0)
    }
}
