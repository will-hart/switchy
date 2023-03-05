//! Configures the microcontroller for use and returns the requried pins

use hal::{
    gpio::{ErasedPin, Input, Output, PinState},
    otg_fs::{UsbBusType, USB},
    pac::{self, TIM2},
    prelude::*,
    timer::MonoTimerUs,
};
use rotary_encoder_hal::Rotary;
use shift_register_hal::ShiftRegister;
use stm32f4xx_hal as hal;
use switch_hal::{ActiveHigh, IntoSwitch, Switch};
use usb_device::class_prelude::UsbBusAllocator;

use crate::usb::interface::UsbInterface;

macro_rules! encoder {
    ($pin_a: expr, $pin_b: expr) => {{
        Rotary::new(
            $pin_a.into_pull_up_input().erase(),
            $pin_b.into_pull_up_input().erase(),
        )
    }};
}

macro_rules! shift_register {
    ($clk_pin: expr, $latch_pin: expr, $data_pin: expr) => {{
        ShiftRegister::new(
            $clk_pin
                .into_push_pull_output_in_state(PinState::High)
                .erase(),
            $latch_pin
                .into_push_pull_output_in_state(PinState::High)
                .erase(),
            $data_pin.into_input().erase(),
        )
    }};
}

/// Configures the micro for operation
pub fn configure<'a>(
    _core_peripherals: cortex_m::Peripherals,
    device_peripherals: pac::Peripherals,
    usb_alloc: &'static mut Option<UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>>,
    usb_mem: &'static mut [u32; 1024],
) -> Configuration<'a> {
    // Take ownership over raw device and convert it into the corresponding HAL struct
    let rcc = device_peripherals.RCC.constrain();

    #[cfg(feature = "dev_board")]
    let hse_freq = 25.MHz();
    #[cfg(not(feature = "dev_board"))]
    let hse_freq = 16.MHz();

    // Freeze the configuration of all the clocks in the system and store the
    // frozen frequencies in `clocks`
    let clocks = rcc
        .cfgr
        .use_hse(hse_freq)
        .sysclk(84.MHz())
        .require_pll48clk()
        .freeze();
    assert!(clocks.is_pll48clk_valid());

    let timer: MonoTimerUs<TIM2> = device_peripherals.TIM2.monotonic_us(&clocks);

    // Acquire GPIO
    let gpioa = device_peripherals.GPIOA.split();
    let gpiob = device_peripherals.GPIOB.split();
    let gpioc = device_peripherals.GPIOC.split();

    // set up the flashy LED
    #[cfg(feature = "board_rev_3")]
    let pin = gpioc.pc3;
    // NOTE: For rev 1/2 on the board there is no LED.
    //       This setup is for the black pill dev board.
    //       In rev 3 this should change to PC3
    //       PC3 and PC13 are both unused in rev 1/2, so no issues here.
    #[cfg(not(feature = "board_rev_3"))]
    let pin = gpioc.pc13;

    let led_pin = pin
        .into_push_pull_output()
        .erase()
        .into_active_high_switch();

    // configure USB
    let usb = USB {
        usb_global: device_peripherals.OTG_FS_GLOBAL,
        usb_device: device_peripherals.OTG_FS_DEVICE,
        usb_pwrclk: device_peripherals.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };

    *usb_alloc = Some(UsbBusType::new(usb, usb_mem));
    let usb_allocator = usb_alloc.as_ref().unwrap();

    // setup the shift registers
    let bank1 = shift_register!(gpiob.pb3, gpiob.pb5, gpiob.pb4);
    let bank2 = shift_register!(gpioc.pc10, gpioc.pc11, gpioc.pc12);
    let encoder1 = encoder!(gpiob.pb12, gpiob.pb13);
    let encoder2 = encoder!(gpiob.pb14, gpiob.pb15);
    let encoder3 = encoder!(gpioc.pc7, gpioc.pc6);
    let encoder4 = encoder!(gpioc.pc9, gpioc.pc8);

    Configuration {
        usb: UsbInterface::new(usb_allocator),
        led_pin,
        timer,

        bank1,
        bank2,

        encoder1,
        encoder2,
        encoder3,
        encoder4,
    }
}

pub struct Configuration<'a> {
    pub led_pin: Switch<ErasedPin<Output>, ActiveHigh>,
    pub timer: MonoTimerUs<TIM2>,
    pub usb: UsbInterface<'a>,

    pub bank1: ShiftRegister<16, ErasedPin<Input>, ErasedPin<Output>>,
    pub bank2: ShiftRegister<16, ErasedPin<Input>, ErasedPin<Output>>,

    pub encoder1: Rotary<ErasedPin<Input>, ErasedPin<Input>>,
    pub encoder2: Rotary<ErasedPin<Input>, ErasedPin<Input>>,
    pub encoder3: Rotary<ErasedPin<Input>, ErasedPin<Input>>,
    pub encoder4: Rotary<ErasedPin<Input>, ErasedPin<Input>>,
}
