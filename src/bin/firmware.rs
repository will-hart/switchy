//! Author: William Hart, March 2023

#![deny(unsafe_code)]
// #![deny(warnings)]
#![deny(missing_docs)]
#![no_main]
#![no_std]

use switchy_rtic as _; // global logger + panicking-behavior + memory layout
use stm32f4xx_hal as hal;
use switch_hal::{ActiveHigh, OutputSwitch, Switch};
use usb_device::class_prelude::UsbBusAllocator;
use hal::{timer::{fugit, MonoTimerUs}, gpio::{ErasedPin, Output}};
use switchy_rtic::{configure};


#[rtic::app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [EXTI1]
)]
mod app {
    use super::*;

    #[monotonic(binds = TIM2, default = true)]
    type SysMono = MonoTimerUs<hal::pac::TIM2>;

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        led_pin: Switch<ErasedPin<Output>, ActiveHigh>,
        led_state: bool,
    }

    #[init(local = [
        USB_BUS: Option<UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> = None, 
        USB_MEM: [u32; 1024] = [0;1024]
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        // configure all the clocks and peripherals
        let config = configure::configure(cx.core, cx.device, cx.local.USB_BUS, cx.local.USB_MEM);


        // show a blinky light
        #[cfg(any(feature = "dev_board", feature = "board_rev_3"))]
        blink::spawn_after(fugit::ExtU32::secs(1u32)).unwrap();

        (
            Shared {},
            Local {
                led_pin: config.led_pin,
                led_state: false,
            },
            init::Monotonics(config.timer),
        )
    }

    #[task]
    fn welcome(_cx: welcome::Context) {
        defmt::info!("Switchy Online!");
    }
    
    #[cfg(any(feature = "dev_board", feature = "board_rev_3"))]
    #[task(local = [led_pin, led_state])]
    fn blink(cx: blink::Context) {
        if *cx.local.led_state {
            cx.local.led_pin.off().ok();
            *cx.local.led_state = false;
            blink::spawn_after(fugit::ExtU32::millis(500)).unwrap();
        } else {
            cx.local.led_pin.on().ok();
            *cx.local.led_state = true;
            blink::spawn_after(fugit::ExtU32::millis(1500)).unwrap();
        }
    }
}
