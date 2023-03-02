#![no_main]
#![no_std]

use switchy_rtic as _; // global logger + panicking-behavior + memory layout

use stm32f4xx_hal as hal;
use switch_hal::{ActiveHigh, OutputSwitch, Switch};
use systick_monotonic::*;
use usb_device::class_prelude::UsbBusAllocator;

use switchy_rtic::configure;

use hal::gpio::{ErasedPin, Output};

#[rtic::app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [EXTI1]
)]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<1000>;

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

        // Setup the monotonic timer

        let config = configure::configure(cx.core, cx.device, cx.local.USB_BUS, cx.local.USB_MEM);

        welcome::spawn().unwrap();
        blink::spawn_after(systick_monotonic::ExtU64::millis(250)).unwrap();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                led_pin: config.led_pin,
                led_state: false,
            },
            init::Monotonics(config.timer),
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    #[task]
    fn welcome(_cx: welcome::Context) {
        defmt::info!("Switchy Online!");
    }

    #[task(local = [led_pin, led_state])]
    fn blink(cx: blink::Context) {
        if *cx.local.led_state {
            cx.local.led_pin.off().ok();
            *cx.local.led_state = false;
            blink::spawn_after(systick_monotonic::ExtU64::millis(500)).unwrap();
        } else {
            cx.local.led_pin.on().ok();
            *cx.local.led_state = true;
            blink::spawn_after(systick_monotonic::ExtU64::millis(1500)).unwrap();
        }
    }
}
