#![no_main]
#![no_std]

use switchy_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [EXTI1]
)]
mod app {
    use stm32f4xx_hal as hal;
    use switch_hal::{ActiveHigh, IntoSwitch, OutputSwitch, Switch};
    use systick_monotonic::*;

    use hal::gpio::{ErasedPin, Output};
    use hal::prelude::*;

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

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        // Setup the monotonic timer
        let mono = Systick::new(cx.core.SYST, 84_000_000);

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(16.MHz())
            .sysclk(84.MHz())
            .require_pll48clk()
            .freeze();
        assert!(clocks.is_pll48clk_valid());

        // Acquire GPIO
        // let gpioa = cx.device.GPIOA.split();
        // let gpiob = device_peripherals.GPIOB.split();
        let gpioc = cx.device.GPIOC.split();

        let mut led_pin = gpioc
            .pc13
            .into_push_pull_output()
            .erase()
            .into_active_high_switch();
        led_pin.off().ok();

        welcome::spawn().unwrap();
        blink::spawn_after(systick_monotonic::ExtU64::secs(1)).unwrap();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                led_pin,
                led_state: false,
            },
            init::Monotonics(mono),
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
        } else {
            cx.local.led_pin.on().ok();
            *cx.local.led_state = true;
        }

        blink::spawn_after(systick_monotonic::ExtU64::secs(1)).unwrap();
    }
}
