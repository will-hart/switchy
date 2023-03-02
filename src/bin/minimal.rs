#![no_main]
#![no_std]

use switchy_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [EXTI1]
)]
mod app {
    use systick_monotonic::Systick;

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = Systick<1000>;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        // Setup the monotonic timer
        let mono = Systick::new(cx.core.SYST, 84_000_000);

        welcome::spawn().unwrap();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
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
}
