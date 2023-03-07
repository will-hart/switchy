//! The main firmware for the Switchy, based on cortex_m_rtic
//! 
//! Author: William Hart, March 2023

#![deny(unsafe_code)]
#![deny(warnings)]
#![deny(missing_docs)]
#![no_main]
#![no_std]

use switchy_rtic as _; // global logger + panicking-behavior + memory layout
use stm32f4xx_hal as hal;

use heapless::{mpmc::Q32, spsc::{Consumer, Producer, Queue}};
use switch_hal::{ActiveHigh, Switch};

#[cfg(feature = "buttons")]
use switch_hal::OutputSwitch;

use usb_device::class_prelude::UsbBusAllocator;
use hal::{
    adc::Adc,
    dma::{PeripheralToMemory, StreamX, Transfer},
    gpio::{ErasedPin, Input, Output},
    pac::{ADC1, DMA2},
    timer::{fugit, MonoTimerUs}
};

#[cfg(feature = "joysticks")]
use hal::pac::Interrupt;

#[cfg(feature = "encoders")]
use rotary_encoder_hal::{Direction};
use rotary_encoder_hal::Rotary;

use switchy_rtic::{
    usb::{
        interface::UsbInterface,
        command::{KeyAction, Command}
    },
    configure
};
use shift_register_hal::ShiftRegister;

/// The period between changing the USB HID keyboard report in ms
pub const USB_QUEUE_CONSUMPTION_DELAY_MS: u32 = 10;

/// The period for querying the inputs in us
pub const INPUT_POLL_PERIOD_US: u32 = 10;

/// The period for sampling the ADCs in ms
pub const ADC_POLL_PERIOD_MS: u32 = 100;

#[rtic::app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [EXTI1, EXTI2]
)]
mod app {
    use super::*;

    #[monotonic(binds = TIM2, default = true)]
    type SysMono = MonoTimerUs<hal::pac::TIM2>;

    // Shared resources go here
    #[shared]
    struct Shared {
        /// Used to queue up commands received from the PC for processing
        command_queue: Q32<Command>,

        /// Used to queue actions from the switchy through the USB HID to the PC
        action_sender: Producer<'static, KeyAction, 32>,

        /// The USB interface used
        usb: UsbInterface<'static>,

        /// The ADC transfer used to read ADC data from DMA
        adc_transfer: Transfer<StreamX<DMA2, 0>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut[u16; 4]>,
    }

    // Local resources go here
    #[local]
    struct Local {
        /// Used by the report sending task to read actions that should be queued
        action_receiver: Consumer<'static, KeyAction, 32>,
        /// The current action that was just sent, or None if no action was recently sent
        current_action: Option<KeyAction>,

        /// The LED pin for blinky lights
        #[cfg(feature = "blink")]
        led_pin: Switch<ErasedPin<Output>, ActiveHigh>,
        /// The current state of the blinky light
        #[cfg(feature = "blink")]
        led_state: bool,

        /// The shift registers
        #[cfg(feature = "buttons")]
        bank1: ShiftRegister<16, ErasedPin<Input>, ErasedPin<Output>>,
        #[cfg(feature = "buttons")]
        bank2: ShiftRegister<16, ErasedPin<Input>, ErasedPin<Output>>,

        // The encoders
        #[cfg(feature = "encoders")]
        encoder1: Rotary<ErasedPin<Input>, ErasedPin<Input>>,
        #[cfg(feature = "encoders")]
        encoder2: Rotary<ErasedPin<Input>, ErasedPin<Input>>,
        #[cfg(feature = "encoders")]
        encoder3: Rotary<ErasedPin<Input>, ErasedPin<Input>>,
        #[cfg(feature = "encoders")]
        encoder4: Rotary<ErasedPin<Input>, ErasedPin<Input>>,

        // The ADC buffer
        #[cfg(feature = "joysticks")]
        adc_buffer: Option<&'static mut [u16; 4]>,
    }

    #[init(local = [
        USB_BUS: Option<UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> = None, 
        USB_MEM: [u32; 1024] = [0; 1024],
        adc_buffer_raw: [u16; 4] = [0; 4],
        action_queue: Queue<KeyAction, 32> = Queue::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::println!("init");

        // configure all the clocks and peripherals
        let config = configure::configure(cx.core, cx.device, cx.local.USB_BUS, cx.local.USB_MEM);
        defmt::println!("configured");

        // configure the message passing queues
        let (action_sender, action_receiver) = cx.local.action_queue.split();

        // forward actions from the queue
        send_actions_to_pc::spawn_after(fugit::ExtU32::millis(USB_QUEUE_CONSUMPTION_DELAY_MS)).unwrap();
        defmt::println!("spawned send action to PC");

        // show a blinky light
        #[cfg(feature = "blink")]
        {
            blink::spawn_after(fugit::ExtU32::secs(1u32)).unwrap();
            defmt::println!("spawned blink");
        }

        // scan inputs
        #[cfg(feature = "buttons")]
        {
            poll_registers::spawn_after(fugit::ExtU32::micros(INPUT_POLL_PERIOD_US)).unwrap();
            defmt::println!("spawned poll registers");
        }

        // scan inputs
        #[cfg(feature = "encoders")]
        {
            poll_encoders::spawn_after(fugit::ExtU32::micros(INPUT_POLL_PERIOD_US)).unwrap();
            defmt::println!("spawned poll encoders");
        }

        #[cfg(feature = "joysticks")] 
        {
            poll_adcs::spawn_after(fugit::ExtU32::millis(ADC_POLL_PERIOD_MS)).unwrap();
            defmt::println!("spawned joysticks");
        }

        (
            Shared {
                command_queue: Q32::new(),
                usb: config.usb,
                adc_transfer: config.adc_transfer,
                action_sender,
            },
            Local {
                #[cfg(feature = "blink")]
                led_pin: config.led_pin,
                #[cfg(feature = "blink")]
                led_state: false,

                action_receiver,
                current_action: None,

                #[cfg(feature = "buttons")]
                bank1: config.bank1,
                #[cfg(feature = "buttons")]
                bank2: config.bank2,

                #[cfg(feature = "encoders")]
                encoder1: config.encoder1,
                #[cfg(feature = "encoders")]
                encoder2: config.encoder2,
                #[cfg(feature = "encoders")]
                encoder3: config.encoder3,
                #[cfg(feature = "encoders")]
                encoder4: config.encoder4,

                #[cfg(feature = "joysticks")]
                adc_buffer: Some(cx.local.adc_buffer_raw),
            },
            init::Monotonics(config.timer),
        )
    }
    
    #[task(local = [bank1, bank2], shared = [action_sender], priority = 2)]
    fn poll_registers(_cx: poll_registers::Context) {
        #[cfg(feature = "buttons")]
        {

            let bank1 = _cx.local.bank1;
            if let Some(value) = bank1.poll() {
                defmt::info!("Received bank1 value {}", value);
            }

            let bank2 = _cx.local.bank2;
            if let Some(value) = bank2.poll() {
                defmt::info!("Received bank2 value {}", value);
            }

            poll_registers::spawn_after(fugit::ExtU32::micros(INPUT_POLL_PERIOD_US)).unwrap();
        }
    }

    #[task(local = [encoder1, encoder2, encoder3, encoder4], shared = [action_sender], priority = 2)]
    fn poll_encoders(_cx: poll_encoders::Context) {
        #[cfg(any(feature = "encoders"))]
        {
            match _cx.local.encoder1.update() {
                Ok(dir) => {
                    match dir {
                        Direction::Clockwise => defmt::debug!("1 cw"),
                        Direction::CounterClockwise => defmt::debug!("1 ccw"),
                        Direction::None => {}
                    };
                }
                Err(_) => { defmt::error!("Received an error from encoder 1"); },
            }

            match _cx.local.encoder2.update() {
                Ok(dir) => {
                    match dir {
                        Direction::Clockwise => defmt::debug!("2 cw"),
                        Direction::CounterClockwise => defmt::debug!("2 ccw"),
                        Direction::None => {}
                    };
                }
                Err(_) => { defmt::error!("Received an error from encoder 2"); },
            }

            match _cx.local.encoder3.update() {
                Ok(dir) => {
                    match dir {
                        Direction::Clockwise => defmt::debug!("3 cw"),
                        Direction::CounterClockwise => defmt::debug!("3 ccw"),
                        Direction::None => {}
                    };
                }
                Err(_) => { defmt::error!("Received an error from encoder 3"); },
            }

            match _cx.local.encoder4.update() {
                Ok(dir) => {
                    match dir {
                        Direction::Clockwise => defmt::debug!("4 cw"),
                        Direction::CounterClockwise => defmt::debug!("4 ccw"),
                        Direction::None => {}
                    };
                }
                Err(_) => { defmt::error!("Received an error from encoder 4"); },
            }

            poll_encoders::spawn_after(fugit::ExtU32::micros(INPUT_POLL_PERIOD_US)).unwrap();
        }
    }


    #[task(shared = [adc_transfer], priority = 1)]
    fn poll_adcs(mut _cx: poll_adcs::Context) {
        #[cfg(feature = "joysticks")]
        {
            _cx.shared.adc_transfer.lock(|transfer| {
                transfer.start(|adc| {
                    adc.start_conversion();
                })
            });

            // poll_adcs::spawn_after(fugit::ExtU32::millis(ADC_POLL_PERIOD_MS)).unwrap();
        }
    }

    #[task]
    fn clear_dma_interrupt(_cx: clear_dma_interrupt::Context) {
        // reset the interrupt
        #[cfg(feature = "joysticks")]
        cortex_m::peripheral::NVIC::pend(Interrupt::DMA2_STREAM0);
    }

    #[task(binds = DMA2_STREAM0, shared = [adc_transfer], local = [adc_buffer], priority = 3)]
    fn dma_interrupt(_cx: dma_interrupt::Context) {
        #[cfg(feature = "joysticks")]
        {
            let dma_interrupt::Context { mut shared, local } = _cx;

            let buffer = shared.adc_transfer.lock(|transfer| {
                let (buffer, _ ) = transfer.next_transfer(local.adc_buffer.take().unwrap()).unwrap();
                buffer
            });

            let joy1x_adc = buffer[0];
            let joy1y_adc = buffer[1];
            let joy2x_adc = buffer[2];
            let joy2y_adc = buffer[3];
            *local.adc_buffer = Some(buffer);

            defmt::info!("J1X {}, J1Y {}, J2X {}, J2Y {}", joy1x_adc, joy1y_adc, joy2x_adc, joy2y_adc);
            clear_dma_interrupt::spawn_after(fugit::ExtU32::millis(ADC_POLL_PERIOD_MS)).unwrap();
        }
    }

    #[task(binds = OTG_FS, shared = [usb, command_queue], priority = 5)]
    fn usb_interrupt(mut cx: usb_interrupt::Context) {
        cx.shared.usb.lock(|u| {
            if u.poll() {
                // defmt::debug!("DATA AVAILABLE, {:?}", u.read_command());
            }
        });
    }

    /// Perioidically monitor the action queue and send on at a time
    #[task(
        local = [action_receiver, current_action], 
        shared = [usb], priority = 1
    )]
    fn send_actions_to_pc(mut cx: send_actions_to_pc::Context) {
        let changed = match cx.local.current_action {
            Some(_) => {
                let _ = cx.local.current_action.take();
                true
            }
            None => {
                match cx.local.action_receiver.dequeue() {
                    Some(item) => {
                        *cx.local.current_action = Some(item);
                        true
                    }
                    None => {
                        false
                    }
                }
            }
        };

        if changed {
            let report = cx.local.current_action.unwrap_or_default().clone();

            cx.shared.usb.lock(|usb| {
                match usb.send_report(report) {
                    Ok(_) => {},
                    Err(e) => {
                        defmt::error!("Error sending USB - {:?}", e);
                    }
                }
            });
        }   

        // requeue the action
        send_actions_to_pc::spawn_after(fugit::ExtU32::millis(USB_QUEUE_CONSUMPTION_DELAY_MS)).unwrap();
    }

    #[task(local = [led_pin, led_state], priority = 1)]
    fn blink(_cx: blink::Context) {
        defmt::debug!("BLINK");

        #[cfg(feature = "blink")]
        {
            if *_cx.local.led_state {
                _cx.local.led_pin.off().ok();
            } else {
                _cx.local.led_pin.on().ok();
            }
            
            *_cx.local.led_state = !*_cx.local.led_state;
            blink::spawn_after(fugit::ExtU32::millis(300)).unwrap();
        }
    }
}
