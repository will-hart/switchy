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
use switch_hal::{ActiveHigh, OutputSwitch, Switch};
use usb_device::class_prelude::UsbBusAllocator;
use hal::{
    adc::Adc,
    dma::{PeripheralToMemory, StreamX,  Transfer},
    gpio::{ErasedPin, Input, Output},
    pac::{ADC1, DMA2},
    timer::{fugit, MonoTimerUs}
};
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
pub const ADC_POLL_PERIOD_MS: u32 = 15;

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
    struct Shared {
        /// Used to queue up commands received from the PC for processing
        command_queue: Q32<Command>,

        /// The USB interface used
        usb: UsbInterface<'static>,

        /// The ADC transfer used to read ADC data from DMA
        adc_transfer: Transfer<StreamX<DMA2, 0>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut[u16;4]>,
    }

    // Local resources go here
    #[local]
    struct Local {
        /// Used to queue actions from the switchy through the USB HID to the PC
        action_sender: Producer<'static, KeyAction, 32>,
        /// Used by the report sending task to read actions that should be queued
        action_receiver: Consumer<'static, KeyAction, 32>,
        /// The current action that was just sent, or None if no action was recently sent
        current_action: Option<KeyAction>,

        /// The LED pin for blinky lights
        #[cfg(any(feature = "dev_board", feature = "board_rev_3"))]
        led_pin: Switch<ErasedPin<Output>, ActiveHigh>,
        /// The current state of the blinky light
        #[cfg(any(feature = "dev_board", feature = "board_rev_3"))]
        led_state: bool,

        /// The shift registers
        bank1: ShiftRegister<16, ErasedPin<Input>, ErasedPin<Output>>,
        bank2: ShiftRegister<16, ErasedPin<Input>, ErasedPin<Output>>,

        // The encoders
        encoder1: Rotary<ErasedPin<Input>, ErasedPin<Input>>,
        encoder2: Rotary<ErasedPin<Input>, ErasedPin<Input>>,
        encoder3: Rotary<ErasedPin<Input>, ErasedPin<Input>>,
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

        // configure the message passing queues
        let (action_sender, action_receiver) = cx.local.action_queue.split();

        // show a blinky light
        #[cfg(any(feature = "dev_board", feature = "board_rev_3"))]
        blink::spawn_after(fugit::ExtU32::secs(1u32)).unwrap();

        // forward actions from the queue
        send_actions_to_pc::spawn_after(fugit::ExtU32::millis(USB_QUEUE_CONSUMPTION_DELAY_MS)).unwrap();

        // scan inputs
        #[cfg(any(feature = "dev_board", feature = "board_rev_3", feature = "board_rev_2"))]
        poll_registers::spawn_after(fugit::ExtU32::micros(INPUT_POLL_PERIOD_US)).unwrap();

        #[cfg(feature = "joysticks")]
        poll_adcs::spawn_after(fugit::ExtU32::millis(ADC_POLL_PERIOD_MS)).unwrap();

        (
            Shared {
                command_queue: Q32::new(),
                usb: config.usb,
                adc_transfer: config.adc_transfer
            },
            Local {
                #[cfg(any(feature = "dev_board", feature = "board_rev_3"))]
                led_pin: config.led_pin,
                #[cfg(any(feature = "dev_board", feature = "board_rev_3"))]
                led_state: false,

                action_sender,
                action_receiver,
                current_action: None,

                bank1: config.bank1,
                bank2: config.bank2,

                encoder1: config.encoder1,
                encoder2: config.encoder2,
                encoder3: config.encoder3,
                encoder4: config.encoder4,

                #[cfg(feature = "joysticks")]
                adc_buffer: Some(cx.local.adc_buffer_raw),
            },
            init::Monotonics(config.timer),
        )
    }
    
    #[task(local = [action_sender, bank1, bank2, encoder1, encoder2, encoder3, encoder4])]
    fn poll_registers(cx: poll_registers::Context) {
        defmt::info!("POLLING");

        let bank1 = cx.local.bank1;
        if let Some(value) = bank1.poll() {
            defmt::info!("Received bank1 value {}", value);
        }

        let bank2 = cx.local.bank2;
        if let Some(value) = bank2.poll() {
            defmt::warn!("Received bank2 value {}", value);
        }

        poll_registers::spawn_after(fugit::ExtU32::micros(INPUT_POLL_PERIOD_US)).unwrap();
    }

    #[cfg(feature = "joysticks")]
    #[task(shared = [adc_transfer])]
    fn poll_adcs(mut cx: poll_adcs::Context) {
        cx.shared.adc_transfer.lock(|transfer| {
            transfer.start(|adc| {
                adc.start_conversion();
            })
        });

        poll_adcs::spawn_after(fugit::ExtU32::millis(ADC_POLL_PERIOD_MS)).unwrap();
    }

    
    #[task(binds = OTG_FS, shared = [usb, command_queue])]
    fn usb_interrupt(mut cx: usb_interrupt::Context) {
        cx.shared.usb.lock(|u| {
            if u.poll() {
                defmt::warn!("DATA AVAILABLE");
            }
        });
    }

    /// Perioidically monitor the action queue and send on at a time
    #[task(
        local = [action_receiver, current_action], 
        shared = [usb]
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

    #[cfg(feature = "joysticks")]
    #[task(binds = DMA1_STREAM0, shared = [adc_transfer], local = [adc_buffer])]
    fn dma_interrupt(cx: dma_interrupt::Context) {
        let dma_interrupt::Context { mut shared, local } = cx;

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
    }

    #[cfg(any(feature = "dev_board", feature = "board_rev_3"))]
    #[task(local = [led_pin, led_state])]
    fn blink(cx: blink::Context) {
        defmt::warn!("BLINK");

        if *cx.local.led_state {
            cx.local.led_pin.off().ok();
            *cx.local.led_state = false;
            blink::spawn_after(fugit::ExtU32::millis(300)).unwrap();
        } else {
            cx.local.led_pin.on().ok();
            *cx.local.led_state = true;
            blink::spawn_after(fugit::ExtU32::millis(300)).unwrap();
        }
    }
}
