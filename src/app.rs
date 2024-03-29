//! The main app module for the switchy firmware

use stm32f4xx_hal as hal;

use heapless::{
    mpmc::Q32,
    spsc::{Consumer, Producer, Queue},
};
use switch_hal::{ActiveHigh, Switch};

#[cfg(feature = "blinky")]
use switch_hal::OutputSwitch;

use hal::{
    adc::Adc,
    dma::{PeripheralToMemory, StreamX, Transfer},
    gpio::{ErasedPin, Input, Output},
    pac::{ADC1, DMA2},
    timer::{fugit, MonoTimerUs},
};
use usb_device::class_prelude::UsbBusAllocator;

#[cfg(feature = "joysticks")]
use crate::usb::command::{JoystickAxis, JoystickNumber};
#[cfg(feature = "joysticks")]
use hal::pac::Interrupt;

#[cfg(feature = "encoders")]
use crate::usb::command::{EncoderDirection, EncoderNumber};
#[cfg(feature = "encoders")]
use rotary_encoder_hal::{Direction, Rotary};

use crate::{
    configure,
    debounced::DebouncedInput,
    key_mapping::KEY_MAPPING,
    usb::{
        command::{ButtonNumber, Command, UserAction},
        descriptor::CustomKeyboardReport,
        interface::UsbInterface,
    },
};

use shift_register_hal::ShiftRegister;

/// The period between changing the USB HID keyboard report in ms
pub const USB_QUEUE_CONSUMPTION_DELAY_MS: u32 = 10;

/// The period for querying the button-like inputs (encoders, buttons) in us.
/// Note that for the buttons on the shift register, this equates to sampling
/// the buttons approx every 1700us (as there are 16 inputs, each input requires
/// two clock pulses to read, and then the inputs must be read in with the latch
/// pin, which takes two more cycles). Therefore to prevent bouncing if we
/// sample 15 times using Ganssle's simple debouncing method, we should expect
/// about 25.5ms of debounce (15*1700us).
pub const INPUT_POLL_PERIOD_US: u32 = 50;

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

    /// Sends an action using the given action sender
    macro_rules! send_action {
        ($context: expr, $action: expr) => {
            $context.shared.action_sender.lock(|sender| {
                let _ = sender.enqueue($action);
            })
        };
    }

    // Shared resources go here
    #[shared]
    struct Shared {
        /// Used to queue up commands received from the PC for processing
        command_queue: Q32<Command>,

        /// Used to queue actions from the switchy through the USB HID to the PC
        action_sender: Producer<'static, UserAction, 32>,

        /// The USB interface used
        usb: UsbInterface<'static>,

        /// The ADC transfer used to read ADC data from DMA
        adc_transfer:
            Transfer<StreamX<DMA2, 0>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut [u16; 4]>,
    }

    // Local resources go here
    #[local]
    struct Local {
        /// Used by the report sending task to read actions that should be queued
        action_receiver: Consumer<'static, UserAction, 32>,

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
        action_queue: Queue<UserAction, 32> = Queue::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        #[cfg(feature = "logging")]
        defmt::info!("[INIT] entering");

        // configure all the clocks and peripherals
        let config = configure::configure(cx.core, cx.device, cx.local.USB_BUS, cx.local.USB_MEM);

        // configure the message passing queues
        let (action_sender, action_receiver) = cx.local.action_queue.split();

        // forward actions from the queue
        send_keyboard_actions_to_pc::spawn_after(fugit::ExtU32::millis(
            USB_QUEUE_CONSUMPTION_DELAY_MS,
        ))
        .unwrap();

        // show a blinky light
        #[cfg(feature = "blink")]
        {
            blink::spawn_after(fugit::ExtU32::secs(1u32)).unwrap();
            defmt::info!("[INIT] spawned blink");
        }

        // scan inputs
        #[cfg(feature = "buttons")]
        {
            poll_registers::spawn_after(fugit::ExtU32::micros(INPUT_POLL_PERIOD_US)).unwrap();

            #[cfg(feature = "logging")]
            defmt::info!("[INIT] spawned button shift register polling");
        }

        // scan inputs
        #[cfg(feature = "encoders")]
        {
            poll_encoders::spawn_after(fugit::ExtU32::micros(INPUT_POLL_PERIOD_US)).unwrap();
            defmt::info!("[INIT] spawned encoder polling");
        }

        #[cfg(feature = "joysticks")]
        {
            poll_adcs::spawn_after(fugit::ExtU32::millis(ADC_POLL_PERIOD_MS)).unwrap();
            defmt::info!("[INIT] spawned joystick polling via ADCs");
        }

        #[cfg(feature = "logging")]
        defmt::info!("[INIT] exiting");

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

    #[task(
        local = [
            bank1,
            bank2,
            debounced: [DebouncedInput::<15>; 32] = [DebouncedInput::<15>::new(); 32]
        ],
        shared = [action_sender],
        priority = 2
    )]
    fn poll_registers(mut _cx: poll_registers::Context) {
        #[cfg(feature = "buttons")]
        {
            let bank1 = _cx.local.bank1;
            let bank2 = _cx.local.bank2;

            if let Some(value) = bank1.poll() {
                let result = _cx.local.debounced[value.bit as usize].debounce(value.is_high);

                if result.is_changed {
                    #[cfg(feature = "logging")]
                    defmt::info!(
                        "Received changed bank1 bit {} {}, is now {}. Value: 0b{:032b}",
                        value.bit,
                        if value.is_changed {
                            "changed"
                        } else {
                            "unchanged"
                        },
                        if result.is_on { "high" } else { "low" },
                        (bank1.get_value() << 16) | bank2.get_value()
                    );

                    _cx.shared.action_sender.lock(|sender| {
                        sender
                            .enqueue(UserAction::Button(ButtonNumber(value.bit), result.is_on))
                            .ok();
                    });
                }
            }

            if let Some(value) = bank2.poll() {
                let result = _cx.local.debounced[value.bit as usize + 16].debounce(value.is_high);

                if result.is_changed {
                    #[cfg(feature = "logging")]
                    defmt::info!(
                        "Received changed bank2 bit {} {}, is now {}. Value: 0b{:032b}",
                        value.bit,
                        if result.is_changed {
                            "changed"
                        } else {
                            "unchanged"
                        },
                        if result.is_on { "high" } else { "low" },
                        (bank1.get_value() << 16) | bank2.get_value()
                    );

                    _cx.shared.action_sender.lock(|sender| {
                        sender
                            .enqueue(UserAction::Button(
                                ButtonNumber(value.bit + 16),
                                result.is_on,
                            ))
                            .ok();
                    });
                }
            }

            poll_registers::spawn_after(fugit::ExtU32::micros(INPUT_POLL_PERIOD_US)).unwrap();
        }
    }

    #[task(local = [encoder1, encoder2, encoder3, encoder4], shared = [action_sender], priority = 2)]
    fn poll_encoders(mut _cx: poll_encoders::Context) {
        #[cfg(any(feature = "encoders"))]
        {
            match _cx.local.encoder1.update() {
                Ok(dir) => {
                    match dir {
                        Direction::Clockwise => send_action!(
                            _cx,
                            UserAction::Encoder(EncoderNumber(1), EncoderDirection::Up)
                        ),
                        Direction::CounterClockwise => send_action!(
                            _cx,
                            UserAction::Encoder(EncoderNumber(1), EncoderDirection::Down)
                        ),
                        Direction::None => {}
                    };
                }
                Err(_) => {
                    #[cfg(feature = "logging")]
                    defmt::error!("Received an error from encoder 1");
                }
            }

            match _cx.local.encoder2.update() {
                Ok(dir) => {
                    #[cfg(feature = "logging")]
                    match dir {
                        Direction::Clockwise => send_action!(
                            _cx,
                            UserAction::Encoder(EncoderNumber(2), EncoderDirection::Up)
                        ),
                        Direction::CounterClockwise => send_action!(
                            _cx,
                            UserAction::Encoder(EncoderNumber(2), EncoderDirection::Down)
                        ),
                        Direction::None => {}
                    };
                }
                Err(_) => {
                    #[cfg(feature = "logging")]
                    defmt::error!("Received an error from encoder 2");
                }
            }

            match _cx.local.encoder3.update() {
                Ok(dir) => {
                    #[cfg(feature = "logging")]
                    match dir {
                        Direction::Clockwise => send_action!(
                            _cx,
                            UserAction::Encoder(EncoderNumber(3), EncoderDirection::Up)
                        ),
                        Direction::CounterClockwise => send_action!(
                            _cx,
                            UserAction::Encoder(EncoderNumber(3), EncoderDirection::Down)
                        ),
                        Direction::None => {}
                    };
                }
                Err(_) => {
                    #[cfg(feature = "logging")]
                    defmt::error!("Received an error from encoder 3");
                }
            }

            match _cx.local.encoder4.update() {
                Ok(dir) => {
                    #[cfg(feature = "logging")]
                    match dir {
                        Direction::Clockwise => send_action!(
                            _cx,
                            UserAction::Encoder(EncoderNumber(4), EncoderDirection::Up)
                        ),
                        Direction::CounterClockwise => send_action!(
                            _cx,
                            UserAction::Encoder(EncoderNumber(4), EncoderDirection::Down)
                        ),
                        Direction::None => {}
                    };
                }
                Err(_) => {
                    #[cfg(feature = "logging")]
                    defmt::error!("Received an error from encoder 4");
                }
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

    #[task(binds = DMA2_STREAM0, shared = [adc_transfer, action_sender], local = [adc_buffer], priority = 3)]
    fn dma_interrupt(mut _cx: dma_interrupt::Context) {
        #[cfg(feature = "joysticks")]
        {
            let dma_interrupt::Context { mut shared, local } = _cx;

            let buffer = shared.adc_transfer.lock(|transfer| {
                let (buffer, _) = transfer
                    .next_transfer(local.adc_buffer.take().unwrap())
                    .unwrap();
                buffer
            });

            #[cfg(feature = "logging")]
            defmt::info!(
                "J1X {}, J1Y {}, J2X {}, J2Y {}",
                buffer[0],
                buffer[1],
                buffer[2],
                buffer[3]
            );

            shared.action_sender.lock(|sender| {
                let _ = sender.enqueue(UserAction::Joystick(
                    JoystickAxis::X,
                    JoystickNumber(1),
                    buffer[0],
                ));
                let _ = sender.enqueue(UserAction::Joystick(
                    JoystickAxis::Y,
                    JoystickNumber(1),
                    buffer[1],
                ));
                let _ = sender.enqueue(UserAction::Joystick(
                    JoystickAxis::X,
                    JoystickNumber(2),
                    buffer[2],
                ));
                let _ = sender.enqueue(UserAction::Joystick(
                    JoystickAxis::Y,
                    JoystickNumber(2),
                    buffer[3],
                ));
            });

            *local.adc_buffer = Some(buffer);
            clear_dma_interrupt::spawn_after(fugit::ExtU32::millis(ADC_POLL_PERIOD_MS)).unwrap();
        }
    }

    #[task(binds = OTG_FS, shared = [usb, command_queue], priority = 5)]
    fn usb_interrupt(mut cx: usb_interrupt::Context) {
        cx.shared.usb.lock(|u| {
            if u.poll() {
                match u.read_command() {
                    Ok(Some(cmd)) => {
                        #[cfg(feature = "logging")]
                        defmt::warn!("Received command {:?}", cmd.as_bytes());

                        cx.shared.command_queue.lock(|q| match q.enqueue(cmd) {
                            Ok(_) => {}
                            Err(_) => {
                                #[cfg(feature = "logging")]
                                defmt::error!(
                                    "Error queuing usb command, queue is full. Dropped command is: {:?}",
                                    cmd.as_bytes(),
                                );
                            }
                        });
                    }
                    Ok(None) => {}
                    Err(e) => {
                        #[cfg(feature = "logging")]
                        defmt::error!("Received USB command read error: {:?}", e);
                    }
                }
            }
        });
    }

    /// Perioidically monitor the action queue and send on at a time
    #[task(
        local = [
            action_receiver,
            keyboard_report: CustomKeyboardReport = CustomKeyboardReport { modifier: 0, reserved: 0, leds: 0, keycodes: [0; 6] },
            current_action: Option<UserAction> = None,
        ],
        shared = [usb],
        priority = 1,
    )]
    fn send_keyboard_actions_to_pc(mut cx: send_keyboard_actions_to_pc::Context) {
        // process one action at a time. If we currently have an action, we
        // should clear it and send a report otherwise we should apply the key
        // associated with this action. For the keyboard we ignore joystick
        // actions but all other actions should be mapped to a specific key
        // combination
        let should_report = if cx.local.current_action.take().is_some() {
            // we have a current action, we should clear it
            cx.local.keyboard_report.clear();

            true
        } else if let Some(action) = cx.local.action_receiver.dequeue() {
            match action {
                UserAction::Button(ButtonNumber(number), is_pressed) if is_pressed => {
                    if let Some(keys) = KEY_MAPPING[number as usize] {
                        cx.local.keyboard_report.keycodes = [keys.key, 0, 0, 0, 0, 0];
                        cx.local.keyboard_report.modifier = keys.modifiers;

                        *cx.local.current_action = Some(action.clone());

                        #[cfg(feature = "logging")]
                        defmt::info!("Applied keyboard code {:?}", action);

                        true
                    } else {
                        // no mapping, just ignore

                        #[cfg(feature = "logging")]
                        defmt::info!("No mapping for keyboard code {:?}, ignoring", action);

                        false
                    }
                }
                UserAction::Button(_, _) => {
                    // when a button is released, clear the current report
                    cx.local.keyboard_report.clear();
                    true
                }
                UserAction::Encoder(_, _) | UserAction::None | UserAction::Joystick(_, _, _) => {
                    // no effect in keyboard mode
                    false
                }
            }
        } else {
            // nothing to report
            false
        };

        if should_report {
            let report = cx.local.keyboard_report;

            // send the report if any actions were received
            cx.shared
                .usb
                .lock(|usb| match usb.send_keyboard_report(*report) {
                    Ok(_) => {}
                    Err(_e) => {
                        #[cfg(feature = "logging")]
                        defmt::error!("Error sending USB - {:?}", _e);
                    }
                });
        }

        // requeue the action
        send_keyboard_actions_to_pc::spawn_after(fugit::ExtU32::millis(
            USB_QUEUE_CONSUMPTION_DELAY_MS,
        ))
        .unwrap();
    }

    #[task(local = [led_pin, led_state], priority = 1)]
    fn blink(_cx: blink::Context) {
        #[cfg(feature = "logging")]
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
