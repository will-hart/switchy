#![deny(missing_docs)]
#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]

//! SHIFT REGISTER HAL
//!
//! This is a higher level abstraction of a parallel to serial
//! shift register, such as 74HC1645. It allows loading in data
//! by polling, and presenting data when complete. It supports
//! shift regsiter chaining up to 32 bits.
//!
//! Built using [`embedded-hal`] traits
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/0.2

use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin};

#[cfg(test)]
pub mod mock;

/// An implementation of the [ShiftRegister] trait for a 74HC165 shift register
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub struct ShiftRegister<const N: u8, TInputPin, TOutputPin> {
    /// The current value that we are reading in, which may contain incomplete / out of order
    /// values read from the serial output of the shift register.
    current_value: u32,

    /// Used to store the previous value that was sent, enabling change detection.
    previous_value: u32,

    /// The current bit we are readig
    current_bit: u8,

    /// The clock pin for the Serial output
    clock_pin: TOutputPin,

    /// The pin to set to read the parallel input into the shift register
    latch_pin: TOutputPin,

    /// The pin to enable the clock input.
    clock_enable_pin: Option<TOutputPin>,

    /// The pin to read serial input from
    serial_read_pin: TInputPin,
}

impl<const N: u8, TInputPin, TOutputPin> ShiftRegister<N, TInputPin, TOutputPin>
where
    TInputPin: InputPin,
    TOutputPin: OutputPin + StatefulOutputPin,
{
    /// Creates a new instance of a shift register from the given pins. No clock
    /// enable pin is provided
    pub fn new(clock_pin: TOutputPin, latch_pin: TOutputPin, serial_read_pin: TInputPin) -> Self {
        Self::new_with_enable(clock_pin, latch_pin, serial_read_pin, None)
    }

    /// Creates a new instance of a shift register from the given pins
    pub fn new_with_enable(
        clock_pin: TOutputPin,
        latch_pin: TOutputPin,
        serial_read_pin: TInputPin,
        clock_enable_pin: Option<TOutputPin>,
    ) -> Self {
        Self {
            current_value: 0,
            previous_value: 0,
            current_bit: 0,
            clock_pin,
            latch_pin,
            clock_enable_pin,
            serial_read_pin,
        }
    }

    /// Updates the shift register and returns Some(T) if reading is complete.
    /// When data is read, it triggers reloading the shift register for the next
    /// read.
    ///
    /// Should be polled periodically to update the data. The maximum poll frequency
    /// depends on the device, refer to the datasheet.
    pub fn poll(&mut self) -> Option<u32> {
        #[cfg(feature = "logging")]
        defmt::warn!(
            "bit {}, current 0x{:x}, previous 0x{:x}",
            self.current_bit,
            self.current_value,
            self.previous_value
        );

        match (
            self.latch_pin.is_set_low().ok(),
            self.clock_pin.is_set_low().ok(),
        ) {
            (Some(true), _) => {
                // we've just read in the data. Now we need to toggle the "load"
                // pin and get set up for reading from the serial output port
                self.latch_pin.set_high().ok();
                None
            }
            (Some(false), _) if self.current_bit >= N => {
                // We've finished getting all the data from serial, now its time to
                // read in the parallel inputs and start again
                self.reset();

                None
            }
            (Some(false), Some(clock_is_low)) => {
                // we're in the middle bits. Lets toggle the clock
                if clock_is_low {
                    // this LOW->HIGH transition prepares the next serial bit
                    self.clock_pin.set_high().ok();
                    let val = if self.serial_read_pin.is_high().ok().unwrap_or(false) {
                        1u32
                    } else {
                        0u32
                    };

                    // shift val in
                    self.current_value = (self.current_value << 1) | val;

                    // move to the next bit
                    self.current_bit += 1;

                    // check if we've reached the end of the bits
                    if self.current_bit >= N {
                        let result = Some(self.current_value);
                        self.reset();
                        result
                    } else {
                        None // not ready yet
                    }
                } else {
                    // Nothing else to do here as this transition doesn't do anything
                    self.clock_pin.set_low().ok();
                    None
                }
            }
            (load_pin, clock_pin) => {
                #[cfg(feature = "logging")]
                defmt::warn!(
                    "ShiftRegister in unexpected state - load: {} clock: {}",
                    load_pin,
                    clock_pin
                );

                None
            }
        }
    }

    /// Enables the device
    pub fn enable(&mut self) {
        if let Some(ref mut pin) = self.clock_enable_pin {
            pin.set_high().ok();
        }
    }

    /// Disables the device
    pub fn disable(&mut self) {
        if let Some(ref mut pin) = self.clock_enable_pin {
            pin.set_low().ok();
        }
    }

    /// Resets the device, discarding all internal state and reading in a new
    /// parallel input from the shift register.
    pub fn reset(&mut self) {
        self.read_parallel_input();

        self.latch_pin.set_low().ok();
        self.clock_pin.set_high().ok();

        self.previous_value = self.current_value;

        self.current_bit = 0;
        self.current_value = 0;
    }

    /// Gets the changes bits between this and the previous read. When [ShiftRegister::poll]
    /// return a value, then previous will be valid until [ShiftRegister::poll] is called again.
    ///
    /// If called at any other time, [ShiftRegister::changed_bits] will return 0
    pub fn changed_bits(&self) -> u32 {
        if self.current_bit != N {
            return 0;
        }

        self.current_value ^ self.previous_value
    }

    fn read_parallel_input(&mut self) {
        self.latch_pin.set_low().ok();
    }
}
