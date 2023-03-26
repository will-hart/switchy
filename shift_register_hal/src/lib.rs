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

/// The output from a shift register read
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub struct ShiftRegisterOutput {
    /// The bit that was just read
    pub bit: u8,

    /// True if the bit that was just read was high
    pub is_high: bool,

    /// True if the bit that was just read has changed
    pub is_changed: bool,
}

/// An implementation of the [ShiftRegister] trait for a 74HC165 shift register
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub struct ShiftRegister<const N: u8, TInputPin, TOutputPin> {
    /// The current value that we are reading in, which may contain incomplete / out of order
    /// values read from the serial output of the shift register.
    current_value: u32,

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

    /// Tracks whether the register is currently disabled
    disabled: bool,
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
            current_bit: 0,
            clock_pin,
            latch_pin,
            clock_enable_pin,
            serial_read_pin,
            disabled: false,
        }
    }

    /// Enables the device
    pub fn enable(&mut self) {
        if self.disabled == false {
            return;
        }

        if let Some(ref mut pin) = self.clock_enable_pin {
            self.disabled = false;
            pin.set_high().ok();
        }
    }

    /// Disables the device
    pub fn disable(&mut self) {
        if self.disabled {
            return;
        }

        if let Some(ref mut pin) = self.clock_enable_pin {
            self.disabled = true;
            pin.set_low().ok();
        }
    }

    /// Resets the device, discarding all internal state and reading in a new
    /// parallel input from the shift register.
    pub fn reset(&mut self) {
        self.read_parallel_input();
        self.clock_pin.set_high().ok();
        self.current_bit = 0;
    }

    fn read_parallel_input(&mut self) {
        self.latch_pin.set_low().ok();
    }

    fn prepare_for_output(&mut self) {
        self.latch_pin.set_high().ok();
    }

    /// Reads in the current bit and returns either the value of the pin (1 = HIGH, 0 = LOW)
    /// or None if the register wasn't currently in a state to read outputs.
    fn read_bit(&mut self) -> Option<ShiftRegisterOutput> {
        if self.disabled {
            return None;
        }

        match (
            self.latch_pin.is_set_low().ok(),
            self.clock_pin.is_set_low().ok(),
        ) {
            (Some(true), _) => {
                // we've just read in the data. Now we need to toggle the "load"
                // pin and get set up for reading from the serial output port
                self.prepare_for_output();
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
                    let is_high = self.serial_read_pin.is_high().ok().unwrap_or(false);

                    let previous_value = self.current_value;

                    // Set the current bit
                    self.current_value = if is_high {
                        self.current_value | (0x1 << self.current_bit)
                    } else {
                        self.current_value & !(0x1 << self.current_bit)
                    };

                    // work out if this value is the same as last time
                    let is_changed = previous_value != self.current_value;

                    // check if we've reached the end of the bits
                    if self.current_bit >= N {
                        self.reset();
                    }

                    // move to the next bit
                    let bit = self.current_bit;
                    self.current_bit += 1;

                    Some(ShiftRegisterOutput {
                        bit,
                        is_high,
                        is_changed,
                    })
                } else {
                    // Nothing else to do here as this transition doesn't do anything
                    self.clock_pin.set_low().ok();
                    None
                }
            }
            #[cfg(feature = "logging")]
            (load_pin, clock_pin) => {
                defmt::warn!(
                    "ShiftRegister in unexpected state - load: {} clock: {}",
                    load_pin,
                    clock_pin
                );

                None
            }
            #[cfg(not(feature = "logging"))]
            (_, _) => None,
        }
    }

    /// Gets the last read bits from registers
    pub fn get_value(&self) -> u32 {
        self.current_value
    }

    /// Polls the shift register, returning either the bit number that was high, or None if
    /// either the current bit was LOW or the shift register was reading in its inputs.
    pub fn poll(&mut self) -> Option<ShiftRegisterOutput> {
        let val = self.read_bit();

        #[cfg(feature = "logging")]
        {
            if let Some(ref output) = val {
                if output.is_changed {
                    defmt::trace!(
                        "ShiftRegister received: bit {}, high {}, val {:b}",
                        output.bit,
                        output.is_high,
                        self.current_value
                    );
                }
            }
        }

        val
    }
}
