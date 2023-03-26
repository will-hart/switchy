//! Contains code for firmware debouncing of inputs using 12 consecutive checks.
//! The value `1` is considered "on".

/// A debounced input that checks `N` times whether a switch is "on" before
/// returning that it is "on".
#[derive(Default, Clone, Copy)]
pub struct DebouncedInput<const N: u8>(u16);

/// Returned by the [DebouncedInput::debounce] function, to denote whether the
/// input has changed and whether the input is currently on.
pub struct DebounceResult {
    /// Has the value changed in the most recent update?
    pub is_changed: bool,
    /// Is the value currently "on"? Note that "on" may be high or low depending
    /// on the hardware configuration)
    pub is_on: bool,
}

impl<const N: u8> DebouncedInput<N> {
    /// Returns a new instance of self, panicing if N >= 16
    pub const fn new() -> Self {
        if N >= 16 {
            panic!("Attempted to debounce an input with N >= 16");
        }

        Self(0xe000)
    }

    /// Debounces the given input taking the current value and returning `true`
    /// if the input is on after debouncing. Note that "on" may be high or low
    /// in hardware, but a boolean should be passed here which is `true` if the
    /// input is currently on
    pub fn debounce(&mut self, is_on: bool) -> DebounceResult {
        let previous_value = self.0 == 0xf000;

        self.0 = (self.0 << 1) | if is_on { 0 } else { 1 } | 0xe000;
        let is_on = self.0 == 0xf000;

        DebounceResult {
            is_changed: is_on != previous_value,
            is_on,
        }
    }
}
