//! Mocked pins for testing the ShiftRegister library

use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin};

pub struct MockPin {
    state: bool,
}

impl MockPin {
    pub fn new() -> Self {
        MockPin { state: true }
    }
}

type MockError = &'static str;

impl InputPin for MockPin {
    type Error = MockError;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.state)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.state)
    }
}

impl OutputPin for MockPin {
    type Error = MockError;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        Ok(())
    }
}

impl StatefulOutputPin for MockPin {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.state)
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.state)
    }
}
