/// A command is a message received from the PC, it consists of:
/// - a u8 message type
/// - two u8 payloads
#[derive(Copy, Clone, Default)]
pub struct Command {
    pub command_id: u8,
    pub payload_a: u8,
    pub payload_b: u8,
}

impl Command {
    /// Creates a new command from the given values
    pub fn new(command_id: u8, payload_a: u8, payload_b: u8) -> Self {
        Self {
            command_id,
            payload_a,
            payload_b,
        }
    }
}

impl From<[u8; 3]> for Command {
    fn from(value: [u8; 3]) -> Self {
        Self {
            command_id: value[0],
            payload_a: value[1],
            payload_b: value[2],
        }
    }
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub enum JoystickAxis {
    X,
    Y,
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub enum EncoderDirection {
    Up,
    Down,
}

#[derive(Copy, Clone, Default)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub struct JoystickNumber(pub u8);

#[derive(Copy, Clone, Default)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub struct ButtonNumber(pub u8);

#[derive(Copy, Clone, Default)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub struct EncoderNumber(pub u8);

#[derive(Clone, Default)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub enum UserAction {
    #[default]
    None,
    Button(ButtonNumber, bool),
    Encoder(EncoderNumber, EncoderDirection),
    Joystick(JoystickAxis, JoystickNumber, u16),
}

/// A KeyAction is something done by a user that should be queued
/// up and sent over the HID to the PC. It includes a key and a modifier.
#[derive(Copy, Clone, Default)]
pub struct KeyAction {
    pub modifiers: u8,
    pub key: u8,
}

impl KeyAction {
    pub fn new(modifiers: u8, key: u8) -> Self {
        Self { modifiers, key }
    }
}
