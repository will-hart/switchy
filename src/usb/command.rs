//! Contains USB commands, which are used by the switchy firmware to pass
//! around messages about USB operations requested by user input.

/// A command is a message received from the PC, it consists of:
/// - a u8 message type
/// - two u8 payloads
#[derive(Copy, Clone, Default)]
pub struct Command {
    /// The ID of the command received from the computer
    pub command_id: u8,

    /// The first byte of payload received from the computer
    pub payload_a: u8,

    /// The second byte of payload received from the computer
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

/// The joystick axis
#[derive(Copy, Clone)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub enum JoystickAxis {
    /// X Axis
    X,
    /// Y Axis
    Y,
}

/// The direction an encoder was turned
#[derive(Copy, Clone)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub enum EncoderDirection {
    /// An up, or clockwise turn of an encoder
    Up,
    /// A down, or counter-clockwise turn of an encoder
    Down,
}

/// New type for a joystick number
#[derive(Copy, Clone, Default)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub struct JoystickNumber(pub u8);

/// New type for a button number
#[derive(Copy, Clone, Default)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub struct ButtonNumber(pub u8);

/// New type for an encoder number
#[derive(Copy, Clone, Default)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub struct EncoderNumber(pub u8);

/// An action performed by a user
#[derive(Clone, Default)]
#[cfg_attr(feature = "logging", derive(defmt::Format))]
pub enum UserAction {
    /// No action was performed
    #[default]
    None,
    /// A button was pushed or released
    Button(ButtonNumber, bool),
    /// An encoder was turned in a given direction
    Encoder(EncoderNumber, EncoderDirection),
    /// An analog joystic reading was made for a given axis
    Joystick(JoystickAxis, JoystickNumber, u16),
}

/// A KeyAction is something done by a user that should be queued
/// up and sent over the HID to the PC. It includes a key and a modifier.
#[derive(Copy, Clone, Default)]
pub struct KeyAction {
    /// The modifers (shift, mod key etc) that are pressed
    pub modifiers: u8,
    /// The code of the key that was pressed
    pub key: u8,
}

impl KeyAction {
    /// Creates a new key action
    pub fn new(modifiers: u8, key: u8) -> Self {
        Self { modifiers, key }
    }
}
