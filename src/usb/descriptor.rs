//! Contains descriptors and report types for the Switchy USB HID device

use serde::ser::{Serialize, SerializeTuple, Serializer};
use usbd_hid::descriptor::{gen_hid_descriptor, AsInputReport, SerializedDescriptor};

/// A descriptor and report for a custom keyboard USB HID device
#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = KEYBOARD) = {
        (usage_page = KEYBOARD, usage_min = 0xE0, usage_max = 0xE7) = {
            #[packed_bits 8] #[item_settings data,variable,absolute] modifier=input;
        };
        (usage_min = 0x00, usage_max = 0xFF) = {
            #[item_settings constant,variable,absolute] reserved=input;
        };
        (usage_page = LEDS, usage_min = 0x01, usage_max = 0x05) = {
            #[packed_bits 5] #[item_settings data,variable,absolute] leds=output;
        };
        (usage_page = KEYBOARD, usage_min = 0x00, usage_max = 0xDD) = {
            #[item_settings data,array,absolute] keycodes=input;
        };
    }
)]
#[derive(Default)]
pub struct CustomKeyboardReport {
    /// The modifier keys that are pressed
    pub modifier: u8,
    /// Reserved - do not use
    pub reserved: u8,
    /// The LEDS that are active
    pub leds: u8,
    /// The keycodes that were currently pressed
    pub keycodes: [u8; 6],
}

impl CustomKeyboardReport {
    /// Clears a report, returning the contents to the default values (0s all
    /// around)
    pub fn clear(&mut self) {
        self.modifier = 0;
        self.keycodes = [0; 6];
    }
}

/// A USB HID report descriptor for a command sent from the computer to the
/// Switchy device
#[gen_hid_descriptor(
    (collection = LOGICAL, usage_page = VENDOR_DEFINED_START, usage = 0x00) = {
        (usage_page = 0xFF17, usage_min = 0x01, usage_max = 0xFF) = {
            #[item_settings data,array,absolute] command=output;
        };
    }
)]
pub struct CommandReport {
    /// The command payload that was sent by the computer
    pub command: [u8; 2],
}
