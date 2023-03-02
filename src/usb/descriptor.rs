use serde::ser::{Serialize, SerializeTuple, Serializer};
use usbd_hid::descriptor::{gen_hid_descriptor, AsInputReport, SerializedDescriptor};

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
pub struct CustomKeyboardReport {
    pub modifier: u8,
    pub reserved: u8,
    pub leds: u8,
    pub keycodes: [u8; 6],
}

#[gen_hid_descriptor(
    (collection = LOGICAL, usage_page = VENDOR_DEFINED_START, usage = 0x00) = {
        (usage_page = 0xFF17, usage_min = 0x01, usage_max = 0xFF) = {
            #[item_settings data,array,absolute] command=output;
        };
    }
)]
pub struct CommandReport {
    pub command: [u8; 2],
}
