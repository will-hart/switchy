use stm32f4xx_hal::otg_fs::{UsbBus, USB};

use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    UsbError,
};
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::hid_class::HIDClass;

use crate::usb::descriptor::{CommandReport, CustomKeyboardReport};

use super::command::KeyAction;

const USB_POLL_MS: u8 = 10;

pub struct UsbInterface<'a> {
    pub hid: HIDClass<'a, UsbBus<USB>>,
    command: HIDClass<'a, UsbBus<USB>>,
    pub bus: UsbDevice<'a, UsbBus<USB>>,
    report: CustomKeyboardReport,
}

impl<'a> UsbInterface<'a> {
    /// Creates a new UsbInterface, configures it and returns it
    pub fn new(alloc: &'a UsbBusAllocator<UsbBus<USB>>) -> UsbInterface<'a> {
        // Create the pedal peripheral
        let hid = HIDClass::new(&alloc, CustomKeyboardReport::desc(), USB_POLL_MS);
        let command = HIDClass::new_ep_out(&alloc, CommandReport::desc(), USB_POLL_MS);

        // TODO: this is a test code from pid.codes, change before release
        let bus = UsbDeviceBuilder::new(&alloc, UsbVidPid(0x1209, 0x0001))
            .manufacturer("Atto Zepto")
            .product("Switchy")
            .serial_number("000001")
            .device_release(0x0020)
            // .device_class(0x03)
            .build();

        UsbInterface {
            hid,
            command,
            bus,
            report: CustomKeyboardReport {
                modifier: 0,
                reserved: 0,
                leds: 0,
                keycodes: [0, 0, 0, 0, 0, 0],
            },
        }
    }

    /// Polls the USB device
    pub fn poll(&mut self) -> bool {
        self.bus.poll(&mut [&mut self.hid, &mut self.command])
    }

    /// Reads received data from the USB device
    pub fn read_command(&mut self) -> Result<([u8; 64], usize), UsbError> {
        let mut buffer: [u8; 64] = [0; 64];
        match self.command.pull_raw_output(&mut buffer) {
            Ok(size) => Ok((buffer, size)),
            Err(UsbError::WouldBlock) => {
                // no pending data
                Ok((buffer, 0))
            }
            Err(err) => panic!("Error receiving data {:?}", err),
        }
    }

    /// Sends the report, if one is ready to go.
    pub fn send_report(&mut self, action: KeyAction) -> Result<bool, usb_device::UsbError> {
        self.report.modifier = action.modifiers;
        self.report.keycodes[0] = action.key;

        match self.hid.push_input(&self.report) {
            Ok(_) => Ok(true),
            Err(e) => {
                defmt::println!("Error printing report {:?}", e);
                Ok(false)
            }
        }
    }
}
