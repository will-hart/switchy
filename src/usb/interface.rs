//! Contains code for configuring and polling a USB HID interface

use stm32f4xx_hal::otg_fs::{UsbBus, USB};

use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    UsbError,
};
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::hid_class::HIDClass;

use crate::usb::descriptor::{CommandReport, CustomKeyboardReport};

use super::command::Command;

const USB_POLL_MS: u8 = 10;

/// A container struct for the USB command and keyboard classes
pub struct UsbInterface<'a> {
    /// The HID keyboard class
    pub hid: HIDClass<'a, UsbBus<USB>>,
    /// The command class for receiving commands from the computer
    command: HIDClass<'a, UsbBus<USB>>,
    /// The USB bus
    pub bus: UsbDevice<'a, UsbBus<USB>>,
}

impl<'a> UsbInterface<'a> {
    /// Creates a new UsbInterface, configures it and returns it
    pub fn new(alloc: &'a UsbBusAllocator<UsbBus<USB>>) -> UsbInterface<'a> {
        // Create the pedal peripheral
        let hid = HIDClass::new(alloc, CustomKeyboardReport::desc(), USB_POLL_MS);
        let command = HIDClass::new_ep_out(alloc, CommandReport::desc(), USB_POLL_MS);

        // TODO: this is a test code from pid.codes, change before release
        let bus = UsbDeviceBuilder::new(alloc, UsbVidPid(0x1209, 0x0001))
            .manufacturer("AttoZepto")
            .product("Switchy")
            .serial_number("000001")
            .device_release(0x0010)
            .build();

        UsbInterface { hid, command, bus }
    }

    /// Polls the USB device
    pub fn poll(&mut self) -> bool {
        self.bus.poll(&mut [&mut self.hid, &mut self.command])
    }

    /// Reads received data from the USB device
    pub fn read_command(&mut self) -> Result<Option<Command>, UsbError> {
        let mut buffer: [u8; 64] = [0; 64];
        match self.command.pull_raw_output(&mut buffer) {
            Ok(size) => {
                // TODO: cache partial buffer
                if size != 3 {
                    #[cfg(feature = "logging")]
                    defmt::warn!("Incomplete buffer, received {:?}", buffer);
                    Ok(None)
                } else {
                    let cmd = Command::new(buffer[0], buffer[1], buffer[2]);
                    Ok(Some(cmd))
                }
            }
            Err(UsbError::WouldBlock) => {
                // no pending data
                Ok(None)
            }
            Err(err) => panic!("Error receiving data {:?}", err),
        }
    }

    /// Sends the keyboard report, if one is ready to go.
    pub fn send_keyboard_report(
        &mut self,
        report: CustomKeyboardReport,
    ) -> Result<bool, usb_device::UsbError> {
        match self.hid.push_input(&report) {
            Ok(_) => Ok(true),
            Err(e) => {
                defmt::println!("Error printing report {:?}", e);
                Ok(false)
            }
        }
    }
}
