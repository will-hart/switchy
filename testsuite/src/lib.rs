#![no_std]
#![cfg_attr(test, no_main)]

use switchy_rtic as _; // memory layout + panic handler

#[defmt_test::tests]
mod tests {}
