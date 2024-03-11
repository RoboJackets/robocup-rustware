//!
//! Status of the FPGA.
//! 
//! May or may not be used
//! 

#[derive(Clone, Copy)]
pub enum FpgaStatus {
    NotReady, // haven't been successfully init
    Standby, // init was done successfully
}