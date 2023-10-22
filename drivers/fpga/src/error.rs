// Error classes for the FPGA driver

#[derive(Debug)]
pub enum FpgaError<SpiError, PinError> {
    // SPI Error
    SPI(SpiError),

    // CS Error
    CSPin(PinError),

    // INIT pin error
    InitPin(PinError),
    
    // PROG pin error
    ProgPin(PinError),
    
    // DONE pin error
    DonePin(PinError),

    /*
        FPGA Timeouts can occur during configuration

        Use the following table to refer to which operation caused the timeout:
        ======================
        |  Code  | Operation |
        ======================
        |   0x1  |   INIT_B  |
        |   0x2  |    DONE   |
        |   0x3  |  Watchdog |
        ======================
        
    */
    FPGATimeout(u8),
}