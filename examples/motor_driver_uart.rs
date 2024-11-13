//!
//! This is an example that tests the interface between a Teensy microcontroller and the UART on STSPIN motor driver chips
//! This will be used to interface with software in the future
//! 


#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use super::*;

    use core::convert::Infallible;
    use core::f32::MAX;
    use core::mem::MaybeUninit;

    use embedded_hal::blocking::delay::DelayMs;

    use fpga_rs::error::FpgaError;
    use imxrt_hal::lpspi::LpspiError;
    use imxrt_iomuxc::prelude::*;

    use nalgebra::{Vector3, Vector4};
    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::board::PERCLK_FREQUENCY;

    use teensy4_bsp::hal as hal;
    use hal::lpi2c::ControllerStatus;
    use hal::lpuart::Direction;
    use hal::gpio::Trigger;
    use hal::timer::Blocking;
    use hal::pit::Chained01;

    use rtic_monotonics::systick::*;

    use robojackets_robocup_rtp::ControlMessage;

    use motion::MotionControl;

    use fpga_rs as fpga;
    use fpga::FPGA_SPI_FREQUENCY;
    use fpga::FPGA_SPI_MODE;
    use fpga::FPGA;

    use icm42605_driver::{IMU, ImuError};

    use main::{
        Fpga, Imu, PitDelay, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY
    };

    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    //const MOTOR_UART_BAUD : board::LpuartBaud = board::lpuartbaud(9600);




    #[local]
    struct Local {
        gpt2: imxrt_hal::gpt::Gpt2,
        body_velocities: Vector3<f32>,
        next_body_velocities: Vector3<f32>,
        start_time: u32,
        motion_controller: MotionControl,
        last_encoders: Vector4<f32>,
        chain_timer: Chained01,
        uart6_rx_buffer : [u8; 1],
        uart6_tx_buffer: [u8; 1],
        lpuart6 : board::Lpuart6
    }

    #[shared]
    struct Shared {
        control_message: Option<ControlMessage>,
        counter: u32,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        // Grab the board peripherals
        let board::Resources {
            mut pins,
            mut gpio1,
            mut gpio2,
            mut gpio3,
            mut gpio4,
            usb,
            lpi2c1,
            lpspi4,
            mut gpt1,
            mut gpt2,
            pit: (pit0, pit1, pit2, _pit3),
            mut lpuart6,
            ..
        } = board::t41(ctx.device);


        // Setup USB Logging
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Gpt 1 as blocking delay
        gpt1.disable();
        gpt1.set_divider(GPT_DIVIDER);
        gpt1.set_clock_source(GPT_CLOCK_SOURCE);
        let delay1 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt1);

        //gpt2.disable();
        //gpt2.set_divider(GPT_DIVIDER);
        //gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        //gpt2.enable();

        // Chained Pit<0> and Pit<1>
        let mut chained_timer = Chained01::new(pit0, pit1);
        chained_timer.enable();

        // Setup Rx Interrupt
        let rx_int = gpio1.input(pins.p15);
        gpio1.set_interrupt(&rx_int, Some(Trigger::FallingEdge));

        // Initialize Fpga SPI
        let mut spi = board::lpspi(
            lpspi4,
            board::LpspiPins {
                pcs0: pins.p10,
                sck: pins.p13,
                sdo: pins.p11,
                sdi: pins.p12,
            },
            FPGA_SPI_FREQUENCY,
        );
        spi.disabled(|spi| spi.set_mode(FPGA_SPI_MODE));

        rx_int.clear_triggered();

        let mut lpuart6 = board::lpuart(lpuart6, pins.p1, pins.p0, 9600);

        // enable the tx/rx of the uart ports
        lpuart6.set_enable(Direction::Tx, true);
        lpuart6.set_enable(Direction::Rx, false);

        test_uart::spawn().ok();

        (
            Shared {
                control_message: None,
                counter: 0,
            },
            
            Local {
                gpt2: gpt2,
                body_velocities: Vector3::new(0.0, 1.0, 0.0),
                next_body_velocities: Vector3::new(0.0, 0.0, 0.0),
                start_time: 0,
                motion_controller: MotionControl::new(),
                last_encoders: Vector4::zeros(),
                chain_timer: chained_timer,
                uart6_rx_buffer : [0],
                uart6_tx_buffer : [0],
                lpuart6 : lpuart6
            }
        )

    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }


    #[task (
        local = [uart6_rx_buffer, uart6_tx_buffer, lpuart6, count : u64 = 0],
        priority = 1
    )]
    async fn test_uart(mut ctx: test_uart::Context) {
        ctx.local.lpuart6.set_enable(Direction::Tx, true);
        ctx.local.lpuart6.set_enable(Direction::Tx, true);

        let transmit_fifo_en = ctx.local.lpuart6.is_fifo_enabled(Direction::Tx);
        let receive_fifo_en = ctx.local.lpuart6.is_fifo_enabled(Direction::Rx);
        log::info!("Transmit fifo is enabled: {transmit_fifo_en}\n Receive fifo is enabled: {receive_fifo_en}\n");
        *ctx.local.count += 1;
        //if (*ctx.local.count < 1000) {
            test_uart::spawn().ok();
        //}
        //ctx.local.lpuart6.dma_read()
    }
}