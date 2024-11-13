//!
//! This is an example that tests the interface between a Teensy and Motor Drivers, but is simplified vs motor_driver_uart.rs
//!
//! 



#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_alloc::Heap;

// adding heap stuff
// might be why the teensy panicked
#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPT1])]
mod app {
    use super::*;

    use bsp::board as board;
    use bsp::hal::lpuart as lpuart;
    use teensy4_bsp as bsp;

    // used for easy serial read
    use cortex_m::prelude::_embedded_hal_serial_Read;


    use core::mem::MaybeUninit;

    use rtic_monotonics::systick::*;

    const ARRAY_SIZE : usize = 4;
    const WATERMARK_VAL : u32 = 4;
    const BAUD_RATE : u32 = 9600;
    const MY_BAUD: board::LpuartBaud = board::lpuart_baud(9600);

    const ONE_SECOND_US: u32 = 1000000;
    const WAIT_UART_US: u32 = 1000;


    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        output: [u8; ARRAY_SIZE]
    }

    #[shared]
    struct Shared {
        buffer: [u8; ARRAY_SIZE],
        index: usize,
        lpuart6: board::Lpuart6
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize the Heap
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE); }

        let board::Resources {
            pins,
            lpuart6,
            usb,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let watermark = lpuart::Watermark::rx(core::num::NonZero::<u32>::new(WATERMARK_VAL).unwrap());

        let mut lpuart6 : board::Lpuart6 = board::lpuart(lpuart6, pins.p1, pins.p0, MY_BAUD.value(board::UART_FREQUENCY)); // last parameter is NOT Baud rate
        lpuart6.disable(|lpuart6| {
            lpuart6.disable_fifo(lpuart::Direction::Tx);
            lpuart6.enable_fifo(watermark);                             // enable receive FIFO so we don't miss anything
            lpuart6.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            lpuart6.set_parity(None);
        });

        let buffer = [0; ARRAY_SIZE];
        let output = [0; ARRAY_SIZE];
        let index = 0;

        // Initialize Systick Async Delay
        let systick_token = rtic_monotonics::create_systick_token!();        
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        //transmit_stuff::spawn().ok();
        wait_dly::spawn().ok();

        (Shared {
            buffer: buffer,
            index: index,
            lpuart6 : lpuart6
        }, Local {
            output: output
        })

    }

    #[idle]
    fn idle(_ : idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }



    #[task(priority = 1, shared = [lpuart6])]
    async fn transmit_stuff(ctx: transmit_stuff::Context) {
        let mut lpuart6 = ctx.shared.lpuart6;
        let send_bytes: [u8; ARRAY_SIZE] = [4, 0, 22, 0];
        let send_byte: u8 = b'a';

        log::info!("WAITING");

        
        for i in 0..ARRAY_SIZE {
            lpuart6.lock(|lpuart6| {
                if lpuart6.status().contains(lpuart::Status::TRANSMIT_EMPTY) {
                    lpuart6.write_byte(send_byte);
                    log::info!("SENDING BYTE");
                } else {
                    lpuart6.clear_status(lpuart::Status::TRANSMIT_EMPTY);
                    log::info!("FAILED TO SEND A BYTE");
                }
                });
            Systick::delay(WAIT_UART_US.micros()).await;
        }
        
        transmit_stuff_delay::spawn().ok();
    }

    #[task(priority = 1)]
    async fn transmit_stuff_delay(_ctx: transmit_stuff_delay::Context) {
        Systick::delay(ONE_SECOND_US.micros()).await;

        transmit_stuff::spawn().ok();
    }

    #[task(priority = 1)]
    async fn wait_to_rcv(ctx: wait_to_rcv::Context) {
        log::info!("Waiting to receive data");

        wait_dly::spawn().ok();
    }

    #[task(priority = 1)]
    async fn wait_dly(ctx: wait_dly::Context) {
        Systick::delay(ONE_SECOND_US.micros()).await;

        wait_to_rcv::spawn().ok();
    }

    
    #[task(binds = LPUART6, priority = 3, shared = [buffer, index, lpuart6])]
    fn lpuart6_interrupt(ctx: lpuart6_interrupt::Context) {
        let lpuart6 = ctx.shared.lpuart6;
        let buffer = ctx.shared.buffer;
        let index = ctx.shared.index;

        (buffer, index, lpuart6).lock(|buffer, index, lpuart6| {
            for _ in 0..WATERMARK_VAL {
                buffer[*index] = lpuart6.read().unwrap();
                *index = (*index + 1) % ARRAY_SIZE;
                log::info!("received {}", buffer[*index]);
                if *index == ARRAY_SIZE - 1{
                    uart_read::spawn().ok();
                }
            }
            //log::info!("Not enough data was read through UART, won't be processed.");
        })
    }

    
    #[task(shared = [buffer, index], local = [output], priority = 1)]
    async fn uart_read(ctx: uart_read::Context) {
        let buffer = ctx.shared.buffer;
        let index = ctx.shared.index;
        let output = ctx.local.output;

        (buffer, index).lock(|buffer, index| {
            for i in 0..ARRAY_SIZE {
                // 0 is NULL in UTF-8, this means the transfer was not compelted
                if buffer[i] == 0 {
                    break;
                }

                output[i] = buffer[i]; // copy over values
                buffer[i] = 0; // reset buffer value
            }

            *index = 0;
        });

        for letter in output {
            if *letter == 0 {
                break;
            }
            log::info!("{}", *letter as char)
        }
        
    }
}