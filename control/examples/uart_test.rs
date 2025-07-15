//!
//! Test for sending data to one of the motors via uart
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
    use core::mem::MaybeUninit;

    use imxrt_hal::{lpuart, timer::Blocking};
    use rtic_monotonics::systick::*;
    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };
    use teensy4_bsp::board;

    use robojackets_robocup_control::{
        motors::{motor_interrupt, send_command}, peripherals::*, GPT_CLOCK_SOURCE, GPT_DIVIDER, GPT_FREQUENCY
    };

    use embedded_hal::blocking::delay::DelayMs;

    const HEAP_SIZE: usize = 4096;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    #[local]
    struct Local {
        // Motor 1
        motor_one_tx: Sender<'static, [u8; 4], 3>,
        motor_one_rx: Receiver<'static, [u8; 4], 3>,

        // Motor 2
        motor_two_tx: Sender<'static, [u8; 4], 3>,
        motor_two_rx: Receiver<'static, [u8; 4], 3>,

        // Motor 3
        motor_three_tx: Sender<'static, [u8; 4], 3>,
        motor_three_rx: Receiver<'static, [u8; 4], 3>,

        // Motor 4
        motor_four_tx: Sender<'static, [u8; 4], 3>,
        motor_four_rx: Receiver<'static, [u8; 4], 3>,

        // Dribbler
        dribbler_tx: Sender<'static, [u8; 4], 3>,
        dribbler_rx: Receiver<'static, [u8; 4], 3>,

        // The logging poller
        poller: imxrt_log::Poller,
    }

    #[shared]
    struct Shared {
        // Motor 1
        motor_one_uart: MotorOneUart,
        motor_one_velocity: i32,

        // Motor 2
        motor_two_uart: MotorTwoUart,
        motor_two_velocity: i32,

        // Motor 3
        motor_three_uart: MotorThreeUart,
        motor_three_velocity: i32,

        // Motor 4
        motor_four_uart: MotorFourUart,
        motor_four_velocity: i32,

        // Dribbler
        dribbler_uart: DribblerUart,
        dribbler_velocity: i32,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        #[allow(static_mut_refs)]
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }

        let board::Resources {
            pins,
            mut gpio1,
            mut gpio2,
            mut gpt2,
            lpuart1,
            lpuart4,
            lpuart6,
            lpuart8,
            lpuart7,
            usb,
            ..
        } = board::t41(ctx.device);

        let poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Gpt 2 as blocking delay
        gpt2.disable();
        gpt2.set_divider(GPT_DIVIDER);
        gpt2.set_clock_source(GPT_CLOCK_SOURCE);
        let mut delay2 = Blocking::<_, GPT_FREQUENCY>::from_gpt(gpt2);

        let motor_en: MotorEn = gpio1.output(pins.p23);
        motor_en.set();
        let kill_n: Killn = gpio2.output(pins.p36);
        kill_n.set();

        delay2.delay_ms(500u32);

        let mut motor_one_uart = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);
        motor_one_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_one_uart.clear_status(lpuart::Status::W1C);
        let (motor_one_tx, motor_one_rx) = make_channel!([u8; 4], 3);

        let mut motor_two_uart = board::lpuart(lpuart4, pins.p8, pins.p7, 115200);
        motor_two_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_two_uart.clear_status(lpuart::Status::W1C);
        let (motor_two_tx, motor_two_rx) = make_channel!([u8; 4], 3);

        let mut motor_three_uart = board::lpuart(
            lpuart1,
            pins.p24,
            pins.p25,
            115200,
        );
        motor_three_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_three_uart.clear_status(lpuart::Status::W1C);
        let (motor_three_tx, motor_three_rx) = make_channel!([u8; 4], 3);

        let mut motor_four_uart = board::lpuart(
            lpuart7,
            pins.p29,
            pins.p28,
            115200,
        );
        motor_four_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        motor_four_uart.clear_status(lpuart::Status::W1C);
        let (motor_four_tx, motor_four_rx) = make_channel!([u8; 4], 3);

        let mut dribbler_uart = board::lpuart(lpuart8, pins.p20, pins.p21, 115200);
        dribbler_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.set_parity(None);
        });
        dribbler_uart.clear_status(lpuart::Status::W1C);
        let (dribbler_tx, dribbler_rx) = make_channel!([u8; 4], 3);

        blink::spawn().ok();
        send_motor_move_commands::spawn().ok();

        (
            Shared {
                motor_one_uart,
                motor_one_velocity: 0i32,
                motor_two_uart,
                motor_two_velocity: 0i32,
                motor_three_uart,
                motor_three_velocity: 0i32,
                motor_four_uart,
                motor_four_velocity: 0i32,
                dribbler_uart,
                dribbler_velocity: 0i32,
            },
            Local {
                motor_one_tx,
                motor_one_rx,
                motor_two_tx,
                motor_two_rx,
                motor_three_rx,
                motor_three_tx,
                motor_four_rx,
                motor_four_tx,
                dribbler_tx,
                dribbler_rx,
                poller,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        shared = [
            motor_one_velocity,
            motor_two_velocity,
            motor_three_velocity,
            motor_four_velocity,
            dribbler_velocity,
        ],
        priority = 1
    )]
    async fn blink(mut ctx: blink::Context) {
        loop {
            let motor_one = ctx.shared.motor_one_velocity.lock(|velocity| *velocity);
            let motor_two = ctx.shared.motor_two_velocity.lock(|velocity| *velocity);
            let motor_three = ctx.shared.motor_three_velocity.lock(|velocity| *velocity);
            let motor_four = ctx.shared.motor_four_velocity.lock(|velocity| *velocity);
            let dribbler = ctx.shared.dribbler_velocity.lock(|velocity| *velocity);

            log::info!(
                "Velocity: [{}, {}, {}, {}, {}]",
                motor_one,
                motor_two,
                motor_three,
                motor_four,
                dribbler
            );
            Systick::delay(500u32.millis()).await;
        }
    }

    #[task(
        shared = [
            motor_one_uart,
            motor_two_uart,
            motor_three_uart,
            motor_four_uart,
            dribbler_uart,
        ],
        local = [
            motor_one_tx,
            motor_two_tx,
            motor_three_tx,
            motor_four_tx,
            dribbler_tx,
        ],
        priority = 1
    )]
    async fn send_motor_move_commands(mut ctx: send_motor_move_commands::Context) {
        let mut iteration = 0u32;
        let mut setpoint = 6200; 
        let mut downcounting = true;
        loop {
            if iteration % 5 == 0 {
                setpoint = match setpoint {
                    6200 => {
                        downcounting = true;
                        3100
                    },
                    3100 => if downcounting { 0 } else { 6200 },
                    0 => if downcounting { -3100 } else { 3100 },
                    -3100 => if downcounting { -6200 } else { 0 },
                    -6200 => {
                        downcounting = false;
                        -3100
                    },
                    _ => {
                        downcounting = true;
                        3100
                    },
                }
            }
            ctx.shared
                .motor_one_uart
                .lock(|uart| send_command(setpoint, ctx.local.motor_one_tx, uart, 0));
            ctx.shared
                .motor_two_uart
                .lock(|uart| send_command(setpoint, ctx.local.motor_two_tx, uart, 0));
            ctx.shared
                .motor_three_uart
                .lock(|uart| send_command(setpoint, ctx.local.motor_three_tx, uart, 0));
            ctx.shared
                .motor_four_uart
                .lock(|uart| send_command(setpoint, ctx.local.motor_four_tx, uart, 0));
            ctx.shared
                .dribbler_uart
                .lock(|uart| send_command(setpoint, ctx.local.dribbler_tx, uart, 0));

            iteration = iteration.wrapping_add(1);
            Systick::delay(100u32.millis()).await;
        }
    }

    #[task(
        shared = [
            motor_one_uart,
            motor_one_velocity,
        ],
        local = [
            motor_one_rx,
            idx: usize = 0,
            reading: bool = false,
            buffer: [u8; 4] = [0u8; 4],
        ],
        binds = LPUART6,
    )]
    // Interrupt driven task to actually send and receive data via the uart
    fn motor_one_uart(ctx: motor_one_uart::Context) {
        motor_interrupt(
            ctx.shared.motor_one_uart,
            ctx.shared.motor_one_velocity,
            ctx.local.motor_one_rx,
            ctx.local.idx,
            ctx.local.reading,
            ctx.local.buffer,
        );
    }

    #[task(
        shared = [
            motor_two_uart,
            motor_two_velocity,
        ],
        local = [
            motor_two_rx,
            idx: usize = 0,
            reading: bool = false,
            buffer: [u8; 4] = [0u8; 4],
        ],
        binds = LPUART4
    )]
    fn motor_two_uart(ctx: motor_two_uart::Context) {
        motor_interrupt(
            ctx.shared.motor_two_uart,
            ctx.shared.motor_two_velocity,
            ctx.local.motor_two_rx,
            ctx.local.idx,
            ctx.local.reading,
            ctx.local.buffer,
        );
    }

    #[task(
        shared = [
            motor_three_uart,
            motor_three_velocity,
        ],
        local = [
            motor_three_rx,
            idx: usize = 0,
            reading: bool = false,
            buffer: [u8; 4] = [0u8; 4],
        ],
        binds = LPUART1
    )]
    fn motor_three_uart(ctx: motor_three_uart::Context) {
        motor_interrupt(
            ctx.shared.motor_three_uart,
            ctx.shared.motor_three_velocity,
            ctx.local.motor_three_rx,
            ctx.local.idx,
            ctx.local.reading,
            ctx.local.buffer,
        );
    }

    #[task(
        shared = [
            motor_four_uart,
            motor_four_velocity,
        ],
        local = [
            motor_four_rx,
            idx: usize = 0,
            reading: bool = false,
            buffer: [u8; 4] = [0u8; 4],
        ],
        binds = LPUART7
    )]
    fn motor_four_uart(ctx: motor_four_uart::Context) {
        motor_interrupt(
            ctx.shared.motor_four_uart,
            ctx.shared.motor_four_velocity,
            ctx.local.motor_four_rx,
            ctx.local.idx,
            ctx.local.reading,
            ctx.local.buffer,
        );
    }

    #[task(
        shared = [
            dribbler_uart,
            dribbler_velocity,
        ],
        local = [
            dribbler_rx,
            idx: usize = 0,
            reading: bool = false,
            buffer: [u8; 4] = [0u8; 4],
        ],
        binds = LPUART8
    )]
    fn dribbler_uart(ctx: dribbler_uart::Context) {
        motor_interrupt(
            ctx.shared.dribbler_uart,
            ctx.shared.dribbler_velocity,
            ctx.local.dribbler_rx,
            ctx.local.idx,
            ctx.local.reading,
            ctx.local.buffer,
        );
    }

    /// This task runs when the USB1 interrupt activates.
    /// Simply poll the logger to control the logging process.
    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
