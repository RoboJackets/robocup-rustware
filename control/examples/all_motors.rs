//!
//! This demo moves each of the motors for 1 second individually
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

///
/// This is a demo example file that turns on and off the onboard led.
///
/// Please follow this example for future examples and sanity tests
///
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [GPIO1_INT0, GPIO1_INT1])]
mod app {
    use bsp::board;
    use embedded_hal::{digital::v2::OutputPin, serial::Read};
    use common::{dribbler::{DribblerCommand, DRIBBLER_COMMAND_SIZE, DRIBBLER_RESPONSE_SIZE}, motor::{MotorCommand, MotorMoveResponse, MOTOR_COMMAND_SIZE, MOTOR_MOVE_RESPONSE_SIZE}};
    use cortex_m::prelude::_embedded_hal_blocking_serial_Write;
    use imxrt_hal::lpuart;
    use teensy4_bsp::{self as bsp, ral};
    use ncomm_utils::packing::Packable;

    use robojackets_robocup_control::peripherals::*;
    use rtic_monotonics::systick::*;

    const TARGET_SPEED: i32 = 24_000;

    fn get_motor_commands(motor: u8) -> ([[u8; MOTOR_COMMAND_SIZE]; 4], [u8; DRIBBLER_COMMAND_SIZE]) {
        let mut commands = [MotorCommand::default(); 5];
        if motor != 5 {
            commands[motor as usize] = MotorCommand::Move { ticks_per_second: 500 };
        }

        let mut buffers = ([[0u8; MOTOR_COMMAND_SIZE]; 4], [0u8; DRIBBLER_COMMAND_SIZE]);
        for i in 0..4 {
            commands[i].pack(&mut buffers.0[i]).unwrap();
        }

        if motor == 5 {
            let command = DribblerCommand::Move { percent: 50 };
            command.pack(&mut buffers.1).unwrap();
        }
    
        buffers
    }

    #[local]
    struct Local {
        _motor_en: MotorEn,
        _kill_n: Killn,
    }

    #[shared]
    struct Shared {
        motor_speeds: [i32; 4],
        motor_one_uart: MotorOneUart,
        motor_two_uart: MotorTwoUart,
        motor_three_uart: MotorThreeUart,
        motor_four_uart: MotorFourUart,
        dribbler_uart: DribblerUart,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            usb,
            mut gpio1,
            mut gpio2,
            lpuart4,
            lpuart8,
            lpuart6,
            ..
        } = board::t41(ctx.device);

        bsp::LoggingFrontend::default_log().register_usb(usb);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        let mut motor_en = gpio1.output(pins.p23);
        let mut kill_n = gpio2.output(pins.p36);

        // Motor One Interface
        let mut motor_one_uart = board::lpuart(
            lpuart6,
            pins.p1,
            pins.p0,
            115_200
        );
        motor_one_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            uart.set_parity(None);
        });

        // Motor two interface
        let mut motor_two_uart = board::lpuart(
            lpuart4,
            pins.p8,
            pins.p7,
            115_200,
        );
        motor_two_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            uart.set_parity(None);
        });

        // Motor three interface
        let mut motor_three_uart = board::lpuart(
            unsafe { ral::lpuart::LPUART1::instance() },
            pins.p24,
            pins.p25,
            115_200
        );
        motor_three_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            uart.set_parity(None);
        });

        // Motor four interface
        let mut motor_four_uart = board::lpuart(
            unsafe { ral::lpuart::LPUART7::instance() },
            pins.p29,
            pins.p28,
            115_200,
        );
        motor_four_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            uart.set_parity(None);
        });

        // Dribbler interface
        let mut dribbler_uart = board::lpuart(
            lpuart8,
            pins.p20,
            pins.p21,
            115_200
        );
        dribbler_uart.disable(|uart| {
            uart.disable_fifo(lpuart::Direction::Rx);
            uart.disable_fifo(lpuart::Direction::Tx);
            uart.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            uart.set_parity(None);
        });

        motor_en.set_high().unwrap();
        kill_n.set_high().unwrap();

        move_motors::spawn().ok();
        relay_motor_speeds::spawn().ok();

        (
            Shared {
                motor_speeds: [0i32; 4],
                motor_one_uart,
                motor_two_uart,
                motor_three_uart,
                motor_four_uart,
                dribbler_uart,
            },
            Local {
                _motor_en: motor_en,
                _kill_n: kill_n,
            }
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
            motor_one_uart,
            motor_two_uart,
            motor_three_uart,
            motor_four_uart,
            dribbler_uart,
        ],
        local = [iteration: u32 = 0, current_motor: u8 = 0],
        priority = 1,
    )]
    async fn move_motors(mut ctx: move_motors::Context) {
        if *ctx.local.iteration % 60 == 0 {
            *ctx.local.current_motor = (*ctx.local.current_motor + 1) % 5;
        }

        let commands = get_motor_commands(*ctx.local.current_motor);

        // ctx.shared.motor_one_uart.lock(|uart| {
        //     uart.bwrite_all(&commands.0[0]).unwrap();
        // });
        let command = MotorCommand::Move { ticks_per_second: 0 };
        let mut buffer = [0u8; MOTOR_COMMAND_SIZE];
        command.pack(&mut buffer).unwrap();
        ctx.shared.motor_two_uart.lock(|uart| {
            uart.bwrite_all(&buffer).unwrap();
        });
        // ctx.shared.motor_three_uart.lock(|uart| {
        //     uart.bwrite_all(&commands.0[2]).unwrap();
        // });
        // ctx.shared.motor_four_uart.lock(|uart| {
        //     uart.bwrite_all(&commands.0[3]).unwrap();
        // });
        // ctx.shared.dribbler_uart.lock(|uart| {
        //     uart.bwrite_all(&commands.1).unwrap();
        // });

        *ctx.local.iteration = ctx.local.iteration.wrapping_add(1);

        wait_to_move_motors::spawn().ok();
    }

    #[task(priority = 1)]
    async fn wait_to_move_motors(_ctx: wait_to_move_motors::Context) {
        Systick::delay((1_000u32 / 60u32).millis()).await;

        move_motors::spawn().ok();
    }

    #[task(
        shared = [motor_speeds],
        priority = 1,
    )]
    async fn relay_motor_speeds(mut ctx: relay_motor_speeds::Context) {
        let speeds = ctx.shared.motor_speeds.lock(|speeds| *speeds);

        log::info!("Motor Speeds: {:?}", speeds);

        wait_to_report_motor_speeds::spawn().ok();
    }

    #[task(priority = 1)]
    async fn wait_to_report_motor_speeds(_ctx: wait_to_report_motor_speeds::Context) {
        Systick::delay(500u32.millis()).await;

        relay_motor_speeds::spawn().ok();
    }

    // #[task(
    //     shared = [motor_one_uart, motor_speeds],
    //     local = [buffer: [u8; MOTOR_MOVE_RESPONSE_SIZE] = [0u8; MOTOR_MOVE_RESPONSE_SIZE]],
    //     priority = 2,
    //     binds = LPUART6
    // )]
    // fn receive_motor_one_data(mut ctx: receive_motor_one_data::Context) {
    //     let success = ctx.shared.motor_one_uart.lock(|uart| {
    //         let mut success = true;
    //         if uart.status().contains(lpuart::Status::RECEIVE_FULL) {
    //             for i in 0..MOTOR_MOVE_RESPONSE_SIZE {
    //                 match uart.read() {
    //                     Ok(data) => ctx.local.buffer[i] = data,
    //                     Err(_err) => {
    //                         success = false;
    //                         break;
    //                     },
    //                 }
    //             }
    //         }
    //         uart.clear_status(lpuart::Status::all());
    //         success
    //     });

    //     if !success {
    //         log::error!("FUCK");
    //     }

    //     let move_response = MotorMoveResponse::unpack(ctx.local.buffer).unwrap();
    //     ctx.shared.motor_speeds.lock(|speeds| {
    //         speeds[0] = move_response.ticks_per_second;
    //     });
    // }

    #[task(
        shared = [motor_two_uart, motor_speeds],
        local = [buffer: [u8; MOTOR_MOVE_RESPONSE_SIZE] = [0u8; MOTOR_MOVE_RESPONSE_SIZE]],
        priority = 2,
        binds = LPUART4
    )]
    fn receive_motor_two_data(mut ctx: receive_motor_two_data::Context) {
        let success = ctx.shared.motor_two_uart.lock(|uart| {
            let mut success = true;
            if uart.status().contains(lpuart::Status::RECEIVE_FULL) {
                for i in 0..MOTOR_MOVE_RESPONSE_SIZE {
                    match uart.read() {
                        Ok(data) => ctx.local.buffer[i] = data,
                        Err(_err) => {
                            success = false;
                            break;
                        },
                    }
                }
            }
            uart.clear_status(lpuart::Status::all());
            success
        });

        if !success {
            log::error!("FUCK");
        }

        let move_response = MotorMoveResponse::unpack(ctx.local.buffer).unwrap();
        ctx.shared.motor_speeds.lock(|speeds| {
            speeds[0] = move_response.ticks_per_second;
        });
    }

    // #[task(
    //     shared = [motor_three_uart, motor_speeds],
    //     local = [buffer: [u8; MOTOR_MOVE_RESPONSE_SIZE] = [0u8; MOTOR_MOVE_RESPONSE_SIZE]],
    //     priority = 2,
    //     binds = LPUART1
    // )]
    // fn receive_motor_three_data(mut ctx: receive_motor_three_data::Context) {
    //     let success = ctx.shared.motor_three_uart.lock(|uart| {
    //         let mut success = true;
    //         if uart.status().contains(lpuart::Status::RECEIVE_FULL) {
    //             for i in 0..MOTOR_MOVE_RESPONSE_SIZE {
    //                 match uart.read() {
    //                     Ok(data) => ctx.local.buffer[i] = data,
    //                     Err(_err) => {
    //                         success = false;
    //                         break;
    //                     },
    //                 }
    //             }
    //         }
    //         uart.clear_status(lpuart::Status::all());
    //         success
    //     });

    //     if !success {
    //         log::error!("FUCK");
    //     }

    //     let move_response = MotorMoveResponse::unpack(ctx.local.buffer).unwrap();
    //     ctx.shared.motor_speeds.lock(|speeds| {
    //         speeds[0] = move_response.ticks_per_second;
    //     });
    // }

    // #[task(
    //     shared = [motor_four_uart, motor_speeds],
    //     local = [buffer: [u8; MOTOR_MOVE_RESPONSE_SIZE] = [0u8; MOTOR_MOVE_RESPONSE_SIZE]],
    //     priority = 2,
    //     binds = LPUART7
    // )]
    // fn receive_motor_four_data(mut ctx: receive_motor_four_data::Context) {
    //     let success = ctx.shared.motor_four_uart.lock(|uart| {
    //         let mut success = true;
    //         if uart.status().contains(lpuart::Status::RECEIVE_FULL) {
    //             for i in 0..MOTOR_MOVE_RESPONSE_SIZE {
    //                 match uart.read() {
    //                     Ok(data) => ctx.local.buffer[i] = data,
    //                     Err(_err) => {
    //                         success = false;
    //                         break;
    //                     },
    //                 }
    //             }
    //         }
    //         uart.clear_status(lpuart::Status::all());
    //         success
    //     });

    //     if !success {
    //         log::error!("FUCK");
    //     }

    //     let move_response = MotorMoveResponse::unpack(ctx.local.buffer).unwrap();
    //     ctx.shared.motor_speeds.lock(|speeds| {
    //         speeds[0] = move_response.ticks_per_second;
    //     });
    // }

    // #[task(
    //     shared = [dribbler_uart, motor_speeds],
    //     local = [buffer: [u8; DRIBBLER_RESPONSE_SIZE] = [0u8; DRIBBLER_RESPONSE_SIZE]],
    //     priority = 2,
    //     binds = LPUART8
    // )]
    // fn receive_dribbler_data(mut ctx: receive_dribbler_data::Context) {
    //     let success = ctx.shared.dribbler_uart.lock(|uart| {
    //         let mut success = true;
    //         if uart.status().contains(lpuart::Status::RECEIVE_FULL) {
    //             for i in 0..DRIBBLER_RESPONSE_SIZE {
    //                 match uart.read() {
    //                     Ok(data) => ctx.local.buffer[i] = data,
    //                     Err(_err) => {
    //                         success = false;
    //                         break;
    //                     },
    //                 }
    //             }
    //         }
    //         uart.clear_status(lpuart::Status::all());
    //         success
    //     });

    //     if !success {
    //         log::error!("FUCK");
    //     }
    // }
}
