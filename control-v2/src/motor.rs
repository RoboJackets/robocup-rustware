//!
//! Tasks for interacting with the motors connected to the motor board
//! 

use embassy_stm32::usart::Uart;
use embassy_stm32::{bind_interrupts, peripherals, usart, mode};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pubsub::{Publisher, Subscriber, WaitResult};
use common::{motor::{MotorCommand, MotorMoveResponse}, dribbler::DribblerCommand, packing::Packable};

pub const MOTOR_COMMAND_FREQUENCY_HZ: u64 = 60;

bind_interrupts!(pub struct Motor1Irqs {
    UART7 => usart::InterruptHandler<peripherals::UART7>;
});

bind_interrupts!(pub struct Motor2Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

bind_interrupts!(pub struct Motor3Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

bind_interrupts!(pub struct Motor4Irqs {
    USART6 => usart::InterruptHandler<peripherals::USART6>;
});

bind_interrupts!(pub struct DribblerIrqs {
    UART4 => usart::InterruptHandler<peripherals::UART4>;
});

#[embassy_executor::task]
pub async fn command_motor_1(
    mut uart: Uart<'static, mode::Async>,
    mut command_receiver: Subscriber<'static, NoopRawMutex, MotorCommand, 4, 1, 1>,
    status_sender: Publisher<'static, NoopRawMutex, MotorMoveResponse, 4, 1, 1>,
) {
    let mut read_buffer = [0u8; 5];
    loop {
        if let WaitResult::Message(command) = command_receiver.next_message().await {
            command.pack(&mut read_buffer).unwrap();
            uart.write(&read_buffer).await.unwrap();
            uart.read(&mut read_buffer).await.unwrap();

            let response = MotorMoveResponse::unpack(&read_buffer).unwrap();
            status_sender.publish_immediate(response);
        }
    }
}

#[embassy_executor::task]
pub async fn command_motor_2(
    mut uart: Uart<'static, mode::Async>,
    mut command_receiver: Subscriber<'static, NoopRawMutex, MotorCommand, 4, 1, 1>,
    status_sender: Publisher<'static, NoopRawMutex, MotorMoveResponse, 4, 1, 1>,
) {
    let mut read_buffer = [0u8; 5];
    loop {
        if let WaitResult::Message(command) = command_receiver.next_message().await {
            command.pack(&mut read_buffer).unwrap();
            uart.write(&read_buffer).await.unwrap();
            uart.read(&mut read_buffer).await.unwrap();

            let response = MotorMoveResponse::unpack(&read_buffer).unwrap();
            status_sender.publish_immediate(response);
        }
    }
}

#[embassy_executor::task]
pub async fn command_motor_3(
    mut uart: Uart<'static, mode::Async>,
    mut command_receiver: Subscriber<'static, NoopRawMutex, MotorCommand, 4, 1, 1>,
    status_sender: Publisher<'static, NoopRawMutex, MotorMoveResponse, 4, 1, 1>,
) {
    let mut read_buffer = [0u8; 5];
    loop {
        if let WaitResult::Message(command) = command_receiver.next_message().await {
            command.pack(&mut read_buffer).unwrap();
            uart.write(&read_buffer).await.unwrap();
            uart.read(&mut read_buffer).await.unwrap();

            let response = MotorMoveResponse::unpack(&read_buffer).unwrap();
            status_sender.publish_immediate(response);
        }
    }
}

#[embassy_executor::task]
pub async fn command_motor_4(
    mut uart: Uart<'static, mode::Async>,
    mut command_receiver: Subscriber<'static, NoopRawMutex, MotorCommand, 4, 1, 1>,
    status_sender: Publisher<'static, NoopRawMutex, MotorMoveResponse, 4, 1, 1>,
) {
    let mut read_buffer = [0u8; 5];
    loop {
        if let WaitResult::Message(command) = command_receiver.next_message().await {
            command.pack(&mut read_buffer).unwrap();
            uart.write(&read_buffer).await.unwrap();
            uart.read(&mut read_buffer).await.unwrap();

            let response = MotorMoveResponse::unpack(&read_buffer).unwrap();
            status_sender.publish_immediate(response);
        }
    }
}

#[embassy_executor::task]
pub async fn command_dribbler(
    mut uart: Uart<'static, mode::Async>,
    mut command_receiver: Subscriber<'static, NoopRawMutex, DribblerCommand, 4, 1, 1>,
) {
    let mut read_buffer = [0u8; 5];
    loop {
        if let WaitResult::Message(command) = command_receiver.next_message().await {
            command.pack(&mut read_buffer).unwrap();
            uart.write(&read_buffer).await.unwrap();
            uart.read(&mut read_buffer).await.unwrap();
        }
    }
}
