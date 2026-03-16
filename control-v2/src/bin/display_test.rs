#![no_std]
#![no_main]

extern crate alloc;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use core::{default::Default, mem::MaybeUninit};
use embassy_executor::Spawner;
use embassy_stm32::i2c;
use embassy_stm32::{
    adc::Adc,
    bind_interrupts,
    gpio::{Level, Output, Speed},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::PubSubChannel};
use embassy_time::{Duration, Timer};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};
use static_cell::StaticCell;

use common::{
    dribbler::DribblerCommand,
    motor::{MotorCommand, MotorMoveResponse},
};
use control_v2::{battery, gpio, graphics, radio::control_command::ControlMessage};

// Allocator
use embedded_alloc::LlffHeap;
#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();
const HEAP_SIZE: usize = 1024 * 8;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

/// The channel for publishing ControlMessages received from the radio
pub static COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, ControlMessage, 4, 2, 1>> =
    StaticCell::new();
/// The channel for publishing kicker states
pub static KICKER_STATE_CHANNEL: StaticCell<
    PubSubChannel<NoopRawMutex, kicker_controller::KickerState, 4, 2, 1>,
> = StaticCell::new();
/// The channel for publishing imu data
pub static IMU_DATA_CHANNEL: StaticCell<
    PubSubChannel<NoopRawMutex, control_v2::imu::ImuData, 4, 1, 1>,
> = StaticCell::new();
/// The channel for publishing motor 1 commands
pub static MOTOR1_COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorCommand, 4, 1, 1>> =
    StaticCell::new();
/// The channel for publishing motor 1 statuses
pub static MOTOR1_STATUS_CHANNEL: StaticCell<
    PubSubChannel<NoopRawMutex, MotorMoveResponse, 4, 1, 1>,
> = StaticCell::new();
/// The channel for publishing motor 2 commands
pub static MOTOR2_COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorCommand, 4, 1, 1>> =
    StaticCell::new();
/// The channel for publishing motor 2 statuses
pub static MOTOR2_STATUS_CHANNEL: StaticCell<
    PubSubChannel<NoopRawMutex, MotorMoveResponse, 4, 1, 1>,
> = StaticCell::new();
/// The channel for publishing motor 3 commands
pub static MOTOR3_COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorCommand, 4, 1, 1>> =
    StaticCell::new();
/// The channel for publishing motor 3 statuses
pub static MOTOR3_STATUS_CHANNEL: StaticCell<
    PubSubChannel<NoopRawMutex, MotorMoveResponse, 4, 1, 1>,
> = StaticCell::new();
/// The channel for publishing motor 4 commands
pub static MOTOR4_COMMAND_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, MotorCommand, 4, 1, 1>> =
    StaticCell::new();
/// The channel for publishing motor 4 statuses
pub static MOTOR4_STATUS_CHANNEL: StaticCell<
    PubSubChannel<NoopRawMutex, MotorMoveResponse, 4, 1, 1>,
> = StaticCell::new();
/// The channel for publishing dribbler commands
pub static DRIBBLER_COMMAND_CHANNEL: StaticCell<
    PubSubChannel<NoopRawMutex, DribblerCommand, 4, 1, 1>,
> = StaticCell::new();
/// The channel for turning off the robot
pub static POWER_OFF_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, (), 4, 2, 2>> =
    StaticCell::new();
/// The channel for publishing the current battery voltage
pub static BATTERY_VOLTAGE_CHANNEL: StaticCell<PubSubChannel<NoopRawMutex, f32, 4, 2, 1>> =
    StaticCell::new();

// Gpio Interrupts
bind_interrupts!(pub struct GpioIrqs {
    EXTI0 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI0>;
    EXTI1 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI1>;
    EXTI15_10 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI15_10>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    unsafe {
        #[allow(static_mut_refs)]
        HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
    }

    let p = embassy_stm32::init(Default::default());

    // Enable the motor board
    let mut kill_n = Output::new(p.PB2, Level::High, Speed::Low);
    kill_n.set_high();
    let mut motor_en = Output::new(p.PC1, Level::High, Speed::Low);
    motor_en.set_high();
    let power_off_irq = embassy_stm32::exti::ExtiInput::new(
        p.PB1,
        p.EXTI1,
        embassy_stm32::gpio::Pull::None,
        GpioIrqs,
    );

    // Initialize the ADC
    let adc = Adc::new(p.ADC1);
    let battery_pin = p.PB0;

    // Initialize the display
    let display_i2c = i2c::I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        graphics::DisplayIrqs,
        p.DMA1_CH2,
        p.DMA1_CH3,
        i2c::Config::default(),
    );
    let mut display = Ssd1306::new(
        I2CDisplayInterface::new(display_i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    if let Err(_err) = display.init() {
        loop {
            error!("Error Initializing Display");
            Timer::after(Duration::from_secs(5)).await;
        }
    }

    // Create communication channels
    let kicker_channel = KICKER_STATE_CHANNEL.init(PubSubChannel::<
        NoopRawMutex,
        kicker_controller::KickerState,
        4,
        2,
        1,
    >::new());
    let power_off_channel =
        POWER_OFF_CHANNEL.init(PubSubChannel::<NoopRawMutex, (), 4, 2, 2>::new());
    let battery_voltage_channel =
        BATTERY_VOLTAGE_CHANNEL.init(PubSubChannel::<NoopRawMutex, f32, 4, 2, 1>::new());

    // Spawn the power switch task
    spawner
        .spawn(gpio::power_switch(
            power_off_irq,
            power_off_channel.publisher().unwrap(),
            kill_n,
        ))
        .unwrap();

    // Spawn the battery monitoring task
    spawner
        .spawn(battery::monitor_battery_voltage(
            adc,
            battery_pin,
            battery_voltage_channel.publisher().unwrap(),
            power_off_channel.publisher().unwrap(),
        ))
        .unwrap();

    // Spawn the display task
    spawner
        .spawn(graphics::display_robot_status(
            display,
            kicker_channel.subscriber().unwrap(),
            battery_voltage_channel.subscriber().unwrap(),
        ))
        .unwrap();
}
