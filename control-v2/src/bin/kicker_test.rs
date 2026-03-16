#![no_std]
#![no_main]

extern crate alloc;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use core::{cell::RefCell, default::Default, mem::MaybeUninit};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
};
use embassy_stm32::{i2c, spi};
use embassy_sync::{
    blocking_mutex::{NoopMutex, raw::NoopRawMutex},
    pubsub::PubSubChannel,
};
use embassy_time::{Duration, Timer};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};
use static_cell::StaticCell;

use common::{
    dribbler::DribblerCommand,
    motor::{MotorCommand, MotorMoveResponse},
};
use control_v2::{graphics, kicker, radio::control_command::ControlMessage, utils};
use kicker_controller::Kicker;

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

    // Initialize the kicker controller
    let kicker_spi = spi::Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi::Config::default());
    let kicker_spi = kicker::KICKER_SPI.init(NoopMutex::new(RefCell::new(kicker_spi)));
    let kicker_csn = Output::new(p.PC4, Level::High, Speed::High);
    let kicker_spi = SpiDevice::new(kicker_spi, kicker_csn);
    let kicker_reset = Output::new(p.PC5, Level::High, Speed::Low);
    let kicker: Kicker<
        Output<'_>,
        SpiDevice<
            '_,
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            spi::Spi<'static, embassy_stm32::mode::Blocking, spi::mode::Master>,
            Output<'_>,
        >,
    > = Kicker::new(kicker_spi, kicker_reset);

    // Create communication channels
    let command_channel =
        COMMAND_CHANNEL.init(PubSubChannel::<NoopRawMutex, ControlMessage, 4, 2, 1>::new());
    let kicker_channel = KICKER_STATE_CHANNEL.init(PubSubChannel::<
        NoopRawMutex,
        kicker_controller::KickerState,
        4,
        2,
        1,
    >::new());
    let power_off_channel =
        POWER_OFF_CHANNEL.init(PubSubChannel::<NoopRawMutex, (), 4, 2, 2>::new());

    // Spawn the Kicker Testing Task
    spawner
        .spawn(utils::kicker_testing_task(
            command_channel.publisher().unwrap(),
        ))
        .unwrap();

    // Spawn the Kicker Control Task
    spawner
        .spawn(kicker::service_kicker(
            kicker,
            command_channel.subscriber().unwrap(),
            kicker_channel.publisher().unwrap(),
            power_off_channel.subscriber().unwrap(),
        ))
        .unwrap();
}
